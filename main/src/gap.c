#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "ble_common.h"
#include "gap.h"
#include "common.h"
#include "gatt_svc.h"
#include "battery.h"
#include "sensor_task.h"

static const char *TAG = "GAP";

/* Forward declarations */
static int  gap_event_handler(struct ble_gap_event *event, void *arg);
static void start_advertising(void);

static uint8_t own_addr_type;
static uint8_t addr_val[6];

static const uint8_t esp_uri[] = {
    BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'T','B','K','B','O','X','I','N','G','.'
};

/* 16-bit Service UUIDs to expose in scan-response */
static const ble_uuid16_t adv_uuids[] = {
    BLE_UUID16_INIT(0x180D),
    BLE_UUID16_INIT(0x1815),
};

/* Helper functions */
static void format_addr(char *buf, const uint8_t a[6]) {
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            a[0], a[1], a[2], a[3], a[4], a[5]);
}

static void start_advertising(void)
{
    int rc;
    
    /* Advertising packet: FLAGS + COMPLETE_NAME */
    struct ble_hs_adv_fields adv;
    memset(&adv, 0, sizeof(adv));
    
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name = (uint8_t *)DEVICE_NAME;
    adv.name_len = strlen(DEVICE_NAME);
    adv.name_is_complete = 1;
    
    rc = ble_gap_adv_set_fields(&adv);
    
    /* Scan-response: UUID list + URI */
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    
    rsp.uuids16 = (ble_uuid16_t *)adv_uuids;
    rsp.num_uuids16 = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    rsp.uuids16_is_complete = 1;
    rsp.uri = (uint8_t *)esp_uri;
    rsp.uri_len = sizeof(esp_uri);
    
    rc = ble_gap_adv_rsp_set_fields(&rsp);

    /* Start advertising with proper intervals */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    /* 800 * 0.625ms = 500ms interval for power saving */
    adv_params.itvl_min = 800;
    adv_params.itvl_max = 800;
    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    
    if (rc == 0) {
        ESP_LOGI(TAG, "Advertising started! Device name: %s", DEVICE_NAME);
        battery_set_ble_state(BLE_STATE_ADVERTISING);
        ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    } 
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;
    
    ESP_LOGI(TAG, "GAP event type: %d", event->type);
    
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);

            /* Request faster connection parameters for real-time data */
            struct ble_gap_upd_params params;
            params.itvl_min = 6;   /* 6 * 1.25ms = 7.5ms */
            params.itvl_max = 12;  /* 12 * 1.25ms = 15ms */
            params.latency = 0;    /* No slave latency for real-time response */
            params.supervision_timeout = 200;  /* 200 * 10ms = 2 seconds */
            params.min_ce_len = 0;
            params.max_ce_len = 0;
            
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            battery_set_ble_state(BLE_STATE_CONNECTED);
            sensor_task_set_connected(event->connect.conn_handle, true);
        } else {
            /* Connection failed; resume advertising */
            start_advertising();
        }
        return 0;
        
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        battery_set_ble_state(BLE_STATE_DISCONNECTED);
        sensor_task_set_connected(0, false);
        start_advertising();
        return 0;
        
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
        start_advertising();
        return 0;
        
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe event; attr_handle=%d", event->subscribe.attr_handle);
        return 0;
        
    case BLE_GAP_EVENT_CONN_UPDATE:
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc == 0) {
            ESP_LOGI(TAG, "Connection updated: interval=%.1fms, latency=%d, timeout=%dms",
                     desc.conn_itvl * 1.25f,
                     desc.conn_latency,
                     desc.supervision_timeout * 10);
        }
        return 0;
        
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        /* Accept any connection parameter update request from peer */
        ESP_LOGI(TAG, "Peer requesting connection update");
        return 0;
        
    default:
        return 0;
    }
}

/* Public functions */
void adv_init(void)
{
    int rc;
    
    ESP_LOGI(TAG, "Initializing advertising...");
    
    /* Ensure we have a proper address */
    rc = ble_hs_util_ensure_addr(0);
    
    /* Figure out address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    
    /* Get the address */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    char addr_str[18];
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "Device BLE address: %s", addr_str);

    vTaskDelay(pdMS_TO_TICKS(100));
    
    start_advertising();
}

int gap_init(void)
{
    int rc;
    
    ESP_LOGI(TAG, "Initializing GAP...");
    
    ble_svc_gap_init();
    
    /* Set device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name; rc=%d", rc);
        return rc;
    }
    
    ESP_LOGI(TAG, "Device name set to: %s", DEVICE_NAME);
    return 0;
}