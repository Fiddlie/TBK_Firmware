/*
 * gap.c – minimal BLE GAP helper for GloveRaw
 *
 *  • Advertise Flags + Complete Name (<31 B)
 *  • Scan-response adds 16-bit UUID list + URI
 *  • Restarts advertising on disconnect / completed event
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "gap.h"
#include "common.h"     /* defines TAG and DEVICE_NAME */
#include "gatt_svc.h"   /* for gatt_svr_subscribe_cb() */

static const char *TAG = "GAP";

/* ───── forward decl. ───── */
static int  gap_event_handler(struct ble_gap_event *event, void *arg);
static void start_advertising(void);

/* ───── globals ───── */
static uint8_t own_addr_type;
static uint8_t addr_val[6];

/* Optional URI that Android can display in scan view */
static const uint8_t esp_uri[] = {
    BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e','s','p','r','e','s','s','i','f','.','c','o','m'
};

/* 16-bit Service UUIDs to expose in scan-response */
static const ble_uuid16_t adv_uuids[] = {
    BLE_UUID16_INIT(0x180D),  /* Heart-Rate */
    BLE_UUID16_INIT(0x1815),  /* Automation-IO */
};

/* ─────────────────────────────────────────────────────────── */

static void format_addr(char *buf, const uint8_t a[6])
{
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            a[0], a[1], a[2], a[3], a[4], a[5]);
}

/* ---------- advertising ---------- */
static void start_advertising(void)
{
    /* 1. Advertising packet: FLAGS + COMPLETE_NAME */
    struct ble_hs_adv_fields adv = { 0 };
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name = (uint8_t *)DEVICE_NAME;
    adv.name_len = strlen(DEVICE_NAME);
    adv.name_is_complete = 1;
    ESP_ERROR_CHECK(ble_gap_adv_set_fields(&adv));

    /* 2. Scan-response: UUID list + URI  (≤31 B) */
    struct ble_hs_adv_fields rsp = { 0 };
    rsp.uuids16            = (ble_uuid16_t *)adv_uuids;
    rsp.num_uuids16        = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    rsp.uuids16_is_complete = 1;
    rsp.uri                = (uint8_t *)esp_uri;
    rsp.uri_len            = sizeof(esp_uri);
    ESP_ERROR_CHECK(ble_gap_adv_rsp_set_fields(&rsp));

    /* 3. Start advertising with our GAP event-handler */
    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min  = 0x20,      /* 20 ms */
        .itvl_max  = 0x40
    };
    ESP_ERROR_CHECK(
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &p, gap_event_handler, NULL));

    ESP_LOGI(TAG, "Advertising started (%s)", DEVICE_NAME);
}

/* ---------- GAP event callback ---------- */
static int gap_event_handler(struct ble_gap_event *e, void *arg)
{
    switch (e->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connect %s; status=%d",
                 e->connect.status ? "failed" : "OK",
                 e->connect.status);
        if (e->connect.status) start_advertising();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", e->disconnect.reason);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "adv complete; reason=%d", e->adv_complete.reason);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        return 0;

    default:
        return 0;
    }
}

/* ---------- public helpers ---------- */
void adv_init(void)
{
    /* Ensure identity address exists */
    ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &own_addr_type));
    ESP_ERROR_CHECK(ble_hs_id_copy_addr(own_addr_type, addr_val, NULL));

    char s[18]; format_addr(s, addr_val);
    ESP_LOGI(TAG, "device address: %s", s);

    start_advertising();
}

int gap_init(void)
{
    ble_svc_gap_init();
    return ble_svc_gap_device_name_set(DEVICE_NAME);
}
