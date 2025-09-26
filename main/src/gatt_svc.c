/*
 *   Service 0xFFF0
 *   Char    0xFFF1  – 20-byte notify payload (IMU raw)
 */

#include "gatt_svc.h"
#include "esp_log.h"
#include "services/gatt/ble_svc_gatt.h"    

static const char *TAG = "GATT_SVC";

/* Static UUIDs */
static const ble_uuid16_t UUID_SVC  = BLE_UUID16_INIT(0xFFF0);
static const ble_uuid16_t UUID_CHAR = BLE_UUID16_INIT(0xFFF1);
uint16_t g_raw_val_handle = 0;

static int dummy_access(uint16_t conn_handle,
             uint16_t attr_handle,
             struct ble_gatt_access_ctxt *ctxt,
             void *arg)
{
    return 0;            
}

/* GATT definition table */
static const struct ble_gatt_svc_def RAW_SVCS[] = {
    {
        .type  = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid  = &UUID_SVC.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {   
                .uuid       = &UUID_CHAR.u,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb  = dummy_access,
                .val_handle = &g_raw_val_handle,
            },
            { 0 }                          
        },
    },
    { 0 }                              
};

void gatt_svc_register(void)
{
    static bool gatt_svc_registered = false;

    /* Mandatory Generic Attribute service (0x1801) */
    ble_svc_gatt_init();

    if (!gatt_svc_registered) {
        ESP_ERROR_CHECK( ble_gatts_count_cfg(RAW_SVCS) );
        ESP_ERROR_CHECK( ble_gatts_add_svcs (RAW_SVCS) );
        gatt_svc_registered = true;
    }

    ESP_LOGI(TAG, "GloveRaw service registered; val_handle=0x%04X", g_raw_val_handle);
}