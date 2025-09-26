/*
 * main/main.c - NimBLE demo with DFU support
 */

#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "sensor_task.h"
#include "dfu_service.h" 
#include "nvs_flash.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "battery.h"

/* ------------------------------------------------------------------ */
#define TAG "MAIN"

/* Forward */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_task(void *arg);
static void nimble_host_cfg_init(void);

/* ------------------------------------------------------------------ */
static void print_current_firmware_info(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
        ESP_LOGI(TAG, "Running from partition: %s", running->label);
    }
}

/* ------------------------------------------------------------------ */
static void on_stack_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
}

static void on_stack_sync(void)
{
    /* BLE stack is ready - start advertising */
    adv_init();
}

static void on_gatts_register(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registered characteristic %s with def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registered descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

static void nimble_host_task(void *arg)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();  /* never returns */
}

/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* Early LED initialization for button wake */
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    if (wakeup == ESP_SLEEP_WAKEUP_UNDEFINED ||
        wakeup == ESP_SLEEP_WAKEUP_GPIO ||
        wakeup == ESP_SLEEP_WAKEUP_EXT0 ||
        wakeup == ESP_SLEEP_WAKEUP_EXT1) {
        /* Quick LED setup without full battery init */
        extern void battery_early_led_init(void);
        battery_early_led_init();
    }

    /* Print firmware info on boot */
    print_current_firmware_info();

    /* NVS (required by controller and OTA) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Initialize DFU service before BLE stack */
    dfu_service_init();

    /* Initialize battery management first (includes I2C init) */
    ESP_LOGI(TAG, "Initializing battery management...");
    esp_err_t ret = battery_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Battery management init failed, continuing anyway");
    }
    
    /* Start sensor task after battery init */
    sensor_task_create();

    /* NimBLE low-level */
    ESP_ERROR_CHECK(nimble_port_init());
    
    /* Initialize GAP */
    gap_init();
    
    /* Register all GATT services BEFORE starting */
    gatt_svc_register();
    dfu_service_register();
    
    /* Configure host */
    nimble_host_cfg_init();

    /* Start host thread */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Setup complete - DFU enabled");
}

/* ------------------------------------------------------------------ */
static void nimble_host_cfg_init(void)
{
    extern void ble_store_config_init(void);

    ble_hs_cfg.reset_cb          = on_stack_reset;
    ble_hs_cfg.sync_cb           = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = on_gatts_register;

    ble_store_config_init();
}

