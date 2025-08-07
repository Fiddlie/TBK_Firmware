/*  main/main.c ─ minimal NimBLE demo  */

#include "common.h"          /* TAG, DEVICE_NAME */
#include "gap.h"
#include "gatt_svc.h"
#include "sensor_task.h"
#include "nvs_flash.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* ------------------------------------------------------------------ */
#define TAG "MAIN"            /* ← restore full macro name */

/* Forward */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_task(void *arg);
static void nimble_host_cfg_init(void);

/* ------------------------------------------------------------------ */
static void on_stack_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
}

static void on_stack_sync(void)
{
    adv_init();                          /* start advertising */
}

static void nimble_host_task(void *arg)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();                   /* never returns */
}

/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* NVS (required by controller) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    sensor_task_create();                /* IMU producer      */

    /* NimBLE low-level */
    ESP_ERROR_CHECK(nimble_port_init());
    gap_init();                          /* device name etc.  */
    gatt_svc_register();                 /* our custom svc    */
    nimble_host_cfg_init();              /* callbacks / store */

    /* Start host thread */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "setup complete");
}

/* ------------------------------------------------------------------ */
static void nimble_host_cfg_init(void)
{
    extern void ble_store_config_init(void);

    ble_hs_cfg.reset_cb          = on_stack_reset;
    ble_hs_cfg.sync_cb           = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = NULL;

    ble_store_config_init();
}
