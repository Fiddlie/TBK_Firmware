/***********************************************************************
 * sensor_task.c  – Fixed version with connection awareness
 ***********************************************************************/
#include "sensor_task.h"
#include "imu.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NimBLE */
#include "host/ble_hs.h"
#include "host/ble_gap.h"

/* characteristic handle exported by gatt_svc.c */
#include "gatt_svc.h"

/*--------------------------------------------------------------------
 * build config
 *-------------------------------------------------------------------*/
#define TAG                "SENSOR"
#define TASK_PRIO          5
#define TASK_STACK_WORDS   (3 * 1024)

/*--------------------------------------------------------------------
 * globals
 *-------------------------------------------------------------------*/
static TaskHandle_t s_task_handle;
static uint32_t s_notify_count = 0;
static uint32_t s_read_count = 0;
static uint32_t s_read_success_count = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/*--------------------------------------------------------------------
 * helper – build 20-byte packet
 *-------------------------------------------------------------------*/
static void fill_packet(uint8_t pkt[20],
                        const imu_raw_t *r,
                        uint32_t t_ms)
{
    int16_t *p16 = (int16_t *)pkt;

    /* first 12 bytes = six int16_t   */
    p16[0] = r->gx;  p16[1] = r->gy;  p16[2] = r->gz;
    p16[3] = r->ax;  p16[4] = r->ay;  p16[5] = r->az;

    uint32_t *p32 = (uint32_t *)(pkt + 12);       /* next 4 bytes time-stamp */
    *p32 = t_ms;

    /* pad the remaining 4 bytes */
    memset(pkt + 16, 0, 4);
}

/*--------------------------------------------------------------------
 * helper - check if any device is connected
 *-------------------------------------------------------------------*/
static bool is_connected(void)
{
    struct ble_gap_conn_desc desc;
    int rc;
    
    /* Try to find any active connection */
    for (uint16_t handle = 0; handle < 0xFFFF; handle++) {
        rc = ble_gap_conn_find(handle, &desc);
        if (rc == 0) {
            s_conn_handle = handle;
            return true;
        }
    }
    
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    return false;
}

/*--------------------------------------------------------------------
 * main task
 *-------------------------------------------------------------------*/
static void sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor task starting...");

    /* Wait a bit for BLE stack to fully initialize */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /*--- init IMU driver ------------------------------------------------*/
    esp_err_t err = imu_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed with error: 0x%x - task will exit", err);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "IMU initialized successfully");

    uint32_t t_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    TickType_t last_log_time = xTaskGetTickCount();
    bool was_connected = false;

    for (;;)
    {
        /* Check connection status */
        bool connected = is_connected();
        
        if (connected && !was_connected) {
            ESP_LOGI(TAG, "Device connected, starting notifications");
            was_connected = true;
        } else if (!connected && was_connected) {
            ESP_LOGI(TAG, "Device disconnected, stopping notifications");
            was_connected = false;
        }

        /* Only read and send data if connected */
        if (connected) {
            s_read_count++;
            
            imu_raw_t raw;
            if (imu_read_raw(&raw))
            {
                s_read_success_count++;
                
                uint8_t pkt[20];
                uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - t_start;
                fill_packet(pkt, &raw, t_ms);

                /* Log data every second */
                if (xTaskGetTickCount() - last_log_time > pdMS_TO_TICKS(1000)) {
                    ESP_LOGI(TAG, "IMU: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d", 
                             raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz);
                    ESP_LOGI(TAG, "Stats: reads=%lu success=%lu notifies=%lu handle=0x%04x conn=0x%04x",
                             s_read_count, s_read_success_count, s_notify_count, 
                             g_raw_val_handle, s_conn_handle);
                    last_log_time = xTaskGetTickCount();
                }

                /* alloc mbuf & copy flat data --------------------------------*/
                struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(pkt));
                if (om) {
                    /* Send notification to specific connection */
                    int rc = ble_gatts_notify_custom(s_conn_handle,
                                                     g_raw_val_handle, om);
                    if (rc == 0) {
                        s_notify_count++;
                    } else if (rc != BLE_HS_ENOTCONN) {
                        /* Only log errors that aren't "not connected" */
                        ESP_LOGW(TAG, "notify error=%d", rc);
                    }
                } else {
                    ESP_LOGW(TAG, "mbuf allocation failed");
                }
            }
        }

        /* 10 ms delay → 100 Hz */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*--------------------------------------------------------------------
 * public factory
 *-------------------------------------------------------------------*/
void sensor_task_create(void)
{
    if (s_task_handle) {
        ESP_LOGW(TAG, "Sensor task already created");
        return;
    }

    ESP_LOGI(TAG, "Creating sensor task...");
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        TASK_STACK_WORDS,
        NULL,
        TASK_PRIO,
        &s_task_handle,
        1);  /* pin on core 1 */
        
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
    } else {
        ESP_LOGI(TAG, "Sensor task created successfully");
    }
}