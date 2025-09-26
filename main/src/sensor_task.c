#include "sensor_task.h"
#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_common.h"
#include "gatt_svc.h"
#include "battery.h"

#define TAG                "SENSOR"
#define TASK_PRIO          7
#define TASK_STACK_WORDS   (3 * 1024)
#define LSM6DSO_TEST_CURRENT    0x20

static TaskHandle_t s_task_handle;
static uint32_t s_notify_count = 0;
static uint32_t s_read_count = 0;
static uint32_t s_read_success_count = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_is_connected = false;

static void fill_packet(uint8_t pkt[20],
                        const imu_raw_t *r,
                        uint32_t t_ms,
                        uint8_t battery_soc,
                        bool is_charging,
                        uint16_t battery_voltage_mv)
{
    int16_t *p16 = (int16_t *)pkt;

    /* First 12 bytes: six int16_t values */
    p16[0] = r->gx;  p16[1] = r->gy;  p16[2] = r->gz;
    p16[3] = r->ax;  p16[4] = r->ay;  p16[5] = r->az;

    /* Next 4 bytes: timestamp */
    uint32_t *p32 = (uint32_t *)(pkt + 12);
    *p32 = t_ms;

    /* byte 16: battery SOC (0-100%) */
    pkt[16] = battery_soc;
    
    /* byte 17: charging status (0=discharging, 1=charging) */
    pkt[17] = is_charging ? 1 : 0;
    
    /* bytes 18-19: battery voltage in mV (replaces padding) */
    uint16_t *p16_voltage = (uint16_t *)(pkt + 18);
    *p16_voltage = battery_voltage_mv;
}

/* Connection state callback (called from GAP event handler) */
void sensor_task_set_connected(uint16_t conn_handle, bool connected)
{
    if (connected) {
        s_conn_handle = conn_handle;
        s_is_connected = true;
        ESP_LOGI(TAG, "Connection established, handle=%d", conn_handle);
    } else {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_is_connected = false;
        ESP_LOGI(TAG, "Connection lost");
    }
}

/* Check if any device is connected */
static bool is_connected(void)
{
    return s_is_connected;
}

/* Main sensor task */
static void sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor task starting...");

    /* Wait a bit for BLE stack to fully initialize */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Initialize IMU driver */
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
    uint8_t battery_soc = 0;
    bool battery_charging = false;
    uint16_t battery_voltage_mv = 0;
    bool was_charging = false;
    /* Update charging config everey 60 seconds */
    static TickType_t s_last_ichg_write = 0;
    
    for (;;)
    {
        bool connected = is_connected();
        
        if (connected && !was_connected) {
            ESP_LOGI(TAG, "Device connected, starting notifications");
            was_connected = true;
        } else if (!connected && was_connected) {
            ESP_LOGI(TAG, "Device disconnected, stopping notifications");
            was_connected = false;
        }

        /* Update all battery info every packet for instant response */
        battery_charging = battery_is_charging();    
        battery_soc = battery_get_soc();              
        battery_voltage_mv = battery_get_voltage_mv(); // Update voltage every packet (10ms)

        /* Log charging state changes */
        if (battery_charging && !was_charging) {
            ESP_LOGI(TAG, "Charging started - stopping IMU reading");
            was_charging = true;
            
            uint8_t ichg = 0x20;
            i2c_write_reg(0x04, &ichg, 1);
            ESP_LOGI(TAG, "Current configured: 50 mA");
        } else if (!battery_charging && was_charging) {
            ESP_LOGI(TAG, "Charging stopped - resuming IMU reading");
            was_charging = false;
        }

        /* Only read and send data if connected */
        if (connected) {
            /* Check if we are reading IMU data */

            if (!battery_charging) {
                s_read_count++;
                
                imu_raw_t raw;
                if (imu_read_raw(&raw))
                {
                    s_read_success_count++;
                    
                    uint8_t pkt[20];
                    uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - t_start;
                    fill_packet(pkt, &raw, t_ms, battery_soc, battery_charging, battery_voltage_mv);

                    if (xTaskGetTickCount() - last_log_time > pdMS_TO_TICKS(1000)) {
                        ESP_LOGI(TAG, "IMU: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d", 
                                 raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz);
                        ESP_LOGI(TAG, "Battery: %d%% %s | Stats: reads=%lu success=%lu notifies=%lu",
                                 battery_soc, battery_charging ? "charging" : "discharging",
                                 s_read_count, s_read_success_count, s_notify_count);
                        last_log_time = xTaskGetTickCount();
                    }

                    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(pkt));
                    if (om) {
                        /* Send notification to specific connection */
                        int rc = ble_gatts_notify_custom(s_conn_handle,
                                                         g_raw_val_handle, om);
                        if (rc == 0) {
                            s_notify_count++;
                            battery_notify_data_transmit();
                        } else if (rc != BLE_HS_ENOTCONN) {
                            /* Only log errors that aren't "not connected" */
                            ESP_LOGW(TAG, "notify error=%d", rc);
                        }
                    } else {
                        ESP_LOGW(TAG, "mbuf allocation failed");
                    }
                }
            } else {
                imu_raw_t zero_raw = {0};  // All IMU values set to 0
                
                uint8_t pkt[20];
                uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS - t_start;
                fill_packet(pkt, &zero_raw, t_ms, battery_soc, battery_charging, battery_voltage_mv);
                
                /* Log charging status every second */
                if (xTaskGetTickCount() - last_log_time > pdMS_TO_TICKS(1000)) {
                    ESP_LOGI(TAG, "Charging mode - IMU stopped");
                    ESP_LOGI(TAG, "Battery: %d%% charging, voltage: %dmV",
                             battery_soc, battery_voltage_mv);
                    last_log_time = xTaskGetTickCount();
                }
                
                /* Still send BLE notifications with battery status */
                struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(pkt));
                if (om) {
                    int rc = ble_gatts_notify_custom(s_conn_handle,
                                                     g_raw_val_handle, om);
                    if (rc == 0) {
                        s_notify_count++;
                        battery_notify_data_transmit();
                    }
                }
            }
        }

        /* 10ms delay for 100Hz sample rate */
        /* Periodic refresh: write ICHG=50mA to register 0x04 every 60 seconds */
        {
            TickType_t now = xTaskGetTickCount();
            if (s_last_ichg_write == 0 || (now - s_last_ichg_write) >= pdMS_TO_TICKS(60000)) {
                uint8_t ichg = 0x20;
                i2c_write_reg(0x04, &ichg, 1);
                ESP_LOGI(TAG, "Current configured: 50 mA");
                s_last_ichg_write = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Create sensor task */
void sensor_task_create(void)
{
    if (s_task_handle) {
        ESP_LOGW(TAG, "Sensor task already created");
        return;
    }

    ESP_LOGI(TAG, "Creating sensor task...");
    
    BaseType_t ret;
    
#ifdef CONFIG_IDF_TARGET_ESP32C3
    /* ESP32-C3 is single core */
    ret = xTaskCreate(
        sensor_task,
        "sensor_task",
        TASK_STACK_WORDS,
        NULL,
        TASK_PRIO,
        &s_task_handle);
#else
    /* ESP32 has dual core, pin to core 1 */
    ret = xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        TASK_STACK_WORDS,
        NULL,
        TASK_PRIO,
        &s_task_handle,
        1);
#endif
        
}
