/* i2c_scanner.c ─ legacy driver  */
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "SCAN";

static void scanner_task(void *arg)
{
    while (1) {
        for (uint8_t addr = 3; addr < 0x78; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ok = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);

            if (ok == ESP_OK) {
                ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0));

    xTaskCreate(scanner_task, "scanner", 2048, NULL, 5, NULL);
}