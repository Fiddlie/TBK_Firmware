/* i2c_shim.c - Provide a global i2c_write_reg() used by sensor_task
 * This version unconditionally writes to a fixed 7-bit I2C address
 * (BQ21080 charger), without any runtime address detection. */

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_SHIM";

/*  */
#ifndef BQ21080_FORCE_ADDR
#define BQ21080_FORCE_ADDR 0x6A
#endif

/*  Writes 'len' bytes to register 'reg'.
 *  Writes to fixed address BQ21080_FORCE_ADDR. */
esp_err_t i2c_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    const uint8_t addr = BQ21080_FORCE_ADDR;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len && data) {
        i2c_master_write(cmd, data, len, true);
    }
    i2c_master_stop(cmd);

    /* Use controller 0 with a modest timeout */
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    (void)TAG; /* silence unused if logs disabled */
    return ret;
}
