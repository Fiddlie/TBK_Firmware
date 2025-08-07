/* imu.c - Corrected implementation for LSM6DSOX */

#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "lsm6dsox_reg.h"
#include "esp_check.h"

static const char *TAG = "IMU";

/* Global I2C handle and context */
static i2c_port_t i2c_port = I2C_NUM_0;
static stmdev_ctx_t dev_ctx;  // Correct type from ST driver

/* LSM6DSOX I2C address */
#define LSM6DSOX_I2C_ADDR 0x6A  // or 0x6B if SA0 is high

/* Platform specific I2C functions for LSM6DSOX driver */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, bufp, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSOX_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, bufp, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Initialize the IMU */
esp_err_t imu_init(void)
{
    /* Configure I2C master - using legacy driver for compatibility with LSM6DSOX driver */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
    
    /* Initialize LSM6DSOX driver context */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = NULL;
    
    /* Check WHO_AM_I */
    uint8_t whoamI;
    lsm6dsox_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DSOX_ID) {
        ESP_LOGE(TAG, "LSM6DSOX not found! WHO_AM_I = 0x%02X (expected 0x%02X)", whoamI, LSM6DSOX_ID);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LSM6DSOX found!");
    
    /* Restore default configuration */
    lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lsm6dsox_reset_get(&dev_ctx, &rst);
    } while (rst);
    
    /* Configure accelerometer */
    lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
    lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_104Hz);
    
    /* Configure gyroscope */
    lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_500dps);
    lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_104Hz);
    
    /* Enable BDU (Block Data Update) */
    lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    
    ESP_LOGI(TAG, "LSM6DSOX initialized successfully");
    return ESP_OK;
}

/* Read raw sensor data */
bool imu_read_raw(imu_raw_t *out)
{
    uint8_t reg;
    
    /* Check if new data is available */
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);
    if (!reg) {
        return false;  /* No new data */
    }
    
    /* Read accelerometer data */
    int16_t data_raw_acceleration[3];
    lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    out->ax = data_raw_acceleration[0];
    out->ay = data_raw_acceleration[1];
    out->az = data_raw_acceleration[2];
    
    /* Read gyroscope data */
    int16_t data_raw_angular_rate[3];
    lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    out->gx = data_raw_angular_rate[0];
    out->gy = data_raw_angular_rate[1];
    out->gz = data_raw_angular_rate[2];
    
    return true;
}