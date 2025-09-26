#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "IMU";

/* Global I2C handle */
static i2c_port_t i2c_port = I2C_NUM_0;

#define LSM6DSO_I2C_ADDR    0x6A

#define I2C_MASTER_SCL_IO   18
#define I2C_MASTER_SDA_IO   19


#define I2C_MASTER_FREQ_HZ  400000  // Should also Raised to 400kHz to reduce I2C contention
#define I2C_TIMEOUT_MS      1000

#define LSM6DSO_WHO_AM_I        0x0F
#define LSM6DSO_CTRL1_XL        0x10
#define LSM6DSO_CTRL2_G         0x11
#define LSM6DSO_CTRL3_C         0x12
#define LSM6DSO_STATUS_REG      0x1E
#define LSM6DSO_OUTX_L_G        0x22
#define LSM6DSO_OUTX_L_XL       0x28

#define LSM6DSO_TEST_CURRENT    0x20

#define LSM6DSO_WHO_AM_I_VALUE  0x6C

/* ODR (Output Data Rate) settings */
#define LSM6DSO_XL_ODR_104Hz    0x40
#define LSM6DSO_GY_ODR_104Hz    0x40

/* Full Scale settings */
#define LSM6DSO_XL_FS_4G        0x08
#define LSM6DSO_GY_FS_500DPS    0x04

/* Status bits */
#define LSM6DSO_STATUS_XLDA     0x01
#define LSM6DSO_STATUS_GDA      0x02

/* Platform specific I2C write function */
static esp_err_t i2c_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Platform specific I2C read function */
static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Software reset */
static esp_err_t lsm6dso_reset(void)
{
    uint8_t ctrl3;
    esp_err_t ret;
    
    /* Read CTRL3 register */
    ret = i2c_read_reg(LSM6DSO_CTRL3_C, &ctrl3, 1);
    
    ctrl3 |= 0x01;  
    ret = i2c_write_reg(LSM6DSO_CTRL3_C, &ctrl3, 1);
    
    /* Wait for reset to complete */
    vTaskDelay(pdMS_TO_TICKS(50));
    
    /* Check reset is done with timeout */
    int retry = 10;
    do {
        ret = i2c_read_reg(LSM6DSO_CTRL3_C, &ctrl3, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read CTRL3_C after reset: %s", esp_err_to_name(ret));
            return ret;
        }
        if (ctrl3 & 0x01) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } while ((ctrl3 & 0x01) && (--retry > 0));
    
    if (retry == 0) {
        ESP_LOGE(TAG, "IMU reset timeout");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/* Initialize the IMU */
esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing LSM6DSO on ESP32-C3...");
    ESP_LOGI(TAG, "I2C: SDA=GPIO%d, SCL=GPIO%d, ADDR=0x%02X", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, LSM6DSO_I2C_ADDR);
    
    /* Configure I2C master */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
        ESP_LOGI(TAG, "I2C driver already installed, continuing...");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Check WHO_AM_I */
    uint8_t whoami = 0;
    ret = i2c_read_reg(LSM6DSO_WHO_AM_I, &whoami, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (whoami != LSM6DSO_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "LSM6DSO not found! WHO_AM_I = 0x%02X (expected 0x%02X)", 
                 whoami, LSM6DSO_WHO_AM_I_VALUE);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LSM6DSO detected successfully! WHO_AM_I = 0x%02X", whoami);
    
    /* Software reset */
    ESP_LOGI(TAG, "Performing software reset...");
    ret = lsm6dso_reset();

    /* Configure CTRL3_C register - Block Data Update enabled */
    uint8_t ctrl3 = 0x04;
    ret = i2c_write_reg(LSM6DSO_CTRL3_C, &ctrl3, 1);
    
    /* Configure accelerometer: 104Hz, 4g */
    uint8_t ctrl1_xl = LSM6DSO_XL_ODR_104Hz | LSM6DSO_XL_FS_4G;
    ret = i2c_write_reg(LSM6DSO_CTRL1_XL, &ctrl1_xl, 1);
    ESP_LOGI(TAG, "Accelerometer configured: 104Hz, ±4g");
    
    /* Configure gyroscope: 104Hz, 500dps */
    uint8_t ctrl2_g = LSM6DSO_GY_ODR_104Hz | LSM6DSO_GY_FS_500DPS;
    ret = i2c_write_reg(LSM6DSO_CTRL2_G, &ctrl2_g, 1);
    ESP_LOGI(TAG, "Gyroscope configured: 104Hz, ±500dps");\

    /* Test the current: 50 mA */
    uint8_t ctrl2_current = LSM6DSO_GY_ODR_104Hz | LSM6DSO_GY_FS_500DPS;
    ret = i2c_write_reg(LSM6DSO_TEST_CURRENT, &ctrl2_current, 1);
    ESP_LOGI(TAG, "Current configured: 50 mA");
    
    /* Wait for settings to stabilize */
    vTaskDelay(pdMS_TO_TICKS(20));
    
    ESP_LOGI(TAG, "LSM6DSO initialization complete!");
    return ESP_OK;
}

bool imu_read_raw(imu_raw_t *out)
{
    if (!out) {
        return false;
    }
    
    uint8_t status;
    if (i2c_read_reg(LSM6DSO_STATUS_REG, &status, 1) != ESP_OK) {
        return false;
    }
    
    /* Check if both accelerometer and gyroscope data are ready */
    if ((status & (LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA)) != 
        (LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA)) {
        return false;
    }
    
    /* Read gyroscope data (6 bytes from 0x22) */
    uint8_t gyro_data[6];
    if (i2c_read_reg(LSM6DSO_OUTX_L_G, gyro_data, 6) != ESP_OK) {
        return false;
    }
    
    /* Convert to int16_t */
    out->gx = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    out->gy = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    out->gz = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);
    
    /* Read accelerometer data (6 bytes from 0x28) */
    uint8_t accel_data[6];
    if (i2c_read_reg(LSM6DSO_OUTX_L_XL, accel_data, 6) != ESP_OK) {
        return false;
    }
    
    /* Convert to int16_t */
    out->ax = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    out->ay = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    out->az = (int16_t)((accel_data[5] << 8) | accel_data[4]);
    
    return true;
}

/* Optional: Deinitialize IMU (cleanup) */
esp_err_t imu_deinit(void)
{
    /* Put sensor in power-down mode */
    uint8_t zero = 0x00;
    i2c_write_reg(LSM6DSO_CTRL1_XL, &zero, 1);
    i2c_write_reg(LSM6DSO_CTRL2_G, &zero, 1);
    
    return i2c_driver_delete(i2c_port);
}

