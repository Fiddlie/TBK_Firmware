#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"
#include "battery.h"

/* Set to 0 to enable full configuration, using TRM procedure */
#ifndef BAT_BQ27427_SAFE_MODE
#define BAT_BQ27427_SAFE_MODE 0
#endif

static const char *TAG = "BATTERY";

/* Quick boot mode flag - reduce logs during button wake */
static bool quick_boot_mode = false;

static inline bool is_button_wake_cause(esp_sleep_wakeup_cause_t cause)
{
    return (cause == ESP_SLEEP_WAKEUP_GPIO) ||
           (cause == ESP_SLEEP_WAKEUP_EXT0) ||
           (cause == ESP_SLEEP_WAKEUP_EXT1);
}

/* I2C configuration (shared by IMU, BQ27427 fuel gauge, BQ21080 charger) */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SCL_IO   18
#define I2C_MASTER_SDA_IO   19
#define I2C_MASTER_FREQ_HZ  400000//400000

/* BQ27427 fuel gauge (TI) */
#define BQ27427_I2C_ADDR    0x55  // 7-bit address
#define BQ27427_CMD_VOLTAGE 0x04  // Voltage register (mV)
#define BQ27427_CMD_CURRENT 0x10  // Average current (signed mA)
#define BQ27427_CMD_SOC     0x1C  // State of Charge (0-100%)
#define BQ27427_CMD_TEMP    0x02  // Temperature
#define BQ27427_CMD_FLAGS   0x06  // Flags register
#define BQ27427_CMD_RM      0x0C  // Remaining capacity (mAh)
#define BQ27427_CMD_FCC     0x0E  // Full charge capacity (mAh)
#define BQ27427_CMD_CONTROL 0x00  // Control register
#define BQ27427_CMD_DESIGN_CAP 0x3C // Design capacity (if supported by part)

/* BQ27427 Control subcommands (see TRM) */
#define BQ27427_CTRL_STATUS           0x0000  // Read control status
#define BQ27427_CTRL_DEVICE_TYPE      0x0001  // Read device type (0x0427)
#define BQ27427_CTRL_FW_VERSION       0x0002  // Read firmware version
#define BQ27427_CTRL_DM_CODE          0x0004  // Data memory code
#define BQ27427_CTRL_PREV_MACWRITE    0x0007  // Previous MAC write
#define BQ27427_CTRL_CHEM_ID          0x0008  // Chemistry ID
#define BQ27427_CTRL_BAT_INSERT       0x000C  // Battery insert
#define BQ27427_CTRL_BAT_REMOVE       0x000D  // Battery remove
#define BQ27427_CTRL_SET_HIBERNATE    0x0011  // Enter hibernate
#define BQ27427_CTRL_CLEAR_HIBERNATE  0x0012  // Exit hibernate
#define BQ27427_CTRL_SET_CFGUPDATE    0x0013  // Enter CONFIG UPDATE
#define BQ27427_CTRL_SHUTDOWN_ENABLE  0x001B  // Enable shutdown
#define BQ27427_CTRL_SHUTDOWN         0x001C  // Shutdown
#define BQ27427_CTRL_SEALED           0x0020  // Seal device
#define BQ27427_CTRL_TOGGLE_GPOUT     0x0023  // Toggle GPOUT
#define BQ27427_CTRL_RESET            0x0041  // Reset
#define BQ27427_CTRL_SOFT_RESET       0x0042  // Soft reset
#define BQ27427_CTRL_EXIT_CFGUPDATE   0x0043  // Exit CONFIG UPDATE
#define BQ27427_CTRL_EXIT_RESIM       0x0044  // Exit RESIM
#define BQ27427_CTRL_UNSEAL          0x8000
#define BQ27427_CTRL_CHEM_A          0x0030
#define BQ27427_CTRL_CHEM_B          0x0031
#define BQ27427_CTRL_CHEM_C          0x0032
/* Start Impedance Track gauging (enable SOC reporting without a full learn cycle) */
#define BQ27427_CTRL_IT_ENABLE        0x0021

/* BQ27427 extended data memory interface */
#define BQ27427_EXT_DESIGN_CAP  0x3C  // Design Capacity (Data Memory access)
#define BQ27427_EXT_DATA_CLASS  0x3E  // Data class selector
#define BQ27427_EXT_DATA_BLOCK  0x3F  // Data block selector
#define BQ27427_EXT_BLOCK_DATA  0x40  // Block data (32 bytes)
#define BQ27427_EXT_CHECKSUM    0x60  // Block data checksum
#define BQ27427_EXT_CONTROL     0x61  // Block data control

/* BQ21080 register map */
/* Default assumed I2C address; runtime detection picks final address */
#define BQ21080_I2C_ADDR_DEFAULT 0x6A
#define CHG_INT_GPIO        GPIO_NUM_4   // BQ21080 charge interrupt pin (moved per HW)
#define GAS_GPOUT_GPIO      GPIO_NUM_10  // BQ27427 GPOUT pin

/* BQ21080 register map */
#define BQ21080_REG_STATUS    0x00  // Status
#define BQ21080_REG_FAULTS    0x01  // Faults
#define BQ21080_REG_MASK      0x02  // Interrupt mask
#define BQ21080_REG_IC_INFO   0x03  // IC info (PN/Rev)
#define BQ21080_REG_TOP_OFF   0x04  // Precharge/termination current
#define BQ21080_REG_FAST_CHG  0x05  // Fast charge current
#define BQ21080_REG_VBAT_CTRL 0x06  // Battery regulation voltage
#define BQ21080_REG_SYS_REG   0x07  // System regulation voltage
#define BQ21080_REG_TMR_ILIM  0x08  // Safety timer and input limit
#define BQ21080_REG_TS_CTRL   0x09  // TS pin control / NTC config

/* LED control (Red/Green/Blue, common anode). */
#define LED_R_GPIO          GPIO_NUM_6   
#define LED_G_GPIO          GPIO_NUM_12  // Green LED on GPIO12
#define LED_B_GPIO          GPIO_NUM_7   

/* LED state */
#define LED_ON              0
#define LED_OFF             1

/* Color balance for yellow (red+green) on common-anode LED */
#define YELLOW_R_GAIN_PCT 80   // Red gain for breathing yellow (percent) 
#define YELLOW_G_GAIN_PCT 100    // Green gain for breathing yellow (percent) 
#define YELLOW_HOLD_LEVEL 250   // Max duty for solid yellow 

/* Helper function to scale LED duty by percentage */
static inline uint8_t led_scale(uint8_t value, uint8_t percent) {
    uint32_t x = ((uint32_t)value * percent) / 100;
    if (x > 255) x = 255;
    return (uint8_t)x;
}

static inline uint8_t yellow_hold_red_duty(void) {
    return led_scale(YELLOW_HOLD_LEVEL, YELLOW_R_GAIN_PCT);
}

static inline uint8_t yellow_hold_green_duty(void) {
    return led_scale(YELLOW_HOLD_LEVEL, YELLOW_G_GAIN_PCT);
}

/* Power button GPIO */
#define POWER_BUTTON_GPIO   GPIO_NUM_3

static battery_status_t battery_status;
static TaskHandle_t led_task_handle = NULL;
static volatile uint8_t ble_connection_state = BLE_STATE_DISCONNECTED;
static volatile uint32_t last_data_transmit_time = 0;
static volatile bool power_button_pressed = false;
static volatile bool system_shutting_down = false;

/* Debug switches */
#ifndef BAT_LED_DEBUG
#define BAT_LED_DEBUG 0
#endif

/* Runtime-detected BQ21080 address (0x6B preferred, fallback 0x6A if not IMU) */
static uint8_t s_bq21080_addr = 0x00;

/* Task handles for notifications */
static TaskHandle_t battery_monitor_task_handle = NULL;
/* Mutex for battery_status protection */
static SemaphoreHandle_t battery_status_mutex = NULL;

/* Forward declarations for local I2C helpers used below */
static esp_err_t i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t *value);
static esp_err_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t value);
/* Forward declarations to avoid implicit declarations (Werror) */
static esp_err_t bq21080_read_status(uint8_t *status, uint8_t *faults);
static esp_err_t bq21080_init(void);
static esp_err_t bq21080_diagnose(void);
static bool bq21080_detect_address(void);

/* BQ21080 8-bit register access using detected address */
static esp_err_t bq21080_read8(uint8_t reg, uint8_t *value)
{
    if (s_bq21080_addr == 0x00) return ESP_FAIL;
    return i2c_read_reg8(s_bq21080_addr, reg, value);
}

static esp_err_t bq21080_write8(uint8_t reg, uint8_t val)
{
    if (s_bq21080_addr == 0x00) return ESP_FAIL;
    return i2c_write_reg8(s_bq21080_addr, reg, val);
}

/* Expose detected charger address for helpers */
uint8_t battery_get_bq21080_addr(void)
{
    return s_bq21080_addr;
}

/* Public helper: set BQ21080 fast charge current (approx mA) */
esp_err_t battery_charger_set_current_ma(int ma)
{
    if (s_bq21080_addr == 0x00) {
        ESP_LOGW(TAG, "BQ21080 address not detected; cannot set current");
        return ESP_ERR_NOT_FOUND;
    }

    /* Clamp to a sane range */
    if (ma < 5) ma = 5;
    if (ma > 800) ma = 800;

    /* Use existing register mapping used elsewhere in this file */
    int code = (int)lroundf((ma - 37.5f) / 12.5f);
    if (code < 0) code = 0;
    if (code > 0x3F) code = 0x3F;
    uint8_t reg = (uint8_t)code;

    esp_err_t wr = bq21080_write8(BQ21080_REG_FAST_CHG, reg);
    if (wr != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BQ21080 ICHG to %dmA (code=%d): %s", ma, code, esp_err_to_name(wr));
        return wr;
    }

    /* Read back for confirmation */
    uint8_t rb = 0;
    if (bq21080_read8(BQ21080_REG_FAST_CHG, &rb) == ESP_OK) {
        int code_rb = rb & 0x3F;
        int ichg_ma = (int)(37.5f + 12.5f * code_rb);
        ESP_LOGI(TAG, "BQ21080 ICHG set: code=%d -> ~%dmA (req=%dmA)", code_rb, ichg_ma, ma);
    }

    return ESP_OK;
}

static bool bq21080_detect_address(void)
{
    static bool detection_done = false;
    if (detection_done) {
        return s_bq21080_addr != 0x00;
    }

    const uint8_t candidates[] = { 0x6B, 0x6A };
    uint8_t ic_info = 0;

    for (size_t i = 0; i < sizeof(candidates); ++i) {
        uint8_t addr = candidates[i];
        if (i2c_read_reg8(addr, BQ21080_REG_IC_INFO, &ic_info) == ESP_OK) {
            if (addr == 0x6A) {
                uint8_t imu_who = 0;
                if (i2c_read_reg8(addr, 0x0F, &imu_who) == ESP_OK && imu_who == 0x6C) {
                    ESP_LOGW(TAG, "Address 0x6A responds like IMU (WHO_AM_I=0x6C); skipping as BQ21080");
                    continue;
                }
            }
            s_bq21080_addr = addr;
            detection_done = true;
            ESP_LOGI(TAG, "BQ21080 detected at 0x%02X (IC_INFO=0x%02X)", addr, ic_info);
            return true;
        }
    }

    detection_done = true;
    s_bq21080_addr = 0x00;
    ESP_LOGW(TAG, "BQ21080 not detected on preferred addresses 0x6B/0x6A");
    return false;
}

/* I2C utility: Read 16-bit register (little-endian) */
static esp_err_t i2c_read_reg16(uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *value = (data[1] << 8) | data[0];  // Little endian
    }
    
    return ret;
}

/* I2C utility: Read and write 8-bit register */
static esp_err_t i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/* I2C utility: Read and write 16-bit register (little-endian) */
static esp_err_t i2c_write_reg16(uint8_t addr, uint8_t reg, uint16_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value & 0xFF, true);  // Low byte first
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true);  // High byte
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/* BQ27427 control */
static esp_err_t bq27427_control_cmd(uint16_t subcommand)
{
    return i2c_write_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CONTROL, subcommand);
}

static esp_err_t bq27427_read_control_result(uint16_t *result)
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow gauge to respond
    return i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CONTROL, result);
}

/* BQ27427 initialisation config process */
#if !BAT_BQ27427_SAFE_MODE
static esp_err_t bq27427_enter_config_update(void)
{
    ESP_LOGI(TAG, "Entering BQ27427 CONFIG UPDATE mode...");
    
    /* 1. SET_CFGUPDATE */
    esp_err_t ret = bq27427_control_cmd(BQ27427_CTRL_SET_CFGUPDATE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SET_CFGUPDATE command");
        return ret;
    }
    
    /* 2. Delay */
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* 3. FLAGS with CFGUPMODE */
    uint16_t flags;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &flags) == ESP_OK) {
        if (flags & 0x0010) {  // CFGUPMODE bit
            ESP_LOGI(TAG, "Successfully entered CONFIG UPDATE mode");
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "Failed to enter CONFIG UPDATE mode");
    return ESP_FAIL;
}

/* BQ27427 exit config process */
static esp_err_t bq27427_exit_config_update(void)
{
    ESP_LOGI(TAG, "Exiting CONFIG UPDATE mode...");
    
    /* 1. EXIT_CFGUPDATE */
    esp_err_t ret = bq27427_control_cmd(BQ27427_CTRL_EXIT_CFGUPDATE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send EXIT_CFGUPDATE command");
        return ret;
    }
    
    /* 2. Delay and reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    bq27427_control_cmd(BQ27427_CTRL_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Configuration update complete");
    return ESP_OK;
}

#endif

/* BQ27427 Unseal safe mode and configure process */
#if !BAT_BQ27427_SAFE_MODE
static __attribute__((unused)) esp_err_t bq27427_fix_current_sensing(void)
{
    /* 1. Unseal */
    ESP_LOGI(TAG, "Unsealing BQ27427...");
    i2c_write_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CONTROL, 0x0414);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CONTROL, 0x3672);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* 2. Config update mode */
    ESP_LOGI(TAG, "Entering config update mode...");
    bq27427_control_cmd(BQ27427_CTRL_SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* 3. Read register */
    uint16_t flags;
    i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &flags);
    if (!(flags & 0x0010)) {
        ESP_LOGE(TAG, "Failed to enter config mode");
        return ESP_FAIL;
    }
    
    /* 4. CC Gain for 7mO internal resistor (Class 104, Offset 4-5) */
    ESP_LOGI(TAG, "Configuring CC Gain for 7mO internal sensing resistor...");
    
    /* CC Gain = 4.768 / Rsense = 4.768 / 0.007 = 681.14 */
    /* BQ27427 uses scaled value: actual_value * 65536 / 4.768 */
    /* 681.14 * 65536 / 4.768 = 9359155 / 1000 = 9359 (0x248F) */
    uint16_t cc_gain = 0x248F;  /* For 7mO resistor */
    
    /* 104 (0x68) - Current */
    i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_CONTROL, 0x00);
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3E, 0x68);

    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3F, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Read Current */
    uint8_t current_data[32];
    for (int i = 0; i < 32; i++) {
        i2c_read_reg8(BQ27427_I2C_ADDR, 0x40 + i, &current_data[i]);
    }
    
    /* CC Gain, offset 4-5 (big-endian) */
    uint16_t old_cc_gain = (current_data[4] << 8) | current_data[5];
    ESP_LOGI(TAG, "Current CC Gain: 0x%04X", old_cc_gain);
    
    /* CC Gain */
    current_data[4] = (cc_gain >> 8) & 0xFF;
    current_data[5] = cc_gain & 0xFF;
    ESP_LOGI(TAG, "New CC Gain: 0x%04X (for 7mO)", cc_gain);
    
    uint8_t checksum = 0;
    for (int i = 0; i < 32; i++) {
        checksum += current_data[i];
    }
    checksum = 0xFF - checksum;
    
    for (int i = 0; i < 32; i++) {
        i2c_write_reg8(BQ27427_I2C_ADDR, 0x40 + i, current_data[i]);
    }
    
    /* Write checksum */
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x60, checksum);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* 5. OpConfig (Class 64, Offset 0) */
    ESP_LOGI(TAG, "Configuring OpConfig for internal sensing...");
    
    /* Data class = 64 (0x40) */
    i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_CONTROL, 0x00);
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3E, 0x40);
    /* Data block = 0 */
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3F, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Read 32-byte block data */
    uint8_t data_block[32];
    for (int i = 0; i < 32; i++) {
        i2c_read_reg8(BQ27427_I2C_ADDR, 0x40 + i, &data_block[i]);
    }
    
    /* OpConfig offset 0-1 */
    uint16_t opcfg = (data_block[1] << 8) | data_block[0];
    ESP_LOGI(TAG, "Current OpConfig: 0x%04X", opcfg);
    
    /* Set BATLOWEN=1 (bit 2) for 7mO internal sense */
    opcfg |= (1 << 2);
    /* Set IWAKE=1 (bit 5) to keep gauge awake */
    opcfg |= (1 << 5);
    /* Clear RSNS (bit 4) to use internal sense resistor */
    opcfg &= ~(1 << 4);
    
    ESP_LOGI(TAG, "New OpConfig: 0x%04X", opcfg);
    
    /* Update block with new OpConfig value */
    data_block[0] = opcfg & 0xFF;
    data_block[1] = (opcfg >> 8) & 0xFF;
    
    /* Calculate checksum */
    uint8_t checksum2 = 0;
    for (int i = 0; i < 32; i++) {
        checksum2 += data_block[i];
    }
    checksum2 = 0xFF - checksum2;
    
    /* Write updated block data */
    for (int i = 0; i < 32; i++) {
        i2c_write_reg8(BQ27427_I2C_ADDR, 0x40 + i, data_block[i]);
    }
    
    /* Write checksum */
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x60, checksum2);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* 6. Exit config mode */
    ESP_LOGI(TAG, "Exiting config mode...");
    bq27427_control_cmd(BQ27427_CTRL_EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* 7. Soft reset to apply changes */
    ESP_LOGI(TAG, "Soft reset to apply changes...");
    bq27427_control_cmd(BQ27427_CTRL_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(300));
    /* Start gauging (IT_ENABLE) so SOC can report without a learn cycle */
    bq27427_control_cmd(BQ27427_CTRL_IT_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(100));
    /* Trigger SOC update as if battery inserted */
    bq27427_control_cmd(BQ27427_CTRL_BAT_INSERT);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* 8. Verify current measurement */
    uint16_t current_raw;
    i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CURRENT, &current_raw);
    int16_t current = (int16_t)current_raw;
    ESP_LOGI(TAG, "Current after fix: %dmA", current);
    
    ESP_LOGI(TAG, "========== Current Sensing Fix Complete (7mO configured) ==========");
    return ESP_OK;
}

#endif

#if !BAT_BQ27427_SAFE_MODE
static esp_err_t bq27427_configure_battery_params(void)
{
    ESP_LOGI(TAG, "========== Configuring BQ27427 for 100mAh battery ==========");
    const uint16_t design_cap = 100;
    const uint16_t design_energy = 370;
    const uint16_t term_voltage = 3300;
    esp_err_t ret = ESP_OK;
    bool in_config_update = false;

    uint16_t ctrl_status = 0;
    if (bq27427_control_cmd(BQ27427_CTRL_STATUS) == ESP_OK &&
        bq27427_read_control_result(&ctrl_status) == ESP_OK &&
        (ctrl_status & 0x2000)) {
        ESP_LOGI(TAG, "Gauge is sealed, sending UNSEAL key 0x8000 twice");
        for (int i = 0; i < 2; ++i) {
            ret = bq27427_control_cmd(BQ27427_CTRL_UNSEAL);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send UNSEAL key (%d)", ret);
                return ret;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    ret = bq27427_enter_config_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter CONFIG UPDATE mode (%d)", ret);
        return ret;
    }
    in_config_update = true;

    ret = i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_CONTROL, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write BlockDataControl (%d)", ret);
        goto cleanup;
    }
    ret = i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_DATA_CLASS, 82);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select State subclass (%d)", ret);
        goto cleanup;
    }
    ret = i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_DATA_BLOCK, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select data block 0 (%d)", ret);
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t block_data[32];
    for (int i = 0; i < 32; ++i) {
        ret = i2c_read_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_BLOCK_DATA + i, &block_data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read block data byte %d (%d)", i, ret);
            goto cleanup;
        }
    }

    uint16_t old_design_cap = ((uint16_t)block_data[6] << 8) | block_data[7];
    uint16_t old_design_energy = ((uint16_t)block_data[8] << 8) | block_data[9];
    uint16_t old_term_voltage = ((uint16_t)block_data[10] << 8) | block_data[11];

    block_data[6] = (uint8_t)((design_cap >> 8) & 0xFF);
    block_data[7] = (uint8_t)(design_cap & 0xFF);
    block_data[8] = (uint8_t)((design_energy >> 8) & 0xFF);
    block_data[9] = (uint8_t)(design_energy & 0xFF);
    block_data[10] = (uint8_t)((term_voltage >> 8) & 0xFF);
    block_data[11] = (uint8_t)(term_voltage & 0xFF);

    ESP_LOGI(TAG, "Applying parameters:");
    ESP_LOGI(TAG, "  Design Capacity: %u mAh -> %u mAh", (unsigned)old_design_cap, (unsigned)design_cap);
    ESP_LOGI(TAG, "  Design Energy:   %u mWh -> %u mWh", (unsigned)old_design_energy, (unsigned)design_energy);
    ESP_LOGI(TAG, "  Terminate Volt:  %u mV -> %u mV", (unsigned)old_term_voltage, (unsigned)term_voltage);

    uint32_t sum = 0;
    for (int i = 0; i < 32; ++i) {
        sum += block_data[i];
        ret = i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_BLOCK_DATA + i, block_data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write block data byte %d (%d)", i, ret);
            goto cleanup;
        }
    }
    uint8_t checksum = (uint8_t)(0xFF - (sum & 0xFF));
    ret = i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_CHECKSUM, checksum);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write checksum (%d)", ret);
        goto cleanup;
    }

    ESP_LOGI(TAG, "Selecting chemistry profile CHEM_B (0x04B2)");
    ret = bq27427_control_cmd(BQ27427_CTRL_CHEM_B);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply chemistry profile (%d)", ret);
        goto cleanup;
    }

    ret = bq27427_exit_config_update();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit CONFIG UPDATE mode (%d)", ret);
        return ret;
    }
    in_config_update = false;

    vTaskDelay(pdMS_TO_TICKS(200));
    bq27427_control_cmd(BQ27427_CTRL_IT_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(10));
    bq27427_control_cmd(BQ27427_CTRL_BAT_INSERT);

    uint16_t verify_cap;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_DESIGN_CAP, &verify_cap) == ESP_OK) {
        ESP_LOGI(TAG, "Verified Design Capacity command: %u mAh", (unsigned)verify_cap);
    }
    uint16_t chem_now;
    if (bq27427_control_cmd(BQ27427_CTRL_CHEM_ID) == ESP_OK &&
        bq27427_read_control_result(&chem_now) == ESP_OK) {
        ESP_LOGI(TAG, "Verified Chem ID: 0x%04X", chem_now);
    }

    ESP_LOGI(TAG, "Configuration complete.");
    return ESP_OK;

cleanup:
    if (in_config_update) {
        bq27427_exit_config_update();
    }
    return ret;
}
#endif /* !BAT_BQ27427_SAFE_MODE */

/* BQ21080 diagnostic helper */
static esp_err_t bq21080_diagnose(void)
{
    ESP_LOGI(TAG, "========== BQ21080 Diagnostic Check ==========");
    
    /* 1. Scan possible charger I2C addresses */
    ESP_LOGI(TAG, "1. Scanning for BQ21080 at possible addresses:");
    uint8_t possible_addrs[] = {0x6B, 0x6A, 0x69, 0x68};  // Candidate addresses
    bool found = false;
    
    for (int i = 0; i < 4; i++) {
        uint8_t addr = possible_addrs[i];
        uint8_t data;
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);  // Try to read register 0
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X, data=0x%02X", addr, data);
            if (s_bq21080_addr != 0x00 && addr == s_bq21080_addr) {
                found = true;
            }
        } else {
            ESP_LOGI(TAG, "  No response at 0x%02X", addr);
        }
    }
    
    if (!found) {
        ESP_LOGE(TAG, "BQ21080 not found at detected address 0x%02X", s_bq21080_addr);
        ESP_LOGE(TAG, "Hardware check needed:");
        ESP_LOGE(TAG, "  1. Check if BQ21080 is soldered");
        ESP_LOGE(TAG, "  2. Check SDA/SCL connections");
        ESP_LOGE(TAG, "  3. Check power supply to BQ21080");
        ESP_LOGE(TAG, "  4. Check CE pin (must be HIGH to enable)");
        return ESP_FAIL;
    }
    
    /* 2. Read BQ21080 registers */
    ESP_LOGI(TAG, "2. Reading BQ21080 registers:");
    
    uint8_t regs[16];
    const char* reg_names[] = {
        "STATUS", "FAULTS", "MASK", "IC_INFO",
        "TOP_OFF", "FAST_CHG", "VBAT_CTRL", "SYS_REG",
        "TMR_ILIM", "Reserved", "Reserved", "Reserved",
        "Reserved", "Reserved", "Reserved", "Reserved"
    };
    
    for (int i = 0; i < 10; i++) {  // Read first 10 registers
        if (bq21080_read8(i, &regs[i]) == ESP_OK) {
            ESP_LOGI(TAG, "  Reg[0x%02X] %-10s = 0x%02X", i, reg_names[i], regs[i]);
        } else {
            ESP_LOGW(TAG, "  Reg[0x%02X] %-10s = READ FAILED", i, reg_names[i]);
        }
    }
    
    /* 3. Analyze status register */
    ESP_LOGI(TAG, "3. Status Register Analysis:");
    uint8_t status = regs[0];
    ESP_LOGI(TAG, "  Charge State: ");
    uint8_t chrg_stat = (status >> 4) & 0x07;
    switch(chrg_stat) {
        case 0: ESP_LOGI(TAG, "    - Not Charging"); break;
        case 1: ESP_LOGI(TAG, "    - Pre-charge"); break;
        case 2: ESP_LOGI(TAG, "    - Fast Charge"); break;
        case 3: ESP_LOGI(TAG, "    - Charge Done"); break;
        case 4: ESP_LOGI(TAG, "    - Timer Fault"); break;
        case 5: ESP_LOGI(TAG, "    - Temperature Fault"); break;
        case 6: ESP_LOGI(TAG, "    - VBAT_OVP Fault"); break;
        default: ESP_LOGI(TAG, "    - Unknown"); break;
    }
    
    ESP_LOGI(TAG, "  PG (Power Good): %s", (status & 0x08) ? "YES" : "NO");
    ESP_LOGI(TAG, "  VINDPM: %s", (status & 0x04) ? "Active" : "Not Active");
    ESP_LOGI(TAG, "  TREG: %s", (status & 0x02) ? "In Regulation" : "Normal");
    
    /* 4. Analyze fault register */
    ESP_LOGI(TAG, "4. Fault Register Analysis:");
    uint8_t faults = regs[1];
    if (faults == 0) {
        ESP_LOGI(TAG, "  No faults detected");
    } else {
        if (faults & 0x80) ESP_LOGW(TAG, "  VBUS_OVP - Input overvoltage");
        if (faults & 0x40) ESP_LOGW(TAG, "  VBAT_OVP - Battery overvoltage");
        if (faults & 0x20) ESP_LOGW(TAG, "  ILIM - Input current limit");
        if (faults & 0x10) ESP_LOGW(TAG, "  IBAT_REG - Battery current regulation");
        if (faults & 0x08) ESP_LOGW(TAG, "  THERM_REG - Thermal regulation");
        if (faults & 0x04) ESP_LOGW(TAG, "  TIMER_FAULT - Safety timer expired");
        if (faults & 0x02) ESP_LOGW(TAG, "  TS_FAULT - TS pin fault");
    }
    
    /* 5. Charging parameters */
    ESP_LOGI(TAG, "5. Charging Parameters:");
    uint8_t fast_chg = regs[5];
    int ichg_code = fast_chg & 0x3F;
    int charge_current_ma = (375 + 125 * ichg_code + 5) / 10;  // 37.5 + 12.5*code (rounded)
    ESP_LOGI(TAG, "  Charge Current (prog): %dmA (code=%d)", charge_current_ma, ichg_code);
    ESP_LOGI(TAG, "  Expected for 100mAh battery: 20-100mA");
    
    uint8_t vbat_ctrl = regs[6];
    int vbat_reg_mv = 3500 + ((vbat_ctrl >> 1) & 0x3F) * 20;  // 3.5V + 20mV per LSB
    ESP_LOGI(TAG, "  Battery Regulation Voltage: %dmV", vbat_reg_mv);
    
    ESP_LOGI(TAG, "===============================================");
    
    return ESP_OK;
}

/* Initialize BQ21080 for a 100 mAh battery */

static esp_err_t bq21080_init(void)
{
    uint8_t status;
    
    ESP_LOGI(TAG, "========== Initializing BQ21080 for 100mAh Battery ==========");
    
    /* Write checksum */
    bq21080_diagnose();
    
    /* Abort if charger access fails */
    if (bq21080_read8(BQ21080_REG_STATUS, &status) != ESP_OK) {
        ESP_LOGE(TAG, "BQ21080 not found!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BQ21080 detected, initial status: 0x%02x", status);
    
    /* Warn if sustained current exceeds the 2C threshold */
    uint8_t chrg_stat = (status >> 4) & 0x07;
    const char* chrg_str[] = {"Not Charging", "Pre-charge", "Fast Charge", 
                             "Charge Done", "Timer Fault", "Temp Fault", 
                             "VBAT_OVP", "Unknown"};
    ESP_LOGI(TAG, "Initial charge state: %s", chrg_str[chrg_stat < 7 ? chrg_stat : 7]);
    
    /* 1. Set charge current for 100 mAh cell: target 50 mA (0.5C) */
    /* BQ21080_REG_FAST_CHG: bit[5:0] = ICHG_CODE */
    /* Per TRM: ICHG(mA) = 37.5 + 12.5 * ICHG_CODE (code 0..63) */
    {
        int target_ma = 75;  /* 0.5C for 100 mAh */
        int code = (int)lroundf((target_ma - 37.5f) / 12.5f);
        if (code < 0) code = 0;
        if (code > 0x3F) code = 0x3F;
        uint8_t fast_chg_reg = (uint8_t)code;
        bq21080_write8(BQ21080_REG_FAST_CHG, fast_chg_reg);
        /* Read-back and log the effective programmed value */
        uint8_t fast_chg_rb = 0;
        if (bq21080_read8(BQ21080_REG_FAST_CHG, &fast_chg_rb) == ESP_OK) {
            int code_rb = fast_chg_rb & 0x3F;
            int ichg_ma = (int)(37.5f + 12.5f * code_rb);
            ESP_LOGI(TAG, "FAST_CHG set: code=%d -> ICHG=%dmA", code_rb, ichg_ma);
        } else {
            ESP_LOGI(TAG, "FAST_CHG write done (read-back failed)");
        }
    }
    
    /* 2. Configure pre-charge and termination current */
    /* BQ21080_REG_TOP_OFF: bit[7:5]=IPRECHG, bit[2:0]=ITERM */
    /* Target ~10 mA (0.1C) for both pre-charge and termination */
    uint8_t top_off = 0x00;  // IPRECHG=37.5mA (min), ITERM=37.5mA (min, approximates 10mA)
    bq21080_write8(BQ21080_REG_TOP_OFF, top_off);
    ESP_LOGI(TAG, "Pre-charge and termination current configured");
    
    /* 3. Charge voltage - 4.20V for Li-ion/LiPo */
    /* BQ21080_REG_VBAT_CTRL: bits[6:1]=VBATREG_CODE, LSB=20mV, formula: 3.5V + code*20mV */
    /* 4.20V -> code = (4200-3500)/20 = 35 (0x23) */
    {
        uint8_t vbat_ctrl = (uint8_t)((35 & 0x3F) << 1);
        bq21080_write8(BQ21080_REG_VBAT_CTRL, vbat_ctrl);
        uint8_t vbat_rb = 0;
        if (bq21080_read8(BQ21080_REG_VBAT_CTRL, &vbat_rb) == ESP_OK) {
            int code_rb = (vbat_rb >> 1) & 0x3F;
            int vbat_mv = 3500 + code_rb * 20;
            ESP_LOGI(TAG, "VBAT_CTRL set: code=%d -> VBATREG=%dmV", code_rb, vbat_mv);
        } else {
            ESP_LOGI(TAG, "VBAT_CTRL write done (read-back failed)");
        }
    }
    
    /* 4. Configure safety timer and input limit */
    /* BQ21080_REG_TMR_ILIM: bit[7:6]=TMR, bit[2:0]=ILIM */
    /* Use 10-hour safety timer and 400mA input limit */
    uint8_t tmr_ilim = 0x80 | 0x04;  // 10hr timer, 400mA input limit
    bq21080_write8(BQ21080_REG_TMR_ILIM, tmr_ilim);
    ESP_LOGI(TAG, "Safety timer: 10 hours, Input limit: 400mA");
    
    /* 5. Read charger IC info */
    uint8_t ic_info;
    if (bq21080_read8(BQ21080_REG_IC_INFO, &ic_info) == ESP_OK) {
        ESP_LOGI(TAG, "BQ21080 IC Info: 0x%02x", ic_info);
        ESP_LOGI(TAG, "  Part Number: %d", (ic_info >> 3) & 0x1F);
        ESP_LOGI(TAG, "  Revision: %d", ic_info & 0x07);
    }
    
    /* 6. Read back final status */
    if (bq21080_read8(BQ21080_REG_STATUS, &status) == ESP_OK) {
        chrg_stat = (status >> 4) & 0x07;
        ESP_LOGI(TAG, "Final charge state: %s", chrg_str[chrg_stat < 7 ? chrg_stat : 7]);
        if (chrg_stat == 1 || chrg_stat == 2) {
            ESP_LOGI(TAG, "BQ21080 is actively charging!");
        }
    }
    
    ESP_LOGI(TAG, "========== BQ21080 Initialization Complete ==========");
    return ESP_OK;
}

/* Read cached BQ21080 status */
static esp_err_t bq21080_read_status(uint8_t *status, uint8_t *faults)
{
    esp_err_t ret = ESP_OK;
    
    if (status) {
        ret = bq21080_read8(BQ21080_REG_STATUS, status);
        if (ret == ESP_OK) {
            /* Log BQ21080 status changes */
            static uint8_t last_status = 0xFF;
            if (*status != last_status) {
                ESP_LOGI(TAG, "BQ21080 status changed: 0x%02X -> 0x%02X", last_status, *status);
                uint8_t chrg_stat = (*status >> 4) & 0x07;
                const char* chrg_str[] = {"Not Charging", "Pre-charge", "Fast Charge", 
                                         "Charge Done", "Timer Fault", "Temp Fault", 
                                         "VBAT_OVP", "Unknown"};
                ESP_LOGI(TAG, "  Charge State: %s", chrg_str[chrg_stat < 7 ? chrg_stat : 7]);
                ESP_LOGI(TAG, "  PG: %s, VINDPM: %s, TREG: %s",
                         (*status & 0x08) ? "Yes" : "No",
                         (*status & 0x04) ? "Active" : "Inactive",
                         (*status & 0x02) ? "Active" : "Normal");
                last_status = *status;
            }
        }
    }
    
    if (faults && ret == ESP_OK) {
        ret = bq21080_read8(BQ21080_REG_FAULTS, faults);
        if (ret == ESP_OK && *faults != 0) {
            ESP_LOGW(TAG, "BQ21080 faults detected: 0x%02X", *faults);
        }
    }
    
    return ret;
}

/* Verify CC gain configuration */

#if !BAT_BQ27427_SAFE_MODE
static __attribute__((unused)) esp_err_t bq27427_verify_cc_gain(void)
{
    ESP_LOGI(TAG, "========== Verifying CC Gain Configuration ==========");
    
    /* 1. Enter CONFIG UPDATE mode */
    bq27427_control_cmd(BQ27427_CTRL_SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* Check FLAGS for CFGUPMODE */
    uint16_t flags;
    i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &flags);
    if (!(flags & 0x0010)) {
        ESP_LOGE(TAG, "Cannot enter config mode to verify CC Gain");
        return ESP_FAIL;
    }
    
    /* 2. Read CC Gain (Class 104, offset 4-5) */
    i2c_write_reg8(BQ27427_I2C_ADDR, BQ27427_EXT_CONTROL, 0x00);
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3E, 0x68);  // Class 104
    i2c_write_reg8(BQ27427_I2C_ADDR, 0x3F, 0x00);  // Offset 0
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t data[32];
    for (int i = 0; i < 32; i++) {
        i2c_read_reg8(BQ27427_I2C_ADDR, 0x40 + i, &data[i]);
    }
    
    uint16_t cc_gain = (data[4] << 8) | data[5];
    ESP_LOGI(TAG, "Current CC Gain: 0x%04X", cc_gain);
    
    /* Write updated block data */
    float rsense = 4.768 / (cc_gain * 4.768 / 65536.0);
    ESP_LOGI(TAG, "Calculated Rsense: %.1f mO", rsense * 1000);
    
    if (cc_gain == 0x248F) {
        ESP_LOGI(TAG, "CC Gain correctly configured for 7mO");
    } else {
        ESP_LOGW(TAG, "CC Gain mismatch! Expected 0x248F, got 0x%04X", cc_gain);
    }
    
    /* 3. Exit CONFIG UPDATE mode */
    bq27427_control_cmd(BQ27427_CTRL_EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    return ESP_OK;
}

/* BQ27427 configuration check */
#endif 

static esp_err_t bq27427_configure(void)
{
    ESP_LOGI(TAG, "========== BQ27427 Configuration Check ==========");
    
    /* 1. Read device type */
    uint16_t device_type = 0;
    if (bq27427_control_cmd(BQ27427_CTRL_DEVICE_TYPE) == ESP_OK) {
        if (bq27427_read_control_result(&device_type) == ESP_OK) {
            ESP_LOGI(TAG, "Device Type: 0x%04X", device_type);
        }
    }
    
    /* 2. Read CHEM_ID (chemistry ID) */
    uint16_t chem_id = 0;
    if (bq27427_control_cmd(BQ27427_CTRL_CHEM_ID) == ESP_OK) {
        if (bq27427_read_control_result(&chem_id) == ESP_OK) {
            ESP_LOGI(TAG, "Chemistry ID: 0x%04X", chem_id);
            /* CHEM_ID table:
             * 0x04B2 = LiPo 4.20V (CHEM_B)
             * 0x0354 = LiPo 4.35V
             * 0x0313 = LiFePO4 3.60V
             */
            if (chem_id == 0x04B2) {
                ESP_LOGI(TAG, "  Chemistry set to LiPo 4.20V (CHEM_B)");
            } else {
                ESP_LOGW(TAG, "  Unknown chemistry ID, may need configuration");
            }
        }
    }
    
    /* 3. Read design capacity */
    uint16_t design_cap;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_DESIGN_CAP, &design_cap) == ESP_OK) {
        ESP_LOGI(TAG, "Design Capacity: %dmAh", design_cap);
        
        if (design_cap == 100) {
            ESP_LOGI(TAG, "  Design capacity correctly set to 100mAh");
        } else {
            ESP_LOGW(TAG, "  Design capacity is %dmAh, should be 100mAh", design_cap);
        }
    }
    
    /* 4. Read full charge capacity (diagnostic) */
    uint16_t full_cap;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FCC, &full_cap) == ESP_OK) {
        ESP_LOGI(TAG, "Full Charge Capacity: %dmAh", full_cap);
        if (full_cap < 80 || full_cap > 120) {
            ESP_LOGW(TAG, "  FCC outside expected range (80-120mAh), needs calibration");
        }
    }
    
    /* 5. Read remaining capacity */
    uint16_t remain_cap;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_RM, &remain_cap) == ESP_OK) {
        ESP_LOGI(TAG, "Remaining Capacity: %dmAh", remain_cap);
    }
    
    /* 6. Control status */
    uint16_t ctrl_status = 0;
    if (bq27427_control_cmd(BQ27427_CTRL_STATUS) == ESP_OK) {
        if (bq27427_read_control_result(&ctrl_status) == ESP_OK) {
            ESP_LOGI(TAG, "Control Status: 0x%04X", ctrl_status);
            ESP_LOGI(TAG, "  - SEALED: %s", (ctrl_status & 0x2000) ? "Yes" : "No");
            ESP_LOGI(TAG, "  - FULL ACCESS SEALED: %s", (ctrl_status & 0x4000) ? "Yes" : "No");
            /* CAL_MODE decoding removed (bit mapping differs across variants) */
            ESP_LOGI(TAG, "  - CCA: %s", (ctrl_status & 0x0040) ? "Coulomb Counter Active" : "Inactive");
            ESP_LOGI(TAG, "  - BCA: %s", (ctrl_status & 0x0020) ? "Board Calibration Active" : "Inactive");
            ESP_LOGI(TAG, "  - QMAX_UP: %s", (ctrl_status & 0x0010) ? "QMax Update" : "No Update");
            ESP_LOGI(TAG, "  - RES_UP: %s", (ctrl_status & 0x0008) ? "Resistance Update" : "No Update");
            ESP_LOGI(TAG, "  - INITCOMP: %s", (ctrl_status & 0x0080) ? "Initialization Complete" : "Not Complete");
        }
    }
    
    /* 7. Check FLAGS register */
    uint16_t flags;
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &flags) == ESP_OK) {
        ESP_LOGI(TAG, "FLAGS: 0x%04X", flags);
        ESP_LOGI(TAG, "  - BAT_DET: %s", (flags & 0x0008) ? "Battery detected" : "No battery");
        ESP_LOGI(TAG, "  - SOC1: %s", (flags & 0x0004) ? "SOC <= 1st threshold" : "SOC > 1st threshold");
        ESP_LOGI(TAG, "  - SOCF: %s", (flags & 0x0002) ? "SOC <= final threshold" : "SOC > final threshold");
        ESP_LOGI(TAG, "  - DSG: %s", (flags & 0x0001) ? "Discharging" : "Not discharging");
        ESP_LOGI(TAG, "  - FC: %s", (flags & 0x0020) ? "Full charged" : "Not full");
        ESP_LOGI(TAG, "  - CHG: %s", (flags & 0x0100) ? "Charging allowed" : "Charging inhibited");
        ESP_LOGI(TAG, "  - IMAX: %s", (flags & 0x0040) ? "Current > Imax" : "Current OK");
    }
    
    ESP_LOGI(TAG, "================================================");
    
    /* Summary */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== BQ27427 Usage Status ==========");
    
    bool impedance_track_ok = false;
    if (design_cap == 100 && chem_id == 0x04B2) {
        ESP_LOGI(TAG, "Configuration correct for 100mAh LiPo 4.20V");
        impedance_track_ok = true;
    } else {
        ESP_LOGE(TAG, "Configuration INCORRECT!");
        ESP_LOGE(TAG, "  - Design Cap: %dmAh (should be 100mAh)", design_cap);
        ESP_LOGE(TAG, "  - CHEM_ID: 0x%04X (should be 0x04B2)", chem_id);
    }
    
    /* Impedance Track summary */
    if ((ctrl_status & 0x0080) && impedance_track_ok) {  // INITCOMP bit
        ESP_LOGI(TAG, "Impedance Track algorithm is ACTIVE");
    } else {
        ESP_LOGW(TAG, "Impedance Track NOT fully active!");
    }
    
    return ESP_OK;
}

/* BQ27427 status helper */
static __attribute__((unused)) esp_err_t bq27427_read_status(battery_status_t *status)
{
    uint16_t value;
    static bool first_read = true;
    static bool low_voltage_warning = false;
    
    /* Voltage (mV) read directly from BQ27427 */
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_VOLTAGE, &value) == ESP_OK) {
        status->voltage_mV = value;
        
        /* Detect sudden voltage jumps (>~50 mV when idle) */
        
        /* Track last stable voltage reading */
        static uint16_t last_stable_voltage = 0;
        if (last_stable_voltage > 0) {
            int16_t jump = abs(status->voltage_mV - last_stable_voltage);
            if (jump > 50 && !status->is_charging) {  // Warn if voltage jumps >50 mV while not charging
                ESP_LOGW(TAG, "Voltage jump detected: %dmV -> %dmV (jump=%dmV)", 
                         last_stable_voltage, status->voltage_mV, jump);
                /* Extra hint if jump is very large */
                if (jump > 70) {
                    ESP_LOGI(TAG, "Large voltage jump may indicate charger connected");
                }
            }
        }
        
        if (status->voltage_mV > 4300) {
            ESP_LOGW(TAG, "Voltage reading too high: %dmV (>4.3V), possible read error", status->voltage_mV);
            /* Re-read once if the voltage looks unrealistic */
            vTaskDelay(pdMS_TO_TICKS(10));
            if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_VOLTAGE, &value) == ESP_OK) {
                status->voltage_mV = value;  // use raw re-read value without bias
                ESP_LOGI(TAG, "Re-read voltage: %dmV", status->voltage_mV);
            }
        }
        
        last_stable_voltage = status->voltage_mV;
        
        /* Emit a low-voltage warning once until the pack recovers */
        if (status->voltage_mV < 3000 && status->voltage_mV > 1000 && !low_voltage_warning) {
            ESP_LOGW(TAG, "LOW BATTERY: Voltage: %dmV (< 3.0V)", status->voltage_mV);
            ESP_LOGW(TAG, "Battery deeply discharged. Please charge immediately!");
            low_voltage_warning = true;
        } else if (status->voltage_mV < 1000) {
            /* Treat readings below 1 V as invalid */
            ESP_LOGW(TAG, "Battery voltage read error or no battery: %dmV", status->voltage_mV);
            status->voltage_mV = 0;  // Treat invalid voltage as zero
        } else if (status->voltage_mV >= 3200 && low_voltage_warning) {
            ESP_LOGI(TAG, "Battery voltage recovered: %dmV", status->voltage_mV);
            low_voltage_warning = false;  // Reset warning when voltage recovers
        }
    }
    
    /* Legacy SOC diagnostic path (0-100%) */
    if (0) {
        /* BQ27427 SOC command access */
        uint16_t raw_soc = value;
        int soc_source = 0; /* 0: TI_GAUGE, 1: GAUGE_RM_FCC */
        
        /* Summary */
        uint8_t soc_low = raw_soc & 0xFF;
        uint8_t soc_high = (raw_soc >> 8) & 0xFF;
        
        /* Periodically emit SOC debug info */
        static uint32_t soc_debug_count = 0;
        if (soc_debug_count++ % 10 == 0) {  // Log every 10 samples
            ESP_LOGI(TAG, "SOC Debug: raw=0x%04X, low=%d, high=%d", raw_soc, soc_low, soc_high);
        }
        
        /* Default to low-byte SOC */
        status->soc_percent = soc_low;
        
        /* If low byte is zero, use high byte when valid */
        if (soc_low == 0 && soc_high > 0 && soc_high <= 100) {
            status->soc_percent = soc_high;
            ESP_LOGW(TAG, "Using high byte for SOC: %d%%", soc_high);
        }
        
        /* If SOC is invalid, try alternate sources */
        if (status->soc_percent == 0 || status->soc_percent > 100) {
            uint16_t rm, fcc;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_RM, &rm) == ESP_OK &&
                i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FCC, &fcc) == ESP_OK && fcc > 0) {
                uint8_t calculated_soc = (rm * 100) / fcc;
                ESP_LOGW(TAG, "SOC abnormal (%d%%), calculated from RM/FCC: %d%% (%d/%dmAh)", 
                         status->soc_percent, calculated_soc, rm, fcc);
                /* Cache values for stuck-SOC detection */
                status->soc_percent = calculated_soc;
                soc_source = 1; /* GAUGE_RM_FCC */
            }
        }
        
        /* Warn if sustained current exceeds the 2C threshold */
        if (status->soc_percent > 100) {
            status->soc_percent = 100;
        }
        
        /* Calibration state tracking helpers */
        static uint16_t last_soc = 0;
        static uint32_t log_count = 0;
        static bool calibration_mode = false;
        
        /* If SOC reads 0 while voltage is high, prompt calibration */
        if (status->soc_percent == 0 && status->voltage_mV > 3500) {
            if (!calibration_mode) {
                ESP_LOGW(TAG, "================================================");
                ESP_LOGW(TAG, "BQ27427 IN CALIBRATION MODE - SOC reads 0%%");
                ESP_LOGW(TAG, "Battery voltage: %.2fV", status->voltage_mV/1000.0);
                ESP_LOGW(TAG, "ACTION REQUIRED: Charge to 4.2V for calibration");
                ESP_LOGW(TAG, "================================================");
        }
        /* Periodic reminder while calibration pending */
        /* Emit reminder every 30 iterations (~5 s) */
            /* Write updated block data */
            if (++log_count % 30 == 0) {  // Log every 30 iterations (~5 s)
                ESP_LOGI(TAG, "Calibration needed! Voltage: %.2fV, BQ27427 SOC: %d%%", 
                         status->voltage_mV/1000.0, status->soc_percent);
            }
        } else if (status->soc_percent > 0 && calibration_mode) {
            ESP_LOGI(TAG, "BQ27427 calibration started! SOC now reading: %d%%", status->soc_percent);
            calibration_mode = false;
        }
        
        /* Log SOC source transitions for diagnostics */
        static int last_soc_source = -1;
        if (first_read || raw_soc != last_soc || (++log_count % 60 == 0) || (last_soc_source != soc_source)) {
            if (!calibration_mode) {
                /* Detect SOC readings that are stuck and request calibration */
                static uint16_t stuck_soc = 0;
                static uint32_t stuck_count = 0;
                if (raw_soc == stuck_soc) {
                    stuck_count++;
                    if (stuck_count > 60) {  // Roughly after 5 minutes
                        ESP_LOGW(TAG, "SOC stuck at %d%% for >5 minutes, may need calibration", status->soc_percent);
                        stuck_count = 0;  // Reset stuck counter
                    }
                } else {
                    stuck_soc = raw_soc;
                    stuck_count = 0;
                }
            }
            last_soc = raw_soc;
            last_soc_source = soc_source;
        }
        
        /* Write checksum */
        if (status->soc_percent > 100) {
            ESP_LOGW(TAG, "BQ27427 SOC invalid: %d%% (>100%%)", status->soc_percent);
            status->soc_percent = 100;
        }
        
        if (first_read) {
            /* silenced */
            first_read = false;
        }
    }
    /* TI BQ27427 SOC (0x1C) returns two bytes (LSB/MSB) */
        /* Per TRM: Standard Commands are 2 bytes; write command code then read 2 bytes */
        uint16_t soc16 = 0;
        if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_SOC, &soc16) == ESP_OK) {
            uint8_t soc_low  = (uint8_t)(soc16 & 0xFF);  /* Low-byte SOC value (0-100%) */
            uint8_t soc_high = (uint8_t)((soc16 >> 8) & 0xFF);
            status->soc_percent = soc_low;
            if (status->soc_percent > 100) status->soc_percent = 100;

            /* Avoid leaving SOC at 0% by recalculating from RM/FCC */
            do {
                if (status->soc_percent > 0 && status->soc_percent <= 100) break;
                /* Fallback to MSB when LSB stays at 0 and MSB is 1..100 */
                if ((soc_low == 0) && (soc_high > 0) && (soc_high <= 100)) {
                    status->soc_percent = soc_high;
                    break;
                }
                /* Fallback: compute SOC from RM/FCC */
                uint16_t rm = 0, fcc = 0;
                if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_RM, &rm) == ESP_OK &&
                    i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FCC, &fcc) == ESP_OK && fcc > 0) {
                    uint8_t calc = (uint8_t)((rm * 100U) / fcc);
                    if (calc > 100) calc = 100;
                    status->soc_percent = calc;
                    break;
                }
            } while (0);
        }
    
    /* Current (mA) read directly from BQ27427 */
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_CURRENT, &value) == ESP_OK) {
        status->current_mA = (int16_t)value;
        
        /* Warn if the current reading looks unrealistic */
        static uint32_t current_log_count = 0;
        if (current_log_count++ < 5) {

            if (abs(status->current_mA) > 500) {
                ESP_LOGW(TAG, "Note: High current may be due to UART 3V3 connection!");
            }
        }
        
        /* For a 100 mAh pack, clamp readings to between -0.5C (50 mA) and +2 A */
        /* Example: 699 mA on a 100 mAh pack is ~7C and likely abnormal */
        if (status->current_mA > 2000) {
            ESP_LOGW(TAG, "Discharge current too high: %dmA (>2A max)", status->current_mA);
            status->current_mA = 2000;  // Limit discharge current to 2A
        } else if (status->current_mA < -100) {
            ESP_LOGW(TAG, "Charge current too high: %dmA (>100mA for 100mAh battery)", status->current_mA);
            status->current_mA = -50;  // Limit charge current magnitude to 0.5C (50 mA)
        }
        
        /* Warn if sustained current exceeds the 2C threshold */
        if (abs(status->current_mA) > 200) {  // 2C threshold
            static uint32_t last_warning = 0;
            uint32_t now = xTaskGetTickCount();
            if (now - last_warning > pdMS_TO_TICKS(30000)) {  // Warn at most every 30s
                ESP_LOGW(TAG, "Abnormal current for 100mAh battery: %dmA (%.1fC rate)", 
                         status->current_mA, (float)abs(status->current_mA) / 100.0f);
                ESP_LOGW(TAG, "This may indicate wrong battery configuration or calibration");
                last_warning = now;
            }
        }
        
        /* Treat currents within +/-1 mA as zero (filter noise) */
        if (abs(status->current_mA) < 1) {
            status->current_mA = 0;  // Treat small currents as zero
        }
    }

    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_TEMP, &value) == ESP_OK) {
        status->temperature_C = (value / 10) - 273;  // K to celus degree
    }

    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &value) == ESP_OK) {
        status->charge_complete = (value & 0x0020) != 0;  // FC flag
    }
    
    /* Attempt to read BQ21080 charge status (best-effort) */
    uint8_t bq21080_status = 0;
    bool bq21080_read_ok = false;
    if (bq21080_read_status(&bq21080_status, NULL) == ESP_OK) {
        bq21080_read_ok = true;
        /* BQ21080 register map */
        uint8_t charge_stat = (bq21080_status >> 4) & 0x07;
        
        /* Log charge state changes but don't cache - always update status */
        static uint8_t last_charge_stat_log = 0xFF;
        if (charge_stat != last_charge_stat_log) {
            ESP_LOGI(TAG, "BQ21080 charge state changed: 0x%02X -> 0x%02X", last_charge_stat_log, charge_stat);
            last_charge_stat_log = charge_stat;
        }
        
        /* Always notify tasks for real-time updates (removed caching) */
        if (led_task_handle) {
            xTaskNotifyGive(led_task_handle);
        }
        if (battery_monitor_task_handle) {
            xTaskNotifyGive(battery_monitor_task_handle);
        }
        
        switch(charge_stat) {
            case 0x00:
                status->is_charging = false;
                ESP_LOGD(TAG, "BQ21080: Not charging");
                break;
            case 0x01:
                status->is_charging = true;
                ESP_LOGD(TAG, "BQ21080: Pre-charge");
                break;
            case 0x02:
                status->is_charging = true;
                ESP_LOGD(TAG, "BQ21080: Fast charge");
                break;
            case 0x03:
                status->charge_complete = true;
                status->is_charging = false;
                ESP_LOGD(TAG, "BQ21080: Charge complete");
                break;
            default:
                status->is_charging = false;
                ESP_LOGW(TAG, "BQ21080: Fault or unknown state: 0x%02X", charge_stat);
                break;
        }
    }
    
    /* If BQ21080 I2C broken,use BQ27427 to determine charging status instead */
    if (!bq21080_read_ok) {
        static bool last_charge_state = false;
        /* Charging if <= -10 mA; discharging if >= +10 mA; else keep last */
        if (status->current_mA <= -10) {
            last_charge_state = true;
        } else if (status->current_mA >= 10) {
            last_charge_state = false;
        }
        status->is_charging = last_charge_state;
    }

    if (0) { 
        uint16_t flags;
        if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_FLAGS, &flags) == ESP_OK) {
            /* Warn if charging is not working */
            static bool flag_warning_shown = false;
            if ((flags & 0x0080) && !status->is_charging && !flag_warning_shown) {
                ESP_LOGW(TAG, "FLAGS shows CHG bit set (0x%04X) but no charging current detected", flags);
                ESP_LOGW(TAG, "This may be a false positive from BQ27427");
                flag_warning_shown = true;
            }
        }
    }
    
    return ESP_OK;
}  /* End of bq27427_read_status function */

/* LED Control Task - Updated with new requirements */
static void led_control_task(void *arg)
{
    /* Initialize breathing LED based on wake source */
    uint8_t breath_value = 0;
    int8_t  breath_direction = 1;

    /* If woken by button, start breathing at maximum brightness */
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool button_wake = is_button_wake_cause(wakeup_reason);
    if (button_wake) {
        /* Set breathing parameters for maximum brightness */
        breath_value = 250;  /* Start at maximum */
        breath_direction = -1;  /* Start by falling */
        if (!quick_boot_mode) ESP_LOGI(TAG, "Button wake - will start with yellow at maximum via PWM");
    }

    /* Disable GPIO hold from deep sleep */
    gpio_hold_dis(LED_R_GPIO);
    gpio_hold_dis(LED_G_GPIO);
    gpio_hold_dis(LED_B_GPIO);

    if (BAT_LED_DEBUG) ESP_LOGI(TAG, "LED control task started");
    /* Continuous breathing cycle - using non-linear steps */
    uint32_t tick_counter = 0;
    /* removed: was used for old fade animation */
    static uint32_t adv_log_counter = 0;
    /* Button long-press tracking */
    static bool btn_was_pressed = false;
    static TickType_t btn_press_tick = 0;
    
    /* Initialize GPIOs (handled by LEDC channel config below) */
    if (BAT_LED_DEBUG) ESP_LOGW(TAG, "Using Red+Green+Blue LED - Green on GPIO12");
    if (BAT_LED_DEBUG) ESP_LOGW(TAG, "Yellow approximated with warm orange (Red + tiny Blue)");
    
    /* Initialize LEDs - if not button wake, turn them OFF */
    /* Skip GPIO manipulation if early init already set them */
    if (!button_wake && !quick_boot_mode) {
        gpio_set_level(LED_R_GPIO, LED_OFF);
        gpio_set_level(LED_G_GPIO, LED_OFF);
        gpio_set_level(LED_B_GPIO, LED_OFF);
    }
    /* If button wake or quick boot, LEDs already configured */
    
    /* Check if waking from deep sleep - log only, no LED feedback */
    /* wakeup_reason already checked above for LED initialization */
    if (is_button_wake_cause(wakeup_reason)) {
        ESP_LOGI(TAG, "Woken from deep sleep by button press (LED starting at maximum)");
        /* LED will be controlled by main loop based on actual state */
    }
    
    /* LED test sequence removed - go directly to normal operation */
    
    /* Configure LEDC PWM - skip if already done in early init */
    if (!quick_boot_mode) {
        /* Configure LEDC PWM timer */
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 5000,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);

        /* Configure Red LED PWM channel */
        ledc_channel_config_t ledc_channel_red = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = button_wake ? yellow_hold_red_duty() : 0,  /* If button wake, start with yellow */
            .gpio_num   = LED_R_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0,
            .flags.output_invert = 1,
        };
        ledc_channel_config(&ledc_channel_red);

        /* Blue LED - Channel 1 (GPIO7) */
        ledc_channel_config_t ledc_channel_blue = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LEDC_CHANNEL_1,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = LED_B_GPIO,
            .duty       = 0,
            .hpoint     = 0,
            .flags.output_invert = 1,
        };
        ledc_channel_config(&ledc_channel_blue);

        /* Green LED - Channel 2 (GPIO12) */
        ledc_channel_config_t ledc_channel_green = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LEDC_CHANNEL_2,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = LED_G_GPIO,
            .duty       = button_wake ? yellow_hold_green_duty() : 0,  /* If button wake, start with yellow */
            .hpoint     = 0,
            .flags.output_invert = 1,
        };
        ledc_channel_config(&ledc_channel_green);
    }

    /* If woken by button, ensure yellow LED is showing */
    if (button_wake || quick_boot_mode) {
        /* If early init was done, PWM is already set, just ensure it's active */
        if (!quick_boot_mode) {
            /* Force immediate update to ensure yellow shows right away */
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            ESP_LOGI(TAG, "Button wake - Yellow LED enabled via PWM");
        }
    }
    
    static bool first_loop = true;

    TickType_t solid_yellow_until = 0;
    if (button_wake || quick_boot_mode) {
        solid_yellow_until = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    }

    while (1) {
        /* For button wake, skip the first wait to start LED immediately */
        if (!first_loop || !button_wake) {
            /* Wait for charger INT/button notify, or timeout every 20ms to refresh */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        } else if (first_loop && button_wake) {
            /* First loop after button wake - no delay, go straight to LED control */
            if (!quick_boot_mode) ESP_LOGI(TAG, "First loop after button wake - immediate LED control");
        }
        TickType_t current_tick = xTaskGetTickCount();
        if (BAT_LED_DEBUG && (tick_counter % 250 == 0)) {  /* 5 seconds at 20ms tick */
            ESP_LOGI(TAG, "LED Debug: charging=%d, complete=%d, ble_state=%d",
                     battery_status.is_charging, battery_status.charge_complete, ble_connection_state);
        }
        
        bool hold_solid_yellow = (solid_yellow_until != 0) && (current_tick < solid_yellow_until);
        if (hold_solid_yellow) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, yellow_hold_red_duty());
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, yellow_hold_green_duty());
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            tick_counter++;
            first_loop = false;
            continue;
        }
        if (solid_yellow_until != 0 && current_tick >= solid_yellow_until) {
            solid_yellow_until = 0;
        }

        /* Step 1: Check if system is shutting down first (highest priority) */
        if (system_shutting_down) {
            /* Turn off all LEDs using PWM */
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

            /* Also set GPIO directly to ensure LEDs are off */
            gpio_set_level(LED_R_GPIO, LED_OFF);
            gpio_set_level(LED_G_GPIO, LED_OFF);
            gpio_set_level(LED_B_GPIO, LED_OFF);

            /* Hold GPIO states during deep sleep to prevent leakage */
            gpio_hold_en(LED_R_GPIO);
            gpio_hold_en(LED_G_GPIO);
            gpio_hold_en(LED_B_GPIO);
            gpio_deep_sleep_hold_en();

            /* Wait for button release, so next press can wake device */
            if (BAT_LED_DEBUG) ESP_LOGI(TAG, "Waiting for button release before deep sleep...");
            while (gpio_get_level(POWER_BUTTON_GPIO) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            /* Configure deep sleep GPIO wakeup on next button press (active-low) */
            uint64_t mask = (1ULL << POWER_BUTTON_GPIO);
            esp_err_t werr = esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
            if (werr != ESP_OK) {
                ESP_LOGE(TAG, "gpio wakeup config failed: %s", esp_err_to_name(werr));
            }

            ESP_LOGI(TAG, "Entering deep sleep. Press button to wake.");
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_deep_sleep_start();
            continue;
        }
        
        /* Step 2: Power button detection and timing (GPIO3, active-low) */
        {
            int level = gpio_get_level(POWER_BUTTON_GPIO);
            bool pressed = (level == 0);
            TickType_t now = xTaskGetTickCount();

            /* On first loop after wake, if button is pressed, start timing */
            if (first_loop && pressed) {
                btn_was_pressed = true;
                btn_press_tick = now;
                ESP_LOGI(TAG, "First loop: button pressed, starting timer");
            } else if (pressed && !btn_was_pressed) {
                /* Button just pressed - start timing */
                btn_was_pressed = true;
                btn_press_tick = now;
            } else if (pressed && btn_was_pressed) {
                /* Button still pressed - check for long press */
                if ((now - btn_press_tick) >= pdMS_TO_TICKS(3000)) {
                    /* Long press detected - trigger shutdown */
                    system_shutting_down = true;
                    ESP_LOGI(TAG, "Power button long-press detected (>3s), shutting down");
                    /* Turn off LEDs immediately when shutdown triggered */
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                    tick_counter++;  /* Still increment counter */
                    continue;  /* Skip rest of loop to handle shutdown next iteration */
                }
                /* Still pressing but not yet 3 seconds - just wait, no LED */
            } else if (!pressed && btn_was_pressed) {
                /* Button released */
                btn_was_pressed = false;
            }
        }
        
        /* Handle charging states first (highest priority) */
        /* Read battery status with mutex protection for consistency */
        bool is_charging = false;
        bool charge_complete = false;
        if (battery_status_mutex && xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            is_charging = battery_status.is_charging;
            charge_complete = battery_status.charge_complete;
            xSemaphoreGive(battery_status_mutex);
        }
        
        if (charge_complete) {
            /* Charging Full: Yellow solid (balanced with per-channel gains) */
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, yellow_hold_red_duty());  /* Red */
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, yellow_hold_green_duty());  /* Green */
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            /* No vTaskDelay here - using ulTaskNotifyTake timeout at line 1656 */
            
        } else if (is_charging) {
            /* When Charging: Red continuous breathing */
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);  /* Blue off */
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            
            /* Exponential breathing curve - stays longer in dark region */
            /* NOTE: Using 10% increment (double) to compensate for slower loop timing */
            if (breath_direction == 1) {  /* Rising */
                if (breath_value == 0) {
                    breath_value = 1;  /* Start from 1 */
                } else {
                    /* Adaptive increase for smooth transition - adjusted for 4s cycle */
                    uint8_t increment;
                    if (breath_value < 100) {
                        /* Low range: use 6% steps (was 10%) */
                        increment = (breath_value * 6) / 100;
                        if (increment < 1) increment = 1;
                    } else if (breath_value < 200) {
                        /* Mid-range: use 4% steps (was 7%) */
                        increment = (breath_value * 4) / 100;
                        if (increment < 1) increment = 1;
                    } else {
                        /* Near peak: use 2.5% steps for smooth transition (was 4%) */
                        increment = (breath_value * 25) / 1000;  /* 2.5% */
                        if (increment < 1) increment = 1;
                    }
                    breath_value += increment;
                }
                if (breath_value >= 250) {
                    breath_value = 250;
                    breath_direction = -1;
                }
            } else {  /* Falling */
                if (breath_value <= 1) {
                    breath_value = 0;
                    breath_direction = 1;
                } else {
                    /* Adaptive decrease for smooth transition - adjusted for 4s cycle */
                    uint8_t decrement;
                    if (breath_value > 200) {
                        /* At peak: use 2.5% steps for smooth transition (was 4%) */
                        decrement = (breath_value * 25) / 1000;  /* 2.5% */
                        if (decrement < 1) decrement = 1;
                    } else if (breath_value > 100) {
                        /* Mid-range: use 4% steps (was 7%) */
                        decrement = (breath_value * 4) / 100;
                        if (decrement < 1) decrement = 1;
                    } else {
                        /* Low range: use 6% steps (was 10%) */
                        decrement = (breath_value * 6) / 100;
                        if (decrement < 1) decrement = 1;
                    }
                    
                    if (breath_value > decrement) {
                        breath_value -= decrement;
                    } else {
                        breath_value = 0;
                    }
                }
            }
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, breath_value);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            /* No vTaskDelay here - using ulTaskNotifyTake timeout at line 1656 */ 
            
        } else if (button_wake && ble_connection_state == BLE_STATE_DISCONNECTED) {
            /* Button wake but BLE not started yet - keep LED at maximum */
            static bool wake_led_shown = false;
            if (!wake_led_shown) {
                if (!quick_boot_mode) ESP_LOGI(TAG, "Button wake: keeping LED solid bright until BLE starts");
                wake_led_shown = true;
            }
            /* Use same values as breathing LED at maximum for consistent yellow */
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, yellow_hold_red_duty());
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, yellow_hold_green_duty());
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        } else {
            /* Not charging - handle BLE states */
            switch (ble_connection_state) {
                case BLE_STATE_ADVERTISING:
                    /* Advertising: Yellow continuous breathing */

                    /* Special case: if button wake and first time entering advertising, keep LED at max briefly */
                    static bool button_wake_handled = false;
                    if (button_wake && !button_wake_handled) {
                        /* Keep LED at maximum for a moment after wake */
                        breath_value = 250;
                        breath_direction = -1;
                        button_wake_handled = true;
                        if (!quick_boot_mode) ESP_LOGI(TAG, "Button wake: keeping LED at maximum before breathing");
                    }

                    /* Exponential breathing curve - stays longer in dark region */
                    /* NOTE: Using 10% increment (double) to compensate for slower loop timing */
                    if (breath_direction == 1) {  /* Rising */
                        if (breath_value == 0) {
                            breath_value = 1;  /* Start from 1 */
                        } else {
                            /* Adaptive increase for smooth transition - adjusted for 4s cycle */
                            uint8_t increment;
                            if (breath_value < 100) {
                                /* Low range: use 6% steps (was 10%) */
                                increment = (breath_value * 6) / 100;
                                if (increment < 1) increment = 1;
                            } else if (breath_value < 200) {
                                /* Mid-range: use 4% steps (was 7%) */
                                increment = (breath_value * 4) / 100;
                                if (increment < 1) increment = 1;
                            } else {
                                /* Near peak: use 2.5% steps for smooth transition (was 4%) */
                                increment = (breath_value * 25) / 1000;  /* 2.5% */
                                if (increment < 1) increment = 1;
                            }
                            breath_value += increment;
                        }
                        if (breath_value >= 250) {
                            breath_value = 250;
                            breath_direction = -1;
                        }
                    } else {  /* Falling */
                        if (breath_value <= 1) {
                            breath_value = 0;
                            breath_direction = 1;
                        } else {
                            /* Adaptive decrease for smooth transition - adjusted for 4s cycle */
                            uint8_t decrement;
                            if (breath_value > 200) {
                                /* At peak: use 2.5% steps for smooth transition (was 4%) */
                                decrement = (breath_value * 25) / 1000;  /* 2.5% */
                                if (decrement < 1) decrement = 1;
                            } else if (breath_value > 100) {
                                /* Mid-range: use 4% steps (was 7%) */
                                decrement = (breath_value * 4) / 100;
                                if (decrement < 1) decrement = 1;
                            } else {
                                /* Low range: use 6% steps (was 10%) */
                                decrement = (breath_value * 6) / 100;
                                if (decrement < 1) decrement = 1;
                            }
                            
                            if (breath_value > decrement) {
                                breath_value -= decrement;
                            } else {
                                breath_value = 0;
                            }
                        }
                    }
                    
                    if (BAT_LED_DEBUG) {
                        if (++adv_log_counter % 100 == 0) {  /* every 2 seconds */
                            ESP_LOGI(TAG, "ADV LED: Red(CH0)=%d, Green(CH2)=%d (balanced for yellow)",
                                     breath_value, breath_value);
                        }
                    }
                    /* Yellow breathing: apply per-channel gains to balance color */
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, led_scale(breath_value, YELLOW_R_GAIN_PCT));
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, led_scale(breath_value, YELLOW_G_GAIN_PCT));
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                    /* No vTaskDelay here - using ulTaskNotifyTake timeout at line 1656 */
                    break;
                    
                case BLE_STATE_CONNECTED:
                case BLE_STATE_TRANSMITTING:
                    /* Connected: Yellow short flash (0.4s) every 4s */
                    if ((tick_counter % 200) < 20) {  /* 0.4s out of 4s (20ms tick = 20*20ms = 0.4s) */
                        /* Yellow flash (use balanced static values) */
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, yellow_hold_red_duty());
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, yellow_hold_green_duty());
                    } else {
                        /* Off */
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                    }
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                    /* No vTaskDelay here - using ulTaskNotifyTake timeout at line 1656 */
                    break;
                    
                case BLE_STATE_DISCONNECTED:
                default:
                    /* Powered on but not advertising/connected - LED off */
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                    /* No vTaskDelay here - using ulTaskNotifyTake timeout at line 1656 */
                    break;
            }
        }
        tick_counter++;
        first_loop = false;  /* Clear first loop flag */
    }
}

/* Quick updater: read only BQ21080 charge state and update flags */
static void battery_quick_update_charge_state(void)
{
    uint8_t bq_status = 0;
    if (bq21080_read_status(&bq_status, NULL) == ESP_OK) {
        uint8_t charge_stat = (bq_status >> 4) & 0x07;
        bool charging = (charge_stat == 0x01) || (charge_stat == 0x02);
        bool charge_complete = (charge_stat == 0x03);

        if (battery_status_mutex &&
            xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            battery_status.is_charging = charging;
            battery_status.charge_complete = charge_complete;
            xSemaphoreGive(battery_status_mutex);
        }
    }
}

/* Warn if sustained current exceeds the 2C threshold */
static void battery_monitor_task(void *arg)
{
    TickType_t last_log_time = 0;
    // static uint32_t impedance_check_count = 0;  // Unused variable
    static bool calibration_mode = false;
    static uint16_t max_voltage_seen = 0;
    static uint16_t min_voltage_seen = 5000;
    
    /* Perform a full status update at most every 100ms; react to charge IRQ immediately */
    const TickType_t FULL_UPDATE_PERIOD = pdMS_TO_TICKS(100);
    TickType_t last_full_update = 0;

    while (1) {
        static uint32_t impedance_check_count = 0;

        /* Wait for CHG_INT notification or 10ms timeout to stay in sync with sensor task */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        /* Fast-path: on charger status edge, update only charge flags immediately */
        if (notified > 0) {
            battery_quick_update_charge_state();
        }

        /* Throttle heavy gauge + charger reads to 10 Hz to reduce I2C contention */
        TickType_t now = xTaskGetTickCount();
        if (now - last_full_update >= FULL_UPDATE_PERIOD) {
            battery_status_t temp_status;
            bq27427_read_status(&temp_status);

            /* Update global status with mutex */
            if (battery_status_mutex &&
                xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(&battery_status, &temp_status, sizeof(battery_status_t));
                xSemaphoreGive(battery_status_mutex);
            }

            last_full_update = now;
        }
        
        /* Track last stable voltage reading */
        if (battery_status.voltage_mV > max_voltage_seen) {
            max_voltage_seen = battery_status.voltage_mV;
        }
        if (battery_status.voltage_mV < min_voltage_seen && battery_status.voltage_mV > 2500) {
            min_voltage_seen = battery_status.voltage_mV;
        }
        
        /* Log BQ21080 status changes */
        if (!calibration_mode && (max_voltage_seen > 4100 || min_voltage_seen < 3400)) {
            calibration_mode = true;
            ESP_LOGI(TAG, "========== CALIBRATION CYCLE DETECTED ==========");
            ESP_LOGI(TAG, "Good! This will help BQ27427 learn your battery");
        }
        
        /* Calibration helper: print hints when near full/empty */
        if (calibration_mode) {
            if (battery_status.is_charging && battery_status.voltage_mV > 4150) {
                ESP_LOGI(TAG, "Calibration: Battery nearly full (%.2fV)", battery_status.voltage_mV / 1000.0);
                ESP_LOGI(TAG, "   Keep charging until 4.20V, then let rest 1 hour");
            } else if (!battery_status.is_charging && battery_status.voltage_mV < 3400) {
                ESP_LOGI(TAG, "Calibration: Battery nearly empty (%.2fV)", battery_status.voltage_mV / 1000.0);
                ESP_LOGI(TAG, "   Good! Now charge to full for calibration");
            }
        }
        
        /* Every 5 seconds */
        if (xTaskGetTickCount() - last_log_time > pdMS_TO_TICKS(5000)) {
            /* Check Impedance Track status periodically */
            if (++impedance_check_count >= 12) {  // Check Impedance Track roughly once a minute
                uint16_t ctrl_status;
                if (bq27427_control_cmd(BQ27427_CTRL_STATUS) == ESP_OK) {
                    if (bq27427_read_control_result(&ctrl_status) == ESP_OK) {
                        if (ctrl_status & 0x0080) {  // INITCOMP
                            ESP_LOGI(TAG, "Impedance Track is ACTIVE and learning");
                        }
                    }
                }
                impedance_check_count = 0;
            }
            
            last_log_time = xTaskGetTickCount();
        }

        /* No additional delay needed - ulTaskNotifyTake already waits 10ms */
    }
}

/* Write updated block data */
/* Write updated block data */
static void IRAM_ATTR charge_isr_handler(void* arg)
{
    /* Notify both LED and battery monitor tasks for immediate response */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Notify LED task */
    if (led_task_handle) {
        vTaskNotifyGiveFromISR(led_task_handle, &xHigherPriorityTaskWoken);
    }
    
    /* Notify battery monitor task to read status immediately */
    if (battery_monitor_task_handle) {
        vTaskNotifyGiveFromISR(battery_monitor_task_handle, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
/* I2C bus scan helper (debug) */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    
    for (uint8_t addr = 3; addr < 0x78; addr++) {
        if (addr % 16 == 0) {
            printf("\n%.2x:", addr);
        }
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf(" %.2x", addr);
            ESP_LOGI(TAG, "Found device at address: 0x%02x", addr);
        } else {
            printf(" --");
        }
    }
    printf("\n");
}

/* Cache values for stuck-SOC detection */
/* Write updated block data */
esp_err_t battery_init(void)
{
    if (!quick_boot_mode) {
        ESP_LOGI(TAG, "Initializing battery management...");
    } else {
        ESP_LOGI(TAG, "Quick boot: skipping verbose battery init logs");
    }

    /* Check if woken by button - will be used to set initial LED state */
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    if (is_button_wake_cause(wakeup)) {
        if (!quick_boot_mode) ESP_LOGI(TAG, "Button wake detected - will set yellow LED via PWM in LED task");
    }
    
    /* Create mutex for battery_status protection */
    battery_status_mutex = xSemaphoreCreateMutex();
    if (battery_status_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create battery status mutex");
    }
    
    /* Initialize the I2C driver before querying BQ27427 */
    uint16_t test_read;
    esp_err_t i2c_status = i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_VOLTAGE, &test_read);
    if (i2c_status == ESP_ERR_INVALID_STATE || i2c_status == ESP_FAIL) {
        ESP_LOGI(TAG, "I2C not initialized, initializing now...");
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        } else {
            ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
            }
        }
    } else if (i2c_status == ESP_OK) {
        ESP_LOGI(TAG, "I2C already initialized, voltage read: %dmV", test_read);
    }
    /* Run an I2C bus scan for visibility */
    i2c_scan();
    
    /* Configure all LED GPIOs - skip if already done for button wake */
    esp_sleep_wakeup_cause_t wake_reason = esp_sleep_get_wakeup_cause();
    if (quick_boot_mode) {
        ESP_LOGI(TAG, "Quick boot: preserving yellow LED state from early init");
    } else if (!is_button_wake_cause(wake_reason)) {
        ESP_LOGI(TAG, "Configuring RGB LEDs - Green on GPIO12");

        /* Configure LED GPIO pins */
        gpio_config_t led_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << LED_R_GPIO) | (1ULL << LED_G_GPIO) | (1ULL << LED_B_GPIO),
            .pull_down_en = 0,
            .pull_up_en = 0,
        };
        gpio_config(&led_conf);

        /* Initialize all LEDs off */
        gpio_set_level(LED_R_GPIO, LED_OFF);
        gpio_set_level(LED_G_GPIO, LED_OFF);
        gpio_set_level(LED_B_GPIO, LED_OFF);
    } else {
        ESP_LOGI(TAG, "Button wake - keeping yellow LED on, skipping GPIO reconfiguration");
    }
    
    /* Write updated block data */
    gpio_config_t int_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Candidate addresses
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CHG_INT_GPIO) | (1ULL << POWER_BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&int_conf);
    
    /* Install GPIO ISRs for charger interrupt and power button */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CHG_INT_GPIO, charge_isr_handler, NULL);
    
    /* BQ27427 configuration check */
    uint16_t test_value;
    bool battery_ic_found = false;
    bool battery_present = false;
    
    if (i2c_read_reg16(BQ27427_I2C_ADDR, BQ27427_CMD_VOLTAGE, &test_value) == ESP_OK) {
        ESP_LOGI(TAG, "BQ27427 detected, voltage reading: %dmV", test_value);
        battery_ic_found = true;
        
        /* Infer battery presence from the measured voltage */
        if (test_value >= 3000 && test_value <= 4300) {
            battery_present = true;
            ESP_LOGI(TAG, "Battery present: %.2fV", test_value/1000.0);
        } else if (test_value >= 2500 && test_value < 3000) {
            ESP_LOGW(TAG, "Battery voltage very low: %.2fV - may be deeply discharged or not present", test_value/1000.0);
            ESP_LOGW(TAG, "If battery is connected, charge immediately!");
            battery_present = false;  // Treat as no battery for < 3.0V
        } else {
            ESP_LOGW(TAG, "No battery detected (abnormal reading: %dmV)", test_value);
            ESP_LOGI(TAG, "BQ27427 IC present but battery not connected");
            battery_present = false;
        }
        
        /* Soft-reset the BQ27427 before configuration */
        uint16_t device_info;
        uint8_t cmd_data[2] = {0x42, 0x00}; // Soft reset command
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BQ27427_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);  // Control register
        i2c_master_write(cmd, cmd_data, 2, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BQ27427 soft reset sent");
            vTaskDelay(pdMS_TO_TICKS(100));
            
            /* Warn if sustained current exceeds the 2C threshold */
            cmd_data[0] = 0x01; cmd_data[1] = 0x00;  // Device type command
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (BQ27427_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, 0x00, true);
            i2c_master_write(cmd, cmd_data, 2, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            
            vTaskDelay(pdMS_TO_TICKS(10));
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x00, &device_info) == ESP_OK) {
                ESP_LOGI(TAG, "BQ27427 Device Type: 0x%04X", device_info);
            }
            
            /* Configure BQ27427 only when a battery is present */
            if (battery_present) {
                ESP_LOGI(TAG, "===== CONFIGURING BQ27427 FOR BATTERY =====");
                ESP_LOGI(TAG, "Target: 100mAh LiPo 4.2V battery");
                ESP_LOGI(TAG, "CHEM_ID=0x04B2 (1202), Design Capacity=100mAh");
                
                /* Reset the gauge before writing new parameters */
                ESP_LOGI(TAG, "Resetting BQ27427...");
                bq27427_control_cmd(BQ27427_CTRL_SOFT_RESET);
                vTaskDelay(pdMS_TO_TICKS(300));

#if BAT_BQ27427_SAFE_MODE
                /* Ensure gauging is enabled to report SOC without a learn cycle */
                bq27427_control_cmd(BQ27427_CTRL_IT_ENABLE);
                vTaskDelay(pdMS_TO_TICKS(100));
                ESP_LOGI(TAG, "Sending BAT_INSERT command...");
                bq27427_control_cmd(BQ27427_CTRL_BAT_INSERT);
                vTaskDelay(pdMS_TO_TICKS(300));
                ESP_LOGW(TAG, "SAFE MODE: Skipping BQ27427 Data Memory writes (CHEM_ID/Capacity/CC Gain)");
                ESP_LOGW(TAG, "SAFE MODE: Using gauge defaults + IT_ENABLE + BAT_INSERT");
#else
                esp_err_t cfg_ret = bq27427_configure_battery_params();
                if (cfg_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to configure BQ27427 data memory (%d)", cfg_ret);
                }
#endif

                bq27427_configure();
            } else {
                ESP_LOGW(TAG, "===== SKIPPING BQ27427 CONFIGURATION =====");
                ESP_LOGW(TAG, "No battery detected - configuration skipped");
                ESP_LOGW(TAG, "Insert battery and restart to configure");
            }
            
            /* Summary */
            uint16_t design_capacity;
            if (0) { /* disabled: 0x08 is not Design Capacity standard command */
                ESP_LOGI(TAG, "BQ27427 Design Capacity from cmd 0x08: %dmAh", design_capacity);
                
                /* Check whether the gauge reports the desired design capacity */
                if (design_capacity != 100) {
                    ESP_LOGW(TAG, "Design capacity mismatch: Read %dmAh, Expected 100mAh", design_capacity);
                    ESP_LOGI(TAG, "Battery is 100mAh LiPo, fuel gauge calibration needed");
                    ESP_LOGI(TAG, "For accurate SOC, perform a full charge-discharge cycle");
                } else {
                    ESP_LOGI(TAG, "Design capacity correct for 100mAh battery");
                }
            }
            
            /* Warn if sustained current exceeds the 2C threshold */
            uint16_t full_capacity;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x0E, &full_capacity) == ESP_OK) {
                ESP_LOGI(TAG, "BQ27427 Full Charge Capacity: %dmAh", full_capacity);
            }
            
            /* Warn if sustained current exceeds the 2C threshold */
            uint16_t remaining_capacity;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x0C, &remaining_capacity) == ESP_OK) {
                ESP_LOGI(TAG, "BQ27427 Remaining Capacity: %dmAh", remaining_capacity);
            }
            
            /* Warn if sustained current exceeds the 2C threshold */
            int16_t avg_current;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x10, (uint16_t*)&avg_current) == ESP_OK) {
                ESP_LOGI(TAG, "BQ27427 Average Current: %dmA", avg_current);
            }
            
            /* Temperature diagnostics */
            uint16_t temperature;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x02, &temperature) == ESP_OK) {
                int temp_c = (temperature / 10) - 273;
                ESP_LOGI(TAG, "BQ27427 Temperature: %d?C", temp_c);
                ESP_LOGI(TAG, "BQ27427 Temperature: %d C", temp_c);
            }  /* end temperature diagnostic block */
            
            /* Decode FLAGS register for readability */
            uint16_t flags;
            if (i2c_read_reg16(BQ27427_I2C_ADDR, 0x06, &flags) == ESP_OK) {
                ESP_LOGI(TAG, "BQ27427 FLAGS: 0x%04X", flags);
                if (flags & 0x0008) ESP_LOGI(TAG, "  Battery detected (DET)");
                if (flags & 0x0008) ESP_LOGI(TAG, "  Battery detected (DET)");
                if (flags & 0x0020) ESP_LOGI(TAG, "  Charge complete (FC)");
                if (flags & 0x0080) ESP_LOGI(TAG, "  Charging detected (CHG)");
                if (flags & 0x0200) ESP_LOGI(TAG, "  Discharging (DSG)");
                if (flags & 0x8000) ESP_LOGI(TAG, "  Battery inserted (BAT_DET)");
                /* Classify current direction for debugging */
                if (avg_current < -2) {
                    ESP_LOGI(TAG, "CHARGING: Current = %dmA into battery", -avg_current);
                    if (-avg_current < 10) {
                        ESP_LOGW(TAG, "  - Expected: 20-100mA (0.2C-1C rate)");
                        ESP_LOGW(TAG, "  - Actual: %dmA", -avg_current);
                        ESP_LOGW(TAG, "  - Possible causes: ");
                        ESP_LOGW(TAG, "    1. USB power insufficient");
                        ESP_LOGW(TAG, "    2. BQ21080 not working");
                        ESP_LOGW(TAG, "    3. Battery protection triggered");
                    }
                } else if (avg_current > 2) {
                    ESP_LOGI(TAG, "DISCHARGING: Current = %dmA from battery", avg_current);
                } else {
                    ESP_LOGI(TAG, "IDLE: No significant current flow");
                }
            }  /* end FLAGS diagnostic block */
            
            /* Ensure SOC output is enabled */
            /* Simulate a battery insert so the gauge refreshes SOC */
            /* Ensure gauging is enabled so SOC reports without learn cycle */
            bq27427_control_cmd(BQ27427_CTRL_IT_ENABLE);
            vTaskDelay(pdMS_TO_TICKS(100));
            cmd_data[0] = 0x0C; cmd_data[1] = 0x00;  // BAT_INSERT command
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (BQ27427_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, 0x00, true);
            i2c_master_write(cmd, cmd_data, 2, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            ESP_LOGI(TAG, "BQ27427 BAT_INSERT command sent");
            
            vTaskDelay(pdMS_TO_TICKS(500));  // Give BQ27427 time to settle after BAT_INSERT
        }  /* end ret == ESP_OK block */
        
        /* If fuel gauge IC found, handle charger and start monitor task */
        if (battery_ic_found) {
        /* Configure the BQ21080 charger if present */
        /* Detect BQ21080 I2C address (0x6B preferred, avoid IMU @0x6A) */
        if (!bq21080_detect_address()) {
            ESP_LOGW(TAG, "BQ21080 not detected on I2C bus (0x6B/0x6A). Skipping charger init.");
        }
        if (s_bq21080_addr != 0x00 && bq21080_init() != ESP_OK) {
            ESP_LOGW(TAG, "BQ21080 init failed, charging control will be limited");
        }
        
        /* Continue setup even when the charger IC is absent */
        }
        /* Start battery monitor task (higher prio to stay responsive with BLE active) */
        xTaskCreate(battery_monitor_task, "battery_mon", 3072, NULL, 8, &battery_monitor_task_handle);
    }  /* end if (i2c_read_reg16(...) == ESP_OK) */
    else {
        ESP_LOGW(TAG, "BQ27427 not found - battery monitoring disabled");
        ESP_LOGW(TAG, "LED functions will still work for BLE status");
        /* Write updated block data */
        battery_status.soc_percent = 50;  // Default to mid-SOC when gauge unavailable
        battery_status.is_charging = false;
        battery_status.charge_complete = false;
    }
    
    /* Start LED task (ties into BLE indicators) */
    /* LED task above NimBLE host and sensor task for smooth UI */
    xTaskCreate(led_control_task, "led_ctrl", 2048, NULL, 4, &led_task_handle);
    
    ESP_LOGI(TAG, "Battery management initialized (battery IC %s)",
             battery_ic_found ? "found" : "not found");
    return ESP_OK;
}

/* Early LED initialization for button wake - minimal setup for fast response */
void battery_early_led_init(void)
{
    /* Set quick boot mode to reduce logs */
    quick_boot_mode = true;

    /* Release any deep-sleep hold so PWM can drive immediately */
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(LED_R_GPIO);
    gpio_hold_dis(LED_G_GPIO);
    gpio_hold_dis(LED_B_GPIO);

    /* Configure GPIO first for fastest response */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_R_GPIO) | (1ULL << LED_G_GPIO) | (1ULL << LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Ensure GPIO default to OFF before PWM takes over */
    gpio_set_level(LED_R_GPIO, LED_OFF);
    gpio_set_level(LED_G_GPIO, LED_OFF);
    gpio_set_level(LED_B_GPIO, LED_OFF);

    /* Quick LEDC timer setup for PWM */
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    /* Configure all three channels to prevent conflicts */
    /* Red channel for yellow */
    ledc_channel_config_t ledc_channel_red = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = yellow_hold_red_duty(),
        .gpio_num   = LED_R_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .flags.output_invert = 1,
    };
    ledc_channel_config(&ledc_channel_red);

    /* Blue channel - set to OFF */
    ledc_channel_config_t ledc_channel_blue = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,  /* Blue OFF */
        .gpio_num   = LED_B_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .flags.output_invert = 1,
    };
    ledc_channel_config(&ledc_channel_blue);

    /* Green channel for yellow */
    ledc_channel_config_t ledc_channel_green = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = yellow_hold_green_duty(),
        .gpio_num   = LED_G_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .flags.output_invert = 1,
    };
    ledc_channel_config(&ledc_channel_green);

    /* Force immediate update on all channels using PWM-based yellow */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, yellow_hold_red_duty());
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, yellow_hold_green_duty());
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

/* Snapshot the latest battery status */
__attribute__((used)) void battery_get_status(battery_status_t *status)
{
    if (status && battery_status_mutex) {
        if (xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(status, &battery_status, sizeof(battery_status_t));
            xSemaphoreGive(battery_status_mutex);
        }
    }
}

/* Cache values for stuck-SOC detection */
__attribute__((used)) uint8_t battery_get_soc(void)
{
    uint8_t soc = 0;
    if (battery_status_mutex && xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        soc = battery_status.soc_percent;
        xSemaphoreGive(battery_status_mutex);
    }
    return soc;
}

/* Warn if sustained current exceeds the 2C threshold */
__attribute__((used)) bool battery_is_charging(void)
{
    bool charging = false;
    if (battery_status_mutex && xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        charging = battery_status.is_charging;
        xSemaphoreGive(battery_status_mutex);
    }
    return charging;
}

/* Return battery voltage in millivolts */
__attribute__((used)) uint16_t battery_get_voltage_mv(void)
{
    uint16_t voltage = 0;
    if (battery_status_mutex && xSemaphoreTake(battery_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        voltage = battery_status.voltage_mV;
        xSemaphoreGive(battery_status_mutex);
    }
    return voltage;
}

/* Cache BLE state so the LED logic can use it */
__attribute__((used)) void battery_set_ble_state(uint8_t state)
{
    if (state <= BLE_STATE_TRANSMITTING) {
        ble_connection_state = state;
    }
}

/* Record the last sensor data transmission time */
__attribute__((used)) void battery_notify_data_transmit(void)
{
    last_data_transmit_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

