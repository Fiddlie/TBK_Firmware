#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"


/* Battery status snapshot */
typedef struct {
    uint16_t voltage_mV;  
    uint16_t soc_percent;   
    int16_t  current_mA;       
    int16_t  temperature_C;   
    bool     is_charging;     
    bool     charge_complete;  
} battery_status_t;

esp_err_t battery_init(void);

/* Early LED initialization for quick button wake response */
void battery_early_led_init(void);

/* Get latest battery status snapshot */
void battery_get_status(battery_status_t *status);
uint8_t battery_get_soc(void);
bool battery_is_charging(void);
uint16_t battery_get_voltage_mv(void);

/* Set charger fast-charge current in mA (best-effort) */
esp_err_t battery_charger_set_current_ma(int ma);

/* BLE connection state values */
#define BLE_STATE_DISCONNECTED  0
#define BLE_STATE_ADVERTISING   1
#define BLE_STATE_CONNECTED     2
#define BLE_STATE_TRANSMITTING  3

/* For LED policy */
void battery_set_ble_state(uint8_t state);

/* Notify that a data packet was transmitted (used for LED timing) */
void battery_notify_data_transmit(void);

/* Get detected BQ21080 7-bit I2C address (0x00 if not found) */
/* In this case, it should be not found */
uint8_t battery_get_bq21080_addr(void);
#endif 
