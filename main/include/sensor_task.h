#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

void sensor_task_create(void);

void sensor_task_set_connected(uint16_t conn_handle, bool connected);

/* Legacy helper used by sensor_task to program BQ21080; provided by i2c_shim.c */
esp_err_t i2c_write_reg(uint8_t reg, const uint8_t *data, size_t len);

#endif
