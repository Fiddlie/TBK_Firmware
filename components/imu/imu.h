#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* 6-DoF raw data structure */
typedef struct {
    int16_t ax, ay, az;  
    int16_t gx, gy, gz;  
} imu_raw_t;

/* Public API */
esp_err_t imu_init(void);             
bool imu_read_raw(imu_raw_t *out);      // Read one sample, returns true on success
esp_err_t imu_deinit(void);       