/* imu.h - Corrected header for LSM6DSOX */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* 6-DoF raw data structure - simplified version */
typedef struct {
    int16_t ax, ay, az;     /* Accelerometer raw values */
    int16_t gx, gy, gz;     /* Gyroscope raw values */
} imu_raw_t;

esp_err_t imu_init(void);               /* Initialize IMU once at startup */
bool imu_read_raw(imu_raw_t *out);      /* Read one sample, returns true on success */