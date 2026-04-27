#ifndef ACCELEROMETER_MANAGER_H
#define ACCELEROMETER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared accelerometer structure
typedef struct {
    // Millisecond timestamp captured when sample stored
    uint32_t timestamp_ms;
    // X-axis acceleration
    int16_t ax_mg;
    // Y-axis acceleration
    int16_t ay_mg;
    // Z-axis acceleration
    int16_t az_mg;
    // Set to 1 when the magnitude between samples exceeds the crash limit
    uint8_t alert;
    uint32_t sequence;
} motor_vis_accel_sample_t;

esp_err_t motor_vis_accel_manager_init(void);

// Copies the latest sample
bool motor_vis_accel_manager_get_latest_sample(motor_vis_accel_sample_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif 
