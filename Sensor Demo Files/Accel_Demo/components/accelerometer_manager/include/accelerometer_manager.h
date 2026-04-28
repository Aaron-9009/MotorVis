#ifndef ACCELEROMETER_MANAGER_H
#define ACCELEROMETER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared accelerometer sample used by the demo firmware without exposing ADXL345 I2C details to app_main().
typedef struct {
    // Millisecond timestamp captured when the sample is stored.
    uint32_t timestamp_ms;
    // X-axis acceleration reported in milli-g.
    int16_t ax_mg;
    // Y-axis acceleration reported in milli-g.
    int16_t ay_mg;
    // Z-axis acceleration reported in milli-g.
    int16_t az_mg;
    // Set to 1 when the v1 delta check sees a sudden acceleration change.
    uint8_t alert;
    // Increments every time the manager stores a new sample.
    uint32_t sequence;
} motor_vis_accel_sample_t;

// Initializes the ADXL345 and starts the background task that keeps the latest sample updated.
esp_err_t motor_vis_accel_manager_init(void);

// Copies the most recent accelerometer sample into out_sample if the manager is running.
bool motor_vis_accel_manager_get_latest_sample(motor_vis_accel_sample_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif
