#ifndef HEART_SENSOR_MANAGER_H
#define HEART_SENSOR_MANAGER_H

// Header exposes the C-facing contract used by the main BLE firmware to read pulse sensor beats from the Arduino-backed manager.
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
// C linkage keeps the exported heart sensor API callable from Motor_Vis.c without exposing Arduino or C++ types.
extern "C" {
#endif

// Shared heart sample structure so the main GATT server can package and log each detected beat consistently.
typedef struct {
    // Millisecond timestamp captured when the beat sample was stored by the manager.
    uint32_t timestamp_ms;
    // Beats per minute reported by PulseSensor Playground for the latest detected beat.
    uint16_t bpm;
    // Latest raw ADC-style sample value reported by the PulseSensor library for bring-up visibility.
    uint16_t raw_adc;
    // Indicates whether bpm is ready to trust; the first beat is marked invalid because there is not yet a previous beat interval.
    uint8_t bpm_valid;
    // Monotonic sequence value that advances whenever a new beat sample is stored.
    uint32_t sequence;
} motor_vis_heart_sample_t;

// Initializes Arduino support, starts PulseSensor Playground, and launches the background beat polling task.
esp_err_t motor_vis_heart_sensor_init(void);

// Copies the latest beat sample into the caller-provided struct and returns true when the manager is available.
bool motor_vis_heart_sensor_get_latest_sample(motor_vis_heart_sample_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif // HEART_SENSOR_MANAGER_H
