#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

// Header exposes the small C-facing contract that lets the BLE firmware read battery telemetry from the battery manager.
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared battery sample structure so the BLE firmware can publish both the measured voltage and the app-friendly percentage.
typedef struct {
    // Millisecond timestamp captured when the battery sample was stored.
    uint32_t timestamp_ms;
    // Battery voltage in millivolts after reversing the Feather V2 voltage divider.
    uint16_t voltage_mv;
    // Estimated single-cell LiPo percentage based on the measured battery voltage.
    uint8_t percentage;
    // Raw ADC count from the ESP32 ADC channel, useful when validating the divider during hardware bring-up.
    uint16_t raw_adc;
    // Valid flag used by BLE and the Android app to tell whether the current voltage reading looks usable.
    uint8_t battery_valid;
    // Monotonic sequence counter that advances whenever a new battery sample is stored.
    uint32_t sequence;
} motor_vis_battery_sample_t;

// Initializes the Feather V2 battery ADC reader and starts the background sampling task.
esp_err_t motor_vis_battery_manager_init(void);

// Copies the latest battery sample into the caller-provided struct and returns true when the manager is available.
bool motor_vis_battery_manager_get_latest_sample(motor_vis_battery_sample_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_MANAGER_H
