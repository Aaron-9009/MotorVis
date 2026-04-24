#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

// Header exposes the small C-facing contract that lets the C BLE firmware read fixes from the C++ GNSS manager.
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
// C linkage keeps the exported GNSS manager API callable from the main BLE firmware written in C.
extern "C" {
#endif

// Shared GNSS fix structure so the C BLE firmware can pull the latest parsed GPS fix from the C++ GNSS manager.
typedef struct {
    // Millisecond timestamp captured when the stored fix snapshot was last updated.
    uint32_t timestamp_ms;
    // Latitude stored in e7 fixed-point format so it can be published in a compact binary BLE payload.
    int32_t latitude_e7;
    // Longitude stored in e7 fixed-point format so it can be published in a compact binary BLE payload.
    int32_t longitude_e7;
    // Fix-valid flag used by BLE and the Android app to tell whether the current coordinates are usable.
    uint8_t fix_valid;
    // Monotonic sequence counter that advances whenever a new fix snapshot is stored.
    uint32_t sequence;
} motor_vis_gnss_fix_t;

// Initializes the UART/TinyGPS++ GNSS manager and starts its background read task.
esp_err_t motor_vis_gnss_init(void);

// Copies the latest fix into the caller-provided struct and returns true when the manager is available.
bool motor_vis_gnss_get_latest_fix(motor_vis_gnss_fix_t *out_fix);

#ifdef __cplusplus
}
#endif

#endif // GNSS_MANAGER_H
