#ifndef MOTOR_VIS_BLE_UUIDS_H
#define MOTOR_VIS_BLE_UUIDS_H

// Header centralizes the custom MotoVis BLE UUID declarations shared by the firmware modules.
#include <stdint.h>
#include "esp_bt_defs.h"

// Shared custom BLE UUID declarations live in one header so the service code can include them without repeating UUID arrays.
// Base service UUID that identifies the MotoVis jacket service namespace.
extern const uint8_t MOTOR_VIS_SERVICE_UUID[16];
// Characteristic UUID reserved for heart-rate packets published by the jacket firmware.
extern const uint8_t MOTOR_VIS_CHAR_UUID_HEART_RATE[16];
// Characteristic UUID reserved for battery or power telemetry published by the jacket firmware.
extern const uint8_t MOTOR_VIS_CHAR_UUID_BATTERY[16];
// Characteristic UUID reserved for accelerometer telemetry published by the jacket firmware.
extern const uint8_t MOTOR_VIS_CHAR_UUID_ACCELEROMETER[16];
// Characteristic UUID reserved for gyroscope or orientation telemetry published by the jacket firmware.
extern const uint8_t MOTOR_VIS_CHAR_UUID_GYRO[16];
// Characteristic UUID used by the live GPS packet characteristic in the current BLE service.
extern const uint8_t MOTOR_VIS_CHAR_UUID_GPS[16];

// Helper converts raw 128-bit UUID byte arrays into the ESP-IDF BLE UUID structure format.
void motor_vis_uuid128_to_esp_bt_uuid(const uint8_t source[16], esp_bt_uuid_t *dest);

#endif // MOTOR_VIS_BLE_UUIDS_H
