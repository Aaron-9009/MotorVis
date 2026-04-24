#include <string.h>

#include "motor_vis_ble_uuids.h"

// File owns the shared 128-bit UUID table for the MotoVis BLE service and the helper used to hand those UUIDs to ESP-IDF APIs.
// Service UUID storage lives here so the BLE firmware can use one shared source of truth for the MotoVis service namespace.
const uint8_t MOTOR_VIS_SERVICE_UUID[16] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

// Characteristic UUIDs stay inside the MotoVis service namespace so each sensor stream keeps its own custom identity.
const uint8_t MOTOR_VIS_CHAR_UUID_HEART_RATE[16] = {
    0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

// Battery packets stay inside the same namespace so the app can treat them as part of the MotoVis jacket service.
const uint8_t MOTOR_VIS_CHAR_UUID_BATTERY[16] = {
    0x02, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

// Accelerometer packets use their own UUID within the MotoVis namespace so motion telemetry can be separated by sensor type.
const uint8_t MOTOR_VIS_CHAR_UUID_ACCELEROMETER[16] = {
    0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

// Gyroscope packets use their own UUID within the same namespace so orientation data stays separate from acceleration data.
const uint8_t MOTOR_VIS_CHAR_UUID_GYRO[16] = {
    0x04, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

// GPS packets currently map to the live location characteristic exposed by the BLE firmware.
const uint8_t MOTOR_VIS_CHAR_UUID_GPS[16] = {
    0x05, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
};

void motor_vis_uuid128_to_esp_bt_uuid(const uint8_t source[16], esp_bt_uuid_t *dest)
{
    // Marks the destination as a 128-bit UUID before copying the raw bytes into ESP-IDF's BLE UUID container.
    // Shared helper keeps BLE code from manually copying each 128-bit UUID at every call site.
    dest->len = ESP_UUID_LEN_128;
    memcpy(dest->uuid.uuid128, source, 16);
}
