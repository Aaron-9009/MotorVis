#include <cstdint>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
}

#include "gnss_manager.h"

// GNSS_Demo isolates the GPS path from BLE so we can verify the module, UART wiring, parser, and packet format by themselves.
static const char *TAG = "GNSS_DEMO";

// This packet mirrors the production GPS GATT characteristic exactly.
// Android should decode these fields as little-endian values in the same order.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    uint8_t fix_valid;
} motor_vis_gps_payload_t;

// TODO: Testing Code
// This status interval supported the earlier bring-up logs. It is commented out so the terminal only prints decoded coordinates.
// static constexpr uint32_t GNSS_DEMO_STATUS_INTERVAL_MS = 1000;
static constexpr uint32_t GNSS_DEMO_POLL_INTERVAL_MS = 100;
// TODO: Testing Code
// Packet length was printed during payload debugging, but the current test only needs decoded latitude and longitude.
// static constexpr uint32_t GNSS_DEMO_GPS_PAYLOAD_LENGTH = sizeof(motor_vis_gps_payload_t);

// Uses the same packet conversion as the BLE firmware, but prints it to the ESP terminal instead of a characteristic.
static void gnss_demo_copy_payload_from_fix(const motor_vis_gnss_fix_t *fix, motor_vis_gps_payload_t *payload)
{
    if (payload == nullptr) {
        return;
    }

    std::memset(payload, 0, sizeof(*payload));

    if (fix == nullptr) {
        return;
    }

    payload->timestamp_ms = fix->timestamp_ms;
    payload->fix_valid = fix->fix_valid ? 1U : 0U;
    payload->latitude_e7 = payload->fix_valid ? fix->latitude_e7 : 0;
    payload->longitude_e7 = payload->fix_valid ? fix->longitude_e7 : 0;
}

// Decodes the same packed GPS payload that LightBlue or the Android app would receive from the GPS characteristic.
static void gnss_demo_log_gatt_packet(const motor_vis_gps_payload_t *payload, uint32_t sequence)
{
    if (payload == nullptr) {
        return;
    }

    (void) sequence;
    // TODO: Testing Code
    // Raw packet bytes are preserved here from the packet-format test but hidden for the clean coordinate-only terminal pass.
    // const uint8_t *packet_bytes = reinterpret_cast<const uint8_t *>(payload);
    double latitude_degrees = static_cast<double>(payload->latitude_e7) / 10000000.0;
    double longitude_degrees = static_cast<double>(payload->longitude_e7) / 10000000.0;

    if (!payload->fix_valid) {
        return;
    }

    // This is the only custom demo log left enabled so the terminal shows the decoded coordinates from the packed GATT payload.
    ESP_LOGI(TAG, "decoded GPS from GATT packet: latitude=%.7f longitude=%.7f", latitude_degrees, longitude_degrees);

    // TODO: Testing Code
    // ESP_LOGI(
    //     TAG,
    //     "GPS packet fields: seq=%lu timestamp_ms=%lu latitude_e7=%ld longitude_e7=%ld fix_valid=%u latitude_deg=%.7f longitude_deg=%.7f payload_len=%lu",
    //     static_cast<unsigned long>(sequence),
    //     static_cast<unsigned long>(payload->timestamp_ms),
    //     static_cast<long>(payload->latitude_e7),
    //     static_cast<long>(payload->longitude_e7),
    //     static_cast<unsigned int>(payload->fix_valid),
    //     latitude_degrees,
    //     longitude_degrees,
    //     static_cast<unsigned long>(GNSS_DEMO_GPS_PAYLOAD_LENGTH)
    // );

    // TODO: Testing Code
    // ESP_LOGI(
    //     TAG,
    //     "GPS packet bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //     static_cast<unsigned int>(packet_bytes[0]),
    //     static_cast<unsigned int>(packet_bytes[1]),
    //     static_cast<unsigned int>(packet_bytes[2]),
    //     static_cast<unsigned int>(packet_bytes[3]),
    //     static_cast<unsigned int>(packet_bytes[4]),
    //     static_cast<unsigned int>(packet_bytes[5]),
    //     static_cast<unsigned int>(packet_bytes[6]),
    //     static_cast<unsigned int>(packet_bytes[7]),
    //     static_cast<unsigned int>(packet_bytes[8]),
    //     static_cast<unsigned int>(packet_bytes[9]),
    //     static_cast<unsigned int>(packet_bytes[10]),
    //     static_cast<unsigned int>(packet_bytes[11]),
    //     static_cast<unsigned int>(packet_bytes[12])
    // );
}

extern "C" void app_main(void)
{
    // TODO: Testing Code
    // ESP_LOGI(TAG, "starting standalone GNSS packet demo firmware");
    // ESP_LOGI(TAG, "GNSS wiring expected by production manager: UART2 TX=GPIO33 RX=GPIO32 baud=9600");
    // ESP_LOGI(TAG, "GPS GATT payload shape: timestamp_ms uint32, latitude_e7 int32, longitude_e7 int32, fix_valid uint8");

    esp_err_t err = motor_vis_gnss_init();
    if (err != ESP_OK) {
        // TODO: Testing Code
        // ESP_LOGE(TAG, "GNSS manager init failed: %s", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // TODO: Testing Code
    // ESP_LOGI(TAG, "GNSS manager initialized, waiting for UART data and valid coordinate updates");

    uint32_t last_sequence = 0;
    // TODO: Testing Code
    // Status ticks supported the earlier waiting/no-fix terminal updates. The coordinate-only test no longer needs them.
    // uint32_t status_ticks = 0;

    while (true) {
        motor_vis_gnss_fix_t latest_fix = {};

        if (motor_vis_gnss_get_latest_fix(&latest_fix)) {
            if (latest_fix.sequence != last_sequence) {
                motor_vis_gps_payload_t payload = {};
                gnss_demo_copy_payload_from_fix(&latest_fix, &payload);
                gnss_demo_log_gatt_packet(&payload, latest_fix.sequence);
                last_sequence = latest_fix.sequence;
            }
        }

        // TODO: Testing Code
        // else if (status_ticks >= (GNSS_DEMO_STATUS_INTERVAL_MS / GNSS_DEMO_POLL_INTERVAL_MS)) {
        //     ESP_LOGI(
        //         TAG,
        //         "waiting for next GNSS coordinate packet: last_seq=%lu last_fix_valid=%u last_latitude_e7=%ld last_longitude_e7=%ld",
        //         static_cast<unsigned long>(latest_fix.sequence),
        //         static_cast<unsigned int>(latest_fix.fix_valid),
        //         static_cast<long>(latest_fix.latitude_e7),
        //         static_cast<long>(latest_fix.longitude_e7)
        //     );
        //     status_ticks = 0;
        // }
        // else if (status_ticks >= (GNSS_DEMO_STATUS_INTERVAL_MS / GNSS_DEMO_POLL_INTERVAL_MS)) {
        //     ESP_LOGW(TAG, "GNSS manager is not returning a latest fix yet");
        //     status_ticks = 0;
        // }
        // status_ticks += 1;
        vTaskDelay(pdMS_TO_TICKS(GNSS_DEMO_POLL_INTERVAL_MS));
    }
}
