// Motor_Vis.c - BLE GATT server for MotoVis Jacket
//
// Implements a simple BLE GATT server that allows a connected phone to read the
// current GPS and heart-rate payloads and subscribe for notifications when new
// GNSS fixes or pulse samples are published. The device advertises itself as "MotoVis Jacket" and uses the onboard
// NeoPixel for status indication.
//
// The NeoPixel shows different colors and blinking patterns based on the device state:
// - Booting: Slow blinking purple
// - BLE Initializing: Slow blinking blue
// - Advertising: Slow blinking cyan
// - Connected: Solid green
// - Error: Fast blinking red
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ---- ESP-IDF headers ----

// Freertos headers
// Provides basic task scheduling, task management, and synchronization primitives for the application.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Bluetooth Protocol Stack headers
// Manages the initialization and release of the bluetooth controller, as well as the Bluedroid stack which provides the Bluetooth protocol implementation.
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

// BLE GAP Interface headers
// Responsible for managing BLE advertising, scanning, and connection parameters, as well as handling GAP events such as advertising start/stop and connection updates.
#include "esp_gap_ble_api.h"

// BLE GATT Server Interface headers
// API for creating GATT services and characteristics, handling read/write requests from clients, and managing GATT server events such as connections and disconnections.
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"

// Non-volatile storage and shared helpers for BLE data.
#include "esp_check.h"
#include "esp_log.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "esp_sleep.h"

// Includes the GNSS manager bridge so BLE can pull the latest parsed GPS fix without including TinyGPS++ directly.
#include "gnss_manager.h"
// Includes the heart sensor manager bridge so BLE can pull PulseSensor samples without including Arduino code directly.
#include "heart_sensor_manager.h"
// Includes the external jacket controls component so button and LED validation stays modular and separate from BLE state logic.
#include "jacket_controls.h"
// Includes the shared 128-bit UUID definitions used by the custom MotoVis BLE service and GPS characteristic.
#include "motor_vis_ble_uuids.h"

// Arduino's ESP32 core provides btInUse as a weak hook that decides whether initArduino()
// should release Bluetooth controller memory. The heart sensor manager calls initArduino()
// before the native MotoVis BLE stack is brought up, so overriding this hook keeps Arduino
// from releasing BTDM memory out from under the firmware's own BLE initialization sequence.
bool btInUse(void)
{
    return true;
}

// --------------------------------------- APPLICATION CONSTANTS ---------------------------------------
// App specific definitions
#define MOTOR_VIS_APP_ID                           0x55 // Internal GATT Server ID

// Maps the active characteristic alias to the GPS characteristic so the GATT event code can target one packet source.
#define MOTOR_VIS_CHARACTERISTIC_UUID             MOTOR_VIS_CHAR_UUID_GPS
// Reserves handles for the service plus the GPS and heart-rate characteristic/value/user-description/CCCD groups.
#define MOTOR_VIS_NUM_HANDLES                      9
#define MOTOR_VIS_DEVICE_NAME                      "MotoVis Jacket"
// TODO: Testing Code
// #define MOTOR_VIS_MAX_VALUE_LENGTH             64
// Previous string payload buffer size used by the BLE test characteristic before GPS packet integration.
#define MOTOR_VIS_STATUS_LED_GPIO                  GPIO_NUM_0
#define MOTOR_VIS_LED_POWER_GPIO                   GPIO_NUM_2
#define MOTOR_VIS_NEOPIXEL_COUNT                   1
#define MOTOR_VIS_CLIENT_CONFIG_LENGTH             2
#define MOTOR_VIS_CLIENT_CONFIG_DESCRIPTOR_UUID    0x2902
#define MOTOR_VIS_USER_DESCRIPTION_DESCRIPTOR_UUID 0x2901
#define MOTOR_VIS_HEART_RATE_MIN_SAFE_BPM          50
#define MOTOR_VIS_HEART_RATE_MAX_SAFE_BPM          120

// Tag for logging in terminal and error reporting.
static const char *TAG = "MOTOR_VIS_BLE";
// Separate heart-data tag keeps pulse logs easy to spot during sensor bring-up.
static const char *TAG_HEART = "MOTOR_VIS_HEART";
// TODO: Testing Code
// static const uint8_t MOTOR_VIS_SERVICE_UUID[16] = {
//     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
//     0x00, 0x00, 0x00, 0xaf, 0xaf, 0xaf, 0xaf, 0xaf
// };
// Previous in-file custom service UUID definition before the shared UUID source was moved to motor_vis_ble_uuids.c.

// LED state machine for status indication.
typedef enum {
    MOTOR_VIS_LED_STATE_BOOTING = 0,
    MOTOR_VIS_LED_STATE_BLE_INITIALIZING,
    MOTOR_VIS_LED_STATE_ADVERTISING,
    MOTOR_VIS_LED_STATE_CONNECTED,
    MOTOR_VIS_LED_STATE_ERROR,
} motor_vis_led_state_t;

// Tracks the asynchronous GATT database build so GPS and heart characteristics are added in a known order.
typedef enum {
    MOTOR_VIS_GATT_BUILD_IDLE = 0,
    MOTOR_VIS_GATT_BUILD_GPS_CHARACTERISTIC,
    MOTOR_VIS_GATT_BUILD_GPS_USER_DESCRIPTION,
    MOTOR_VIS_GATT_BUILD_GPS_CCCD,
    MOTOR_VIS_GATT_BUILD_HEART_CHARACTERISTIC,
    MOTOR_VIS_GATT_BUILD_HEART_USER_DESCRIPTION,
    MOTOR_VIS_GATT_BUILD_HEART_CCCD,
    MOTOR_VIS_GATT_BUILD_COMPLETE,
} motor_vis_gatt_build_state_t;

// --------------------------------------- PAYLOAD STRUCTURE DEFINITIONS ---------------------------------------
// Fixed-format BLE GPS payload, it mirrors the custom packet fields for the GPS characteristic.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    uint8_t fix_valid;
} motor_vis_gps_payload_t;

// Fixed-format BLE heart-rate payload, it mirrors the custom packet fields for the heart-rate characteristic.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t bpm;
    uint16_t raw_adc;
    uint8_t bpm_valid;
} motor_vis_heart_rate_payload_t;

typedef struct __attribute__((packed)) {} motor_vis_placeholder_payload_t;

//-----------------------------------------------------------------------------------------------------------


// --------------------------------------- PAYLOAD LENGTH DEFINITIONS ---------------------------------------
//Used to define the length of the data being sent from MC to App.
#define MOTOR_VIS_GPS_PAYLOAD_LENGTH ((uint16_t) sizeof(motor_vis_gps_payload_t))
#define MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH ((uint16_t) sizeof(motor_vis_heart_rate_payload_t))
#define MOTOR_VIS_PLACEHOLDER_PAYLOAD_LENGTH ((uint16_t) sizeof(motor_vis_placeholder_payload_t));
//-----------------------------------------------------------------------------------------------------------


// --------------------------------------- BLE RUNTIME STATE ---------------------------------------
// Tracks service handles, connection status, advertising readiness, and publish progress for the live BLE session.
static uint16_t motor_vis_service_handle = 0;
// TODO: Testing Code
// static uint16_t motor_vis_characteristic_handle = 0;
// Previous handle tracked the generic read/write test characteristic.
static uint16_t motor_vis_gps_characteristic_handle = 0;
// Tracks the standard user-description descriptor handle so generic BLE browser apps can show a friendly GPS label.
static uint16_t motor_vis_gps_user_description_handle = 0;
// Tracks the CCCD handle so the firmware abides by notification subscriptions for the GPS characteristic.
static uint16_t motor_vis_gps_cccd_handle = 0;
// Tracks the heart-rate characteristic and CCCD handles independently from GPS so clients can subscribe to either packet stream.
static uint16_t motor_vis_heart_characteristic_handle = 0;
// Tracks the standard user-description descriptor handle so generic BLE browser apps can show a friendly heart label.
static uint16_t motor_vis_heart_user_description_handle = 0;
static uint16_t motor_vis_heart_cccd_handle = 0;
static esp_gatt_if_t motor_vis_gatts_if = ESP_GATT_IF_NONE;
static uint16_t motor_vis_connection_id = 0;
static bool motor_vis_connected = false;
static bool motor_vis_ble_ready = false;
static bool motor_vis_adv_data_configured = false;
static bool motor_vis_scan_rsp_configured = false;
static bool motor_vis_advertising = false;
// Tracks whether the current BLE client enabled notifications on the GPS characteristic.
static bool motor_vis_gps_notifications_enabled = false;
// Tracks whether the current BLE client enabled notifications on the heart-rate characteristic.
static bool motor_vis_heart_notifications_enabled = false;
// Tracks whether the jacket controls component initialized successfully so shared BLE callbacks can safely drive the external LEDs.
static bool motor_vis_jacket_controls_ready = false;
// Remembers the most recent GNSS sequence that has already been published into the BLE attribute database.
static uint32_t motor_vis_last_published_gnss_sequence = 0;
// Remembers the most recent heart sensor sequence that has already been published into the BLE attribute database.
static uint32_t motor_vis_last_published_heart_sequence = 0;
// Tracks which GATT attribute is currently being added while ESP-IDF reports asynchronous add events.
static motor_vis_gatt_build_state_t motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_IDLE;
// Tracks whether a crash/alert event is still inside the cancellable window before the companion app alert is sent.
static bool motor_vis_alert_cancel_window_active = false;
// Prevents normal BLE reconnect behavior from fighting the temporary light-sleep transition while the firmware is intentionally powering down activity.
static bool motor_vis_light_sleep_transition_active = false;
static volatile motor_vis_led_state_t motor_vis_led_state = MOTOR_VIS_LED_STATE_BOOTING;
static led_strip_handle_t motor_vis_status_pixel = NULL;

// TODO: Testing Code
// static uint8_t motor_vis_characteristic_value[MOTOR_VIS_MAX_VALUE_LENGTH] = "MotoVis Ready";
// static uint16_t motor_vis_characteristic_length = 13;
// static esp_attr_value_t motor_vis_attribute_value = {
//     .attr_max_len = MOTOR_VIS_MAX_VALUE_LENGTH,
//     .attr_len = 13,
//     .attr_value = motor_vis_characteristic_value,
// };
// Previous string-based test characteristic storage before the BLE service switched to the dedicated GPS payload.

// Initializes the GPS characteristic with an invalid fix so BLE remains readable even before GNSS data is established.
static uint8_t motor_vis_gps_characteristic_value[MOTOR_VIS_GPS_PAYLOAD_LENGTH] = {0};
// Stores the human-readable GPS descriptor string that LightBlue can display beside the packet characteristic.
static uint8_t motor_vis_gps_user_description_value[] = "MotoVis GPS Packet";
// Stores the CCCD value so the firmware can remember whether notifications are enabled for the active client.
static uint8_t motor_vis_gps_cccd_value[MOTOR_VIS_CLIENT_CONFIG_LENGTH] = {0};
// Initializes the heart-rate characteristic with an invalid sample so BLE remains readable before a pulse is detected.
static uint8_t motor_vis_heart_characteristic_value[MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH] = {0};
// Stores the human-readable heart descriptor string that LightBlue can display beside the packet characteristic.
static uint8_t motor_vis_heart_user_description_value[] = "MotoVis Heart Packet";
// Stores the heart CCCD value separately so GPS and heart notifications can be enabled independently.
static uint8_t motor_vis_heart_cccd_value[MOTOR_VIS_CLIENT_CONFIG_LENGTH] = {0};

// Attribute metadata for the GPS characteristic's fixed-size packet payload.
static esp_attr_value_t motor_vis_gps_attribute_value = {
    .attr_max_len = MOTOR_VIS_GPS_PAYLOAD_LENGTH,
    .attr_len = MOTOR_VIS_GPS_PAYLOAD_LENGTH,
    .attr_value = motor_vis_gps_characteristic_value,
};

// Attribute metadata for the standard Characteristic User Description descriptor that provides a friendly GPS label.
static esp_attr_value_t motor_vis_gps_user_description_attribute_value = {
    .attr_max_len = sizeof(motor_vis_gps_user_description_value) - 1,
    .attr_len = sizeof(motor_vis_gps_user_description_value) - 1,
    .attr_value = motor_vis_gps_user_description_value,
};

// Attribute metadata for the standard Client Characteristic Configuration Descriptor (CCCD).
static esp_attr_value_t motor_vis_gps_cccd_attribute_value = {
    .attr_max_len = MOTOR_VIS_CLIENT_CONFIG_LENGTH,
    .attr_len = MOTOR_VIS_CLIENT_CONFIG_LENGTH,
    .attr_value = motor_vis_gps_cccd_value,
};

// Attribute metadata for the heart-rate characteristic's fixed-size packet payload.
static esp_attr_value_t motor_vis_heart_attribute_value = {
    .attr_max_len = MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH,
    .attr_len = MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH,
    .attr_value = motor_vis_heart_characteristic_value,
};

// Attribute metadata for the standard Characteristic User Description descriptor that provides a friendly heart label.
static esp_attr_value_t motor_vis_heart_user_description_attribute_value = {
    .attr_max_len = sizeof(motor_vis_heart_user_description_value) - 1,
    .attr_len = sizeof(motor_vis_heart_user_description_value) - 1,
    .attr_value = motor_vis_heart_user_description_value,
};

// Attribute metadata for the heart-rate Client Characteristic Configuration Descriptor (CCCD).
static esp_attr_value_t motor_vis_heart_cccd_attribute_value = {
    .attr_max_len = MOTOR_VIS_CLIENT_CONFIG_LENGTH,
    .attr_len = MOTOR_VIS_CLIENT_CONFIG_LENGTH,
    .attr_value = motor_vis_heart_cccd_value,
};

// --------------------------------------- BLE ADVERTISING CONFIGURATION ---------------------------------------
// Advertising data configuration.
static esp_ble_adv_data_t advertising_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x0000,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    // .service_uuid_len = sizeof(MOTOR_VIS_SERVICE_UUID),
    .service_uuid_len = 0,
    // .p_service_uuid = (uint8_t *)MOTOR_VIS_SERVICE_UUID,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Scan response data configuration. The primary advertisement carries the
//  device name while the scan response carries the 128-bit
// service UUID for clients that inspect the full payload.
static esp_ble_adv_data_t scan_response_data = {
    .set_scan_rsp = true,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    // .include_name = true,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x0000,
    .service_data_len = 0,
    .p_service_data = NULL,
    // .service_uuid_len = 0,
    .service_uuid_len = sizeof(MOTOR_VIS_SERVICE_UUID),
    // .p_service_uuid = NULL,
    .p_service_uuid = (uint8_t *) MOTOR_VIS_SERVICE_UUID,
    .flag = 0,
};

// Advertising parameters configuration.
static esp_ble_adv_params_t advertising_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// --------------------------------------- FORWARD DECLARATIONS ---------------------------------------
// Lists the main helper entry points used by the BLE event handlers and startup path.
static void start_advertising(const char *reason);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static esp_err_t motor_vis_status_led_init(void);
static void motor_vis_set_led_state(motor_vis_led_state_t state);
static void motor_vis_status_led_task(void *parameter);
static void motor_vis_report_error(const char *step, esp_err_t err);
static void motor_vis_status_led_apply_rgb(uint8_t red, uint8_t green, uint8_t blue);
static bool motor_vis_is_advertising_ready(void);
static void motor_vis_sync_pairing_led_with_ble_state(void);
static const char *motor_vis_wakeup_cause_to_string(esp_sleep_wakeup_cause_t wakeup_cause);
static esp_err_t motor_vis_enter_temporary_light_sleep(void);

// Resets the GPS CCCD state whenever a new connection lifecycle begins.
static void motor_vis_reset_gps_cccd_state(void);
// Resets the heart CCCD state whenever a new connection lifecycle begins.
static void motor_vis_reset_heart_cccd_state(void);
// Resets every notification subscription flag so reconnects always start from a known state.
static void motor_vis_reset_all_cccd_state(void);
// Builds a known invalid GPS payload so BLE remains readable before GNSS has a valid fix.
static void motor_vis_copy_invalid_gps_payload(motor_vis_gps_payload_t *payload);
// Builds a known invalid heart payload so BLE remains readable before the first pulse sample arrives.
static void motor_vis_copy_invalid_heart_payload(motor_vis_heart_rate_payload_t *payload);
// Writes the GPS payload into the BLE attribute database and optionally notifies subscribed clients.
static esp_err_t motor_vis_publish_gps_payload(const motor_vis_gps_payload_t *payload, bool notify_client);
// Writes the heart-rate payload into the BLE attribute database and optionally notifies subscribed clients.
static esp_err_t motor_vis_publish_heart_payload(const motor_vis_heart_rate_payload_t *payload, bool notify_client);
// Logs the exact GPS characteristic packet layout so firmware bring-up matches what the Android app will decode.
static void motor_vis_log_gps_characteristic_packet(const motor_vis_gps_payload_t *payload, bool notify_client);
// Logs the exact heart characteristic packet layout so firmware bring-up matches what the Android app will decode.
static void motor_vis_log_heart_characteristic_packet(const motor_vis_heart_rate_payload_t *payload, bool notify_client);
// Converts the shared GNSS manager struct into the BLE GPS packet format.
static esp_err_t motor_vis_publish_gnss_fix(const motor_vis_gnss_fix_t *fix, bool notify_client);
// Converts the shared heart manager struct into the BLE heart-rate packet format.
static esp_err_t motor_vis_publish_heart_sample(const motor_vis_heart_sample_t *sample, bool notify_client);
// Adds the next GATT characteristic or descriptor while the custom service is being constructed.
static void motor_vis_add_gps_characteristic(void);
static void motor_vis_add_gps_user_description(void);
static void motor_vis_add_gps_cccd(void);
static void motor_vis_add_heart_characteristic(void);
static void motor_vis_add_heart_user_description(void);
static void motor_vis_add_heart_cccd(void);

//-------------------------------------------------------------------------------------------------------------------------

// Utility function to log errors and set LED to error state.
static void motor_vis_report_error(const char *step, esp_err_t err)
{
    // Logs error.
    ESP_LOGE(TAG, "%s failed: %s", step, esp_err_to_name(err));
    // Sets LED to error state.
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ERROR);
}

// Centralizes the async advertising prerequisites so the GAP callback path and
// the supervisor loop use the same readiness test.
static bool motor_vis_is_advertising_ready(void)
{
    return motor_vis_ble_ready &&
        motor_vis_adv_data_configured &&
        motor_vis_scan_rsp_configured;
}

// Keeps the external pairing LED aligned with the real BLE discoverability state instead of a separate local toggle.
static void motor_vis_sync_pairing_led_with_ble_state(void)
{
    if (!motor_vis_jacket_controls_ready) {
        return;
    }

    // The pairing LED should blink whenever the jacket is advertising for a BLE connection and turn off whenever advertising is inactive.
    esp_err_t err = jacket_controls_set_led_mode(
        JACKET_CONTROL_LED_PAIRING,
        motor_vis_advertising ? JACKET_CONTROL_LED_BLINK_SLOW : JACKET_CONTROL_LED_OFF
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to synchronize pairing LED with BLE state: %s", esp_err_to_name(err));
    }
}

// Converts ESP-IDF wakeup causes into readable log text so temporary sleep bring-up is easier to follow in the terminal.
static const char *motor_vis_wakeup_cause_to_string(esp_sleep_wakeup_cause_t wakeup_cause)
{
    switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        return "undefined";

    case ESP_SLEEP_WAKEUP_GPIO:
        return "gpio";

    case ESP_SLEEP_WAKEUP_TIMER:
        return "timer";

    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        return "touchpad";

    case ESP_SLEEP_WAKEUP_ULP:
        return "ulp";

    case ESP_SLEEP_WAKEUP_UART:
        return "uart";

    case ESP_SLEEP_WAKEUP_EXT0:
        return "ext0";

    case ESP_SLEEP_WAKEUP_EXT1:
        return "ext1";

    default:
        return "other";
    }
}

// TEMPORARY POWER NOTE:
// This helper does NOT fully power the Feather off. A real on/off button still requires an external
// hardware power path that can actually cut battery power to the MCU. Until that hardware exists,
// the power button enters Light-sleep so BLE and the main application work pause, then the same button
// wakes the MCU back up. This is a temporary debugging and bring-up measure only.
static esp_err_t motor_vis_enter_temporary_light_sleep(void)
{
    esp_err_t err = ESP_OK;

    motor_vis_light_sleep_transition_active = true;

    ESP_LOGW(
        TAG,
        "TEMPORARY POWER MODE: full MCU power-off is not implemented yet; entering Light-sleep instead"
    );

    if (motor_vis_connected &&
        motor_vis_gatts_if != ESP_GATT_IF_NONE) {
        // Requests a clean BLE disconnect so LightBlue and the Android side see the temporary power transition.
        err = esp_ble_gatts_close(motor_vis_gatts_if, motor_vis_connection_id);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to request BLE disconnect before Light-sleep: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "requested BLE disconnect before Light-sleep");
        }
    }

    if (motor_vis_advertising) {
        err = esp_ble_gap_stop_advertising();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to stop advertising before Light-sleep: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "requested advertising stop before Light-sleep");
        }
    }

    // Rebuilds local BLE state immediately so the firmware comes back up as an idle disconnected peripheral after wake.
    motor_vis_connected = false;
    motor_vis_connection_id = 0;
    motor_vis_advertising = false;
    motor_vis_sync_pairing_led_with_ble_state();
    motor_vis_reset_all_cccd_state();

    // Turns visible indicators off so the board clearly looks dormant during the temporary sleep period.
    motor_vis_status_led_apply_rgb(0, 0, 0);
    err = jacket_controls_set_all_leds_off();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to turn jacket LEDs off before Light-sleep: %s", esp_err_to_name(err));
    }

    // Waits for the button to be released so the active-low wake source does not fire immediately on sleep entry.
    ESP_LOGI(TAG, "waiting for power button release before entering Light-sleep");
    while (jacket_controls_is_button_pressed(JACKET_CONTROL_BUTTON_POWER)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    jacket_controls_sync_button_state(JACKET_CONTROL_BUTTON_POWER);

    // Clears any previous wake configuration before the temporary power-button wake source is armed.
    err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to clear previous wakeup sources before Light-sleep: %s", esp_err_to_name(err));
    }

    err = gpio_wakeup_enable(GPIO_NUM_5, GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) {
        motor_vis_light_sleep_transition_active = false;
        return err;
    }

    err = esp_sleep_enable_gpio_wakeup();
    if (err != ESP_OK) {
        gpio_wakeup_disable(GPIO_NUM_5);
        motor_vis_light_sleep_transition_active = false;
        return err;
    }

    ESP_LOGI(TAG, "entering temporary Light-sleep now; press the power button again to wake the MCU");
    vTaskDelay(pdMS_TO_TICKS(50));

    err = esp_light_sleep_start();
    if (err != ESP_OK) {
        gpio_wakeup_disable(GPIO_NUM_5);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
        motor_vis_light_sleep_transition_active = false;
        return err;
    }

    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(
        TAG,
        "MCU woke from temporary Light-sleep, wakeup_cause=%s (%d)",
        motor_vis_wakeup_cause_to_string(wakeup_cause),
        (int) wakeup_cause
    );

    // Waits for release after wake and re-synchronizes the debouncer so the wake press is not replayed as a new sleep request.
    while (jacket_controls_is_button_pressed(JACKET_CONTROL_BUTTON_POWER)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    jacket_controls_sync_button_state(JACKET_CONTROL_BUTTON_POWER);

    gpio_wakeup_disable(GPIO_NUM_5);
    err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to disable GPIO wakeup after Light-sleep: %s", esp_err_to_name(err));
    }

    motor_vis_light_sleep_transition_active = false;

    // Restores the normal idle state so BLE advertising can come back to life after the temporary sleep period.
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ADVERTISING);
    if (motor_vis_is_advertising_ready() &&
        !motor_vis_connected &&
        !motor_vis_advertising) {
        start_advertising("wake from temporary light sleep");
    }

    return ESP_OK;
}

// Rebuilds the per-connection CCCD state so notifications default to disabled on every new connection.
static void motor_vis_reset_gps_cccd_state(void)
{
    memset(motor_vis_gps_cccd_value, 0, sizeof(motor_vis_gps_cccd_value));
    motor_vis_gps_notifications_enabled = false;

    if (motor_vis_gps_cccd_handle != 0) {
        esp_err_t err = esp_ble_gatts_set_attr_value(
            motor_vis_gps_cccd_handle,
            sizeof(motor_vis_gps_cccd_value),
            motor_vis_gps_cccd_value
        );
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to reset GPS CCCD state: %s", esp_err_to_name(err));
        }
    }
}

// Rebuilds the heart-rate CCCD state so heart notifications default to disabled on every new connection.
static void motor_vis_reset_heart_cccd_state(void)
{
    memset(motor_vis_heart_cccd_value, 0, sizeof(motor_vis_heart_cccd_value));
    motor_vis_heart_notifications_enabled = false;

    if (motor_vis_heart_cccd_handle != 0) {
        esp_err_t err = esp_ble_gatts_set_attr_value(
            motor_vis_heart_cccd_handle,
            sizeof(motor_vis_heart_cccd_value),
            motor_vis_heart_cccd_value
        );
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to reset heart CCCD state: %s", esp_err_to_name(err));
        }
    }
}

// Resets all CCCD state together so the connection lifecycle cannot accidentally leave one stream subscribed.
static void motor_vis_reset_all_cccd_state(void)
{
    motor_vis_reset_gps_cccd_state();
    motor_vis_reset_heart_cccd_state();
}

// Creates a consistent invalid GPS payload that the Android app can treat as "no fix available yet".
static void motor_vis_copy_invalid_gps_payload(motor_vis_gps_payload_t *payload)
{
    memset(payload, 0, sizeof(*payload));
    payload->fix_valid = 0;
}

// Creates a consistent invalid heart payload that the Android app can treat as "no BPM available yet".
static void motor_vis_copy_invalid_heart_payload(motor_vis_heart_rate_payload_t *payload)
{
    memset(payload, 0, sizeof(*payload));
    payload->bpm_valid = 0;
}

// Logs the same packed field order that is stored in the GPS characteristic value.
static void motor_vis_log_gps_characteristic_packet(const motor_vis_gps_payload_t *payload, bool notify_client)
{
    const uint8_t *packet_bytes = (const uint8_t *) payload;

    ESP_LOGI(
        TAG,
        "GPS GATT packet fields: timestamp_ms=%lu latitude_e7=%ld longitude_e7=%ld fix_valid=%u payload_len=%u notify_requested=%u notifications_enabled=%u",
        (unsigned long) payload->timestamp_ms,
        (long) payload->latitude_e7,
        (long) payload->longitude_e7,
        (unsigned int) payload->fix_valid,
        (unsigned int) MOTOR_VIS_GPS_PAYLOAD_LENGTH,
        notify_client ? 1U : 0U,
        motor_vis_gps_notifications_enabled ? 1U : 0U
    );

    // Shows the raw little-endian byte layout that LightBlue/Android will receive from the characteristic value.
    ESP_LOGI(
        TAG,
        "GPS GATT packet bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        (unsigned int) packet_bytes[0],
        (unsigned int) packet_bytes[1],
        (unsigned int) packet_bytes[2],
        (unsigned int) packet_bytes[3],
        (unsigned int) packet_bytes[4],
        (unsigned int) packet_bytes[5],
        (unsigned int) packet_bytes[6],
        (unsigned int) packet_bytes[7],
        (unsigned int) packet_bytes[8],
        (unsigned int) packet_bytes[9],
        (unsigned int) packet_bytes[10],
        (unsigned int) packet_bytes[11],
        (unsigned int) packet_bytes[12]
    );
}

// Logs the same packed field order that is stored in the heart-rate characteristic value.
static void motor_vis_log_heart_characteristic_packet(const motor_vis_heart_rate_payload_t *payload, bool notify_client)
{
    const uint8_t *packet_bytes = (const uint8_t *) payload;

    //------------------------------------Pulse Sensor Alert Logic------------------------------------
    ESP_LOGI(
        TAG_HEART,
        "HEART GATT packet fields: timestamp_ms=%lu bpm=%u raw_adc=%u bpm_valid=%u payload_len=%u notify_requested=%u notifications_enabled=%u",
        (unsigned long) payload->timestamp_ms,
        (unsigned int) payload->bpm,
        (unsigned int) payload->raw_adc,
        (unsigned int) payload->bpm_valid,
        (unsigned int) MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH,
        notify_client ? 1U : 0U,
        motor_vis_heart_notifications_enabled ? 1U : 0U
    );

    // Shows the raw little-endian byte layout that LightBlue/Android will receive from the heart characteristic value.
    ESP_LOGI(
        TAG_HEART,
        "HEART GATT packet bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        (unsigned int) packet_bytes[0],
        (unsigned int) packet_bytes[1],
        (unsigned int) packet_bytes[2],
        (unsigned int) packet_bytes[3],
        (unsigned int) packet_bytes[4],
        (unsigned int) packet_bytes[5],
        (unsigned int) packet_bytes[6],
        (unsigned int) packet_bytes[7],
        (unsigned int) packet_bytes[8]
    );

    if (payload->bpm_valid &&
        (payload->bpm < MOTOR_VIS_HEART_RATE_MIN_SAFE_BPM ||
         payload->bpm > MOTOR_VIS_HEART_RATE_MAX_SAFE_BPM)) {
        // Heart-rate alerts are log-only in this phase; future alert logic will decide whether this opens a cancel window.
        ESP_LOGW(
            TAG_HEART,
            "heart rate outside safe range: bpm=%u safe_min=%u safe_max=%u",
            (unsigned int) payload->bpm,
            (unsigned int) MOTOR_VIS_HEART_RATE_MIN_SAFE_BPM,
            (unsigned int) MOTOR_VIS_HEART_RATE_MAX_SAFE_BPM
        );
    }
}

// Logs each newly captured heart sample in plain English so teammates can watch live pulse updates in the terminal
// without needing to decode the BLE packet fields by hand first.
static void motor_vis_log_live_heart_sample(const motor_vis_heart_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    ESP_LOGI(
        TAG_HEART,
        "live heart sample: seq=%lu timestamp_ms=%lu bpm=%u raw_adc=%u bpm_valid=%u",
        (unsigned long) sample->sequence,
        (unsigned long) sample->timestamp_ms,
        (unsigned int) sample->bpm,
        (unsigned int) sample->raw_adc,
        (unsigned int) sample->bpm_valid
    );
}

// Centralizes attribute updates and notification sending for the GPS characteristic.
static esp_err_t motor_vis_publish_gps_payload(const motor_vis_gps_payload_t *payload, bool notify_client)
{
    memcpy(motor_vis_gps_characteristic_value, payload, sizeof(*payload));

    if (motor_vis_gps_characteristic_handle == 0) {
        // Allows the buffer to be primed before the BLE attribute handle exists.
        return ESP_OK;
    }

    esp_err_t err = esp_ble_gatts_set_attr_value(
        motor_vis_gps_characteristic_handle,
        MOTOR_VIS_GPS_PAYLOAD_LENGTH,
        motor_vis_gps_characteristic_value
    );
    if (err != ESP_OK) {
        return err;
    }

    motor_vis_log_gps_characteristic_packet(payload, notify_client);

    if (notify_client &&
        motor_vis_connected &&
        motor_vis_gps_notifications_enabled &&
        motor_vis_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(
            motor_vis_gatts_if,
            motor_vis_connection_id,
            motor_vis_gps_characteristic_handle,
            MOTOR_VIS_GPS_PAYLOAD_LENGTH,
            motor_vis_gps_characteristic_value,
            false
        );
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

// Centralizes attribute updates and notification sending for the heart-rate characteristic.
static esp_err_t motor_vis_publish_heart_payload(const motor_vis_heart_rate_payload_t *payload, bool notify_client)
{
    memcpy(motor_vis_heart_characteristic_value, payload, sizeof(*payload));

    if (motor_vis_heart_characteristic_handle == 0) {
        // Allows the buffer to be primed before the BLE attribute handle exists.
        return ESP_OK;
    }

    esp_err_t err = esp_ble_gatts_set_attr_value(
        motor_vis_heart_characteristic_handle,
        MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH,
        motor_vis_heart_characteristic_value
    );
    if (err != ESP_OK) {
        return err;
    }

    motor_vis_log_heart_characteristic_packet(payload, notify_client);

    if (notify_client &&
        motor_vis_connected &&
        motor_vis_heart_notifications_enabled &&
        motor_vis_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(
            motor_vis_gatts_if,
            motor_vis_connection_id,
            motor_vis_heart_characteristic_handle,
            MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH,
            motor_vis_heart_characteristic_value,
            false
        );
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

// Translates the GNSS manager fix into the custom GPS BLE payload without exposing TinyGPS++ to C code.
static esp_err_t motor_vis_publish_gnss_fix(const motor_vis_gnss_fix_t *fix, bool notify_client)
{
    motor_vis_gps_payload_t payload;

    if (fix == NULL) {
        motor_vis_copy_invalid_gps_payload(&payload);
    } else {
        payload.timestamp_ms = fix->timestamp_ms;
        payload.fix_valid = fix->fix_valid ? 1U : 0U;
        payload.latitude_e7 = payload.fix_valid ? fix->latitude_e7 : 0;
        payload.longitude_e7 = payload.fix_valid ? fix->longitude_e7 : 0;
    }

    return motor_vis_publish_gps_payload(&payload, notify_client);
}

// Translates the heart manager sample into the custom heart-rate BLE payload without exposing Arduino to C code.
static esp_err_t motor_vis_publish_heart_sample(const motor_vis_heart_sample_t *sample, bool notify_client)
{
    motor_vis_heart_rate_payload_t payload;

    if (sample == NULL) {
        motor_vis_copy_invalid_heart_payload(&payload);
    } else {
        payload.timestamp_ms = sample->timestamp_ms;
        payload.bpm_valid = sample->bpm_valid ? 1U : 0U;
        payload.bpm = payload.bpm_valid ? sample->bpm : 0;
        payload.raw_adc = sample->raw_adc;
    }

    return motor_vis_publish_heart_payload(&payload, notify_client);
}

// Function to apply RGB color to the status LED.
static void motor_vis_status_led_apply_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // If the LED strip device handle is not initialized, skip applying the color to avoid errors.
    if (motor_vis_status_pixel == NULL) {
        return;
    }

    led_strip_set_pixel(motor_vis_status_pixel, 0, red, green, blue);
    led_strip_refresh(motor_vis_status_pixel);
}

// Function to initialize the status LED (NeoPixel) and its power control.
static esp_err_t motor_vis_status_led_init(void)
{
    // Configure the GPIO for NeoPixel power control.
    gpio_config_t power_config = {
        .pin_bit_mask = 1ULL << MOTOR_VIS_LED_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Error handling.
    ESP_RETURN_ON_ERROR(gpio_config(&power_config), TAG, "failed to configure NeoPixel power pin");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOTOR_VIS_LED_POWER_GPIO, 1), TAG, "failed to enable NeoPixel power");

    // Configure the LED strip (NeoPixel) parameters.
    led_strip_config_t strip_config = {
        .strip_gpio_num = MOTOR_VIS_STATUS_LED_GPIO,
        .max_leds = MOTOR_VIS_NEOPIXEL_COUNT,
    };

    // RMT configuration for the LED strip driver.
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    // Create a new LED strip device using the RMT driver and the specified configuration.
    ESP_RETURN_ON_ERROR(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &motor_vis_status_pixel),
        TAG,
        "failed to create NeoPixel device"
    );
    ESP_RETURN_ON_ERROR(led_strip_clear(motor_vis_status_pixel), TAG, "failed to clear NeoPixel");

    return ESP_OK;
}
// Function to update the LED state machine.
static void motor_vis_set_led_state(motor_vis_led_state_t state)
{
    motor_vis_led_state = state;
}

// FreeRTOS task to manage the status LED based on the current state.
static void motor_vis_status_led_task(void *parameter)
{
    bool led_on = false;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Default delay between LED updates, can be overridden by specific states.
        TickType_t delay_ticks = pdMS_TO_TICKS(250);
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;

        // Switch statement to determine LED behavior based on the current state.
        switch (motor_vis_led_state) {
        case MOTOR_VIS_LED_STATE_BOOTING:
            led_on = !led_on;
            delay_ticks = pdMS_TO_TICKS(120);
            red = 32;
            green = 12;
            break;

        case MOTOR_VIS_LED_STATE_BLE_INITIALIZING:
            led_on = !led_on;
            delay_ticks = pdMS_TO_TICKS(350);
            blue = 32;
            break;

        case MOTOR_VIS_LED_STATE_ADVERTISING:
            led_on = !led_on;
            delay_ticks = pdMS_TO_TICKS(800);
            green = 16;
            blue = 16;
            break;

        case MOTOR_VIS_LED_STATE_CONNECTED:
            led_on = true;
            delay_ticks = pdMS_TO_TICKS(1000);
            green = 32;
            break;

        case MOTOR_VIS_LED_STATE_ERROR:
            led_on = !led_on;
            delay_ticks = pdMS_TO_TICKS(80);
            red = 32;
            break;

        default:
            led_on = false;
            delay_ticks = pdMS_TO_TICKS(500);
            break;
        }

        // Apply the determined RGB color to the LED based on the current state.
        if (led_on) {
            motor_vis_status_led_apply_rgb(red, green, blue);
        } else {
            motor_vis_status_led_apply_rgb(0, 0, 0);
        }

        // Delay until the next update based on the determined delay for the current state.
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

// Function to start BLE advertising if the BLE setup is complete, otherwise logs a warning.
static void start_advertising(const char *reason)
{
    // Check if BLE is ready and advertising data is configured before starting advertising.
    // if (!motor_vis_ble_ready || !motor_vis_adv_data_configured || !motor_vis_scan_rsp_configured) {
    if (!motor_vis_is_advertising_ready()) {
        ESP_LOGW(TAG, "advertising start skipped (%s), BLE setup is not complete yet", reason);
        return;
    }

    ESP_LOGI(TAG, "starting advertising (%s)", reason);
    esp_err_t err = esp_ble_gap_start_advertising(&advertising_params);
    if (err != ESP_OK) {
        motor_vis_advertising = false;
        motor_vis_report_error("start advertising", err);
    }
}

// Adds the GPS packet characteristic as the first custom data stream in the MotoVis service.
static void motor_vis_add_gps_characteristic(void)
{
    esp_bt_uuid_t characteristic_uuid = {0};
    esp_gatt_char_prop_t properties =
        ESP_GATT_CHAR_PROP_BIT_READ |
        ESP_GATT_CHAR_PROP_BIT_NOTIFY;

    motor_vis_uuid128_to_esp_bt_uuid(MOTOR_VIS_CHAR_UUID_GPS, &characteristic_uuid);
    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_GPS_CHARACTERISTIC;

    esp_err_t err = esp_ble_gatts_add_char(
        motor_vis_service_handle,
        &characteristic_uuid,
        ESP_GATT_PERM_READ,
        properties,
        &motor_vis_gps_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add GPS characteristic", err);
    }
}

// Adds the standard user-description descriptor so generic BLE browser apps can label the GPS packet characteristic.
static void motor_vis_add_gps_user_description(void)
{
    esp_bt_uuid_t descriptor_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = MOTOR_VIS_USER_DESCRIPTION_DESCRIPTOR_UUID,
        },
    };

    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_GPS_USER_DESCRIPTION;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        motor_vis_service_handle,
        &descriptor_uuid,
        ESP_GATT_PERM_READ,
        &motor_vis_gps_user_description_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add GPS user description", err);
    }
}

// Adds the GPS CCCD so clients can independently enable GPS notifications.
static void motor_vis_add_gps_cccd(void)
{
    esp_bt_uuid_t descriptor_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = MOTOR_VIS_CLIENT_CONFIG_DESCRIPTOR_UUID,
        },
    };

    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_GPS_CCCD;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        motor_vis_service_handle,
        &descriptor_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        &motor_vis_gps_cccd_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add GPS CCCD", err);
    }
}

// Adds the heart-rate packet characteristic after the GPS characteristic group has finished.
static void motor_vis_add_heart_characteristic(void)
{
    esp_bt_uuid_t characteristic_uuid = {0};
    esp_gatt_char_prop_t properties =
        ESP_GATT_CHAR_PROP_BIT_READ |
        ESP_GATT_CHAR_PROP_BIT_NOTIFY;

    motor_vis_uuid128_to_esp_bt_uuid(MOTOR_VIS_CHAR_UUID_HEART_RATE, &characteristic_uuid);
    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_HEART_CHARACTERISTIC;

    esp_err_t err = esp_ble_gatts_add_char(
        motor_vis_service_handle,
        &characteristic_uuid,
        ESP_GATT_PERM_READ,
        properties,
        &motor_vis_heart_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add heart characteristic", err);
    }
}

// Adds the standard user-description descriptor so generic BLE browser apps can label the heart packet characteristic.
static void motor_vis_add_heart_user_description(void)
{
    esp_bt_uuid_t descriptor_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = MOTOR_VIS_USER_DESCRIPTION_DESCRIPTOR_UUID,
        },
    };

    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_HEART_USER_DESCRIPTION;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        motor_vis_service_handle,
        &descriptor_uuid,
        ESP_GATT_PERM_READ,
        &motor_vis_heart_user_description_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add heart user description", err);
    }
}

// Adds the heart-rate CCCD so clients can independently enable heart notifications.
static void motor_vis_add_heart_cccd(void)
{
    esp_bt_uuid_t descriptor_uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = MOTOR_VIS_CLIENT_CONFIG_DESCRIPTOR_UUID,
        },
    };

    motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_HEART_CCCD;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        motor_vis_service_handle,
        &descriptor_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        &motor_vis_heart_cccd_attribute_value,
        NULL
    );
    if (err != ESP_OK) {
        motor_vis_report_error("add heart CCCD", err);
    }
}

// Handler for GATT read events, sends the current characteristic or descriptor value back to the client.
static void handle_read_event(
    esp_gatt_if_t gatts_if,
    const esp_ble_gatts_cb_param_t *param
)
{
    esp_gatt_rsp_t response = {0};
    esp_gatt_status_t status = ESP_GATT_OK;

    // Mirrors the exact handle the client requested so the BLE stack associates the response with the correct attribute.
    response.attr_value.handle = param->read.handle;

    if (param->read.handle == motor_vis_gps_characteristic_handle) {
        // GPS reads return the fixed-format custom GPS payload expected by the companion app.
        response.attr_value.len = MOTOR_VIS_GPS_PAYLOAD_LENGTH;
        memcpy(
            response.attr_value.value,
            motor_vis_gps_characteristic_value,
            MOTOR_VIS_GPS_PAYLOAD_LENGTH
        );
    } else if (param->read.handle == motor_vis_gps_user_description_handle) {
        // Exposes the standard user-description text so LightBlue and similar tools can label the GPS stream clearly.
        response.attr_value.len = sizeof(motor_vis_gps_user_description_value) - 1;
        memcpy(
            response.attr_value.value,
            motor_vis_gps_user_description_value,
            sizeof(motor_vis_gps_user_description_value) - 1
        );
    } else if (param->read.handle == motor_vis_gps_cccd_handle) {
        // Exposes the current CCCD state so clients can confirm whether notifications are enabled.
        response.attr_value.len = sizeof(motor_vis_gps_cccd_value);
        memcpy(
            response.attr_value.value,
            motor_vis_gps_cccd_value,
            sizeof(motor_vis_gps_cccd_value)
        );
    } else if (param->read.handle == motor_vis_heart_characteristic_handle) {
        // Heart reads return the fixed-format custom heart-rate payload.
        response.attr_value.len = MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH;
        memcpy(
            response.attr_value.value,
            motor_vis_heart_characteristic_value,
            MOTOR_VIS_HEART_RATE_PAYLOAD_LENGTH
        );
    } else if (param->read.handle == motor_vis_heart_user_description_handle) {
        // Exposes the standard user-description text so LightBlue and similar tools can label the heart stream clearly.
        response.attr_value.len = sizeof(motor_vis_heart_user_description_value) - 1;
        memcpy(
            response.attr_value.value,
            motor_vis_heart_user_description_value,
            sizeof(motor_vis_heart_user_description_value) - 1
        );
    } else if (param->read.handle == motor_vis_heart_cccd_handle) {
        // Exposes the heart CCCD state separately from GPS so clients can inspect subscriptions independently.
        response.attr_value.len = sizeof(motor_vis_heart_cccd_value);
        memcpy(
            response.attr_value.value,
            motor_vis_heart_cccd_value,
            sizeof(motor_vis_heart_cccd_value)
        );
    } else {
        status = ESP_GATT_NOT_FOUND;
    }

    esp_err_t err = esp_ble_gatts_send_response(
        gatts_if,
        param->read.conn_id,
        param->read.trans_id,
        status,
        (status == ESP_GATT_OK) ? &response : NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to send read response: %s", esp_err_to_name(err));
    }
}

// Handler for GATT write events, updates the CCCD based on the client's subscription request.
static void handle_write_event(
    esp_gatt_if_t gatts_if,
    const esp_ble_gatts_cb_param_t *param
)
{
    esp_gatt_status_t status = ESP_GATT_OK;

    // Checks if the write operation is a prepared write, which is not supported in this implementation.
    if (param->write.is_prep) {
        status = ESP_GATT_REQ_NOT_SUPPORTED;
    // TODO: Testing Code
    // } else if (param->write.handle == motor_vis_characteristic_handle) {
    //     uint16_t copy_length = param->write.len;
    //
    //     if (copy_length >= MOTOR_VIS_MAX_VALUE_LENGTH) {
    //         copy_length = MOTOR_VIS_MAX_VALUE_LENGTH - 1;
    //     }
    //
    //     memcpy(motor_vis_characteristic_value, param->write.value, copy_length);
    //     motor_vis_characteristic_value[copy_length] = '\0';
    //     motor_vis_characteristic_length = copy_length;
    //
    //     esp_err_t err = esp_ble_gatts_set_attr_value(
    //         motor_vis_characteristic_handle,
    //         motor_vis_characteristic_length,
    //         motor_vis_characteristic_value
    //     );
    //
    //     if (err != ESP_OK) {
    //         ESP_LOGE(TAG, "failed to update characteristic value: %s", esp_err_to_name(err));
    //         status = ESP_GATT_ERROR;
    //     } else {
    //         ESP_LOGI(
    //             TAG,
    //             "characteristic updated from phone: %.*s",
    //             motor_vis_characteristic_length,
    //             motor_vis_characteristic_value
    //         );
    //     }
    // Previous BLE test behavior accepted arbitrary phone-written strings on the single placeholder characteristic.
    } else if (param->write.handle == motor_vis_gps_cccd_handle) {
        // Only the CCCD is writable here because the firmware owns and publishes the GPS payload.
        if (param->write.len != MOTOR_VIS_CLIENT_CONFIG_LENGTH) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        } else {
            // Reads the little-endian CCCD bitfield to determine whether notifications are being enabled or disabled.
            uint16_t client_config_value =
                (uint16_t) param->write.value[0] |
                ((uint16_t) param->write.value[1] << 8);

            if (client_config_value == 0x0000 || client_config_value == 0x0001) {
                motor_vis_gps_cccd_value[0] = param->write.value[0];
                motor_vis_gps_cccd_value[1] = param->write.value[1];
                motor_vis_gps_notifications_enabled = (client_config_value == 0x0001);

                esp_err_t err = esp_ble_gatts_set_attr_value(
                    motor_vis_gps_cccd_handle,
                    sizeof(motor_vis_gps_cccd_value),
                    motor_vis_gps_cccd_value
                );
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to update GPS CCCD: %s", esp_err_to_name(err));
                    status = ESP_GATT_ERROR;
                } else {
                    ESP_LOGI(
                        TAG,
                        "GPS notifications %s",
                        motor_vis_gps_notifications_enabled ? "enabled" : "disabled"
                    );
                }
            } else {
                status = ESP_GATT_REQ_NOT_SUPPORTED;
            }
        }
    } else if (param->write.handle == motor_vis_heart_cccd_handle) {
        // Heart CCCD writes are handled separately so the app can subscribe to pulse data without subscribing to GPS.
        if (param->write.len != MOTOR_VIS_CLIENT_CONFIG_LENGTH) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        } else {
            // Reads the little-endian CCCD bitfield to determine whether heart notifications are being enabled or disabled.
            uint16_t client_config_value =
                (uint16_t) param->write.value[0] |
                ((uint16_t) param->write.value[1] << 8);

            if (client_config_value == 0x0000 || client_config_value == 0x0001) {
                motor_vis_heart_cccd_value[0] = param->write.value[0];
                motor_vis_heart_cccd_value[1] = param->write.value[1];
                motor_vis_heart_notifications_enabled = (client_config_value == 0x0001);

                esp_err_t err = esp_ble_gatts_set_attr_value(
                    motor_vis_heart_cccd_handle,
                    sizeof(motor_vis_heart_cccd_value),
                    motor_vis_heart_cccd_value
                );
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to update heart CCCD: %s", esp_err_to_name(err));
                    status = ESP_GATT_ERROR;
                } else {
                    ESP_LOGI(
                        TAG,
                        "heart notifications %s",
                        motor_vis_heart_notifications_enabled ? "enabled" : "disabled"
                    );
                }
            } else {
                status = ESP_GATT_REQ_NOT_SUPPORTED;
            }
        }
    } else {
        status = ESP_GATT_WRITE_NOT_PERMIT;
    }

    if (param->write.need_rsp) {
        esp_err_t err = esp_ble_gatts_send_response(
            gatts_if,
            param->write.conn_id,
            param->write.trans_id,
            status,
            NULL
        );
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to send write response: %s", esp_err_to_name(err));
        }
    }
}
// Handler for GAP events, manages advertising state and connection parameter updates.
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "advertising data configured");
        motor_vis_adv_data_configured = true;
        start_advertising("adv data configured");
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "scan response data configured");
        motor_vis_scan_rsp_configured = true;
        start_advertising("scan response configured");
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "advertising started as \"%s\"", MOTOR_VIS_DEVICE_NAME);
            motor_vis_advertising = true;
            motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ADVERTISING);
            motor_vis_sync_pairing_led_with_ble_state();
        } else {
            ESP_LOGE(TAG, "advertising failed to start, status=%d", param->adv_start_cmpl.status);
            motor_vis_advertising = false;
            motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ERROR);
            motor_vis_sync_pairing_led_with_ble_state();
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "advertising stopped, status=%d", param->adv_stop_cmpl.status);
        motor_vis_advertising = false;
        if (!motor_vis_connected) {
            motor_vis_sync_pairing_led_with_ble_state();
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(
            TAG,
            "connection params updated: status=%d min=%d max=%d latency=%d timeout=%d",
            param->update_conn_params.status,
            param->update_conn_params.min_int,
            param->update_conn_params.max_int,
            param->update_conn_params.latency,
            param->update_conn_params.timeout
        );
        break;

    default:
        break;
    }
}

// Handler for GATT server events, manages service creation, characteristic read/write, and connection events.
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        esp_err_t err;

        ESP_LOGI(TAG, "gatt server registered, status=%d app_id=%d", param->reg.status, param->reg.app_id);
        motor_vis_gatts_if = gatts_if;

        err = esp_ble_gap_set_device_name(MOTOR_VIS_DEVICE_NAME);
        if (err != ESP_OK) {
            motor_vis_report_error("set device name", err);
            return;
        }

        // Advertising payload setup completes asynchronously, so GAP callbacks later mark when each payload is ready.
        err = esp_ble_gap_config_adv_data(&advertising_data);
        if (err != ESP_OK) {
            motor_vis_report_error("configure advertising data", err);
            return;
        }

        err = esp_ble_gap_config_adv_data(&scan_response_data);
        if (err != ESP_OK) {
            motor_vis_report_error("configure scan response data", err);
            return;
        }

        // Builds the primary service from the shared 128-bit MotoVis service UUID.
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {0},
            },
        };
        motor_vis_uuid128_to_esp_bt_uuid(MOTOR_VIS_SERVICE_UUID, &service_id.id.uuid);

        err = esp_ble_gatts_create_service(gatts_if, &service_id, MOTOR_VIS_NUM_HANDLES);
        if (err != ESP_OK) {
            motor_vis_report_error("create service", err);
        }
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        // TODO: Testing Code
        // esp_bt_uuid_t characteristic_uuid = {
        //     .len = ESP_UUID_LEN_16,
        //     .uuid = {
        //         .uuid16 = MOTOR_VIS_CHARACTERISTIC_UUID,
        //     },
        // };
        // esp_gatt_char_prop_t properties =
        //     ESP_GATT_CHAR_PROP_BIT_READ |
        //     ESP_GATT_CHAR_PROP_BIT_WRITE;
        // Previous placeholder characteristic used a temporary 16-bit UUID and allowed the phone to write string data.

        motor_vis_service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "service created, handle=%d", motor_vis_service_handle);

        esp_err_t err = esp_ble_gatts_start_service(motor_vis_service_handle);
        if (err != ESP_OK) {
            motor_vis_report_error("start service", err);
            return;
        }

        // Starts the sequential GATT database build: GPS value/CCCD first, then heart-rate value/CCCD.
        motor_vis_add_gps_characteristic();
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        esp_err_t err = ESP_OK;

        if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_GPS_CHARACTERISTIC) {
            // Remembers the GPS characteristic handle so GNSS updates can be pushed into the BLE attribute database.
            motor_vis_gps_characteristic_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "GPS characteristic added, handle=%d", motor_vis_gps_characteristic_handle);

            // Primes the newly created characteristic with an invalid payload until GNSS has real data available.
            err = motor_vis_publish_gnss_fix(NULL, false);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "failed to prime GPS characteristic: %s", esp_err_to_name(err));
            }

            // Adds the standard user-description descriptor before the CCCD so browser apps can display a friendly label.
            motor_vis_add_gps_user_description();
        } else if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_HEART_CHARACTERISTIC) {
            // Remembers the heart characteristic handle so PulseSensor updates can be pushed into the BLE attribute database.
            motor_vis_heart_characteristic_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "heart characteristic added, handle=%d", motor_vis_heart_characteristic_handle);

            // Primes the newly created characteristic with an invalid payload until the first beat sample arrives.
            err = motor_vis_publish_heart_sample(NULL, false);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "failed to prime heart characteristic: %s", esp_err_to_name(err));
            }

            // Adds the standard user-description descriptor before the CCCD so browser apps can display a friendly label.
            motor_vis_add_heart_user_description();
        } else {
            ESP_LOGW(TAG, "unexpected characteristic added while GATT build state=%d", motor_vis_gatt_build_state);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_GPS_USER_DESCRIPTION) {
            // Remembers the GPS user-description handle so manual read responses can expose the friendly label text.
            motor_vis_gps_user_description_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "GPS user description added, handle=%d", motor_vis_gps_user_description_handle);
            motor_vis_add_gps_cccd();
        } else if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_GPS_CCCD) {
            // Remember the GPS CCCD handle so BLE writes can toggle location notification state correctly.
            motor_vis_gps_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "GPS CCCD added, handle=%d", motor_vis_gps_cccd_handle);
            motor_vis_reset_gps_cccd_state();

            // Continue building the service by adding the heart-rate characteristic after the GPS group is complete.
            motor_vis_add_heart_characteristic();
        } else if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_HEART_USER_DESCRIPTION) {
            // Remembers the heart user-description handle so manual read responses can expose the friendly label text.
            motor_vis_heart_user_description_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "heart user description added, handle=%d", motor_vis_heart_user_description_handle);
            motor_vis_add_heart_cccd();
        } else if (motor_vis_gatt_build_state == MOTOR_VIS_GATT_BUILD_HEART_CCCD) {
            // Remember the heart CCCD handle so BLE writes can toggle pulse notification state correctly.
            motor_vis_heart_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "heart CCCD added, handle=%d", motor_vis_heart_cccd_handle);
            motor_vis_reset_heart_cccd_state();
            motor_vis_gatt_build_state = MOTOR_VIS_GATT_BUILD_COMPLETE;
            ESP_LOGI(TAG, "MotoVis GATT database build complete");
        } else {
            ESP_LOGW(TAG, "unexpected descriptor added while GATT build state=%d", motor_vis_gatt_build_state);
        }
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "read request on handle=%d", param->read.handle);
        handle_read_event(gatts_if, param);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "write request on handle=%d len=%d", param->write.handle, param->write.len);
        handle_write_event(gatts_if, param);
        break;

    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t connection_params = {0};
        uint16_t previous_connection_id = motor_vis_connection_id;
        bool replacing_existing_connection =
            motor_vis_connected &&
            (previous_connection_id != param->connect.conn_id);

        if (replacing_existing_connection &&
            motor_vis_gatts_if != ESP_GATT_IF_NONE) {
            // This firmware still tracks one active BLE client, so a new connection replaces the previously active session.
            esp_err_t close_previous_err = esp_ble_gatts_close(motor_vis_gatts_if, previous_connection_id);
            if (close_previous_err != ESP_OK) {
                ESP_LOGW(
                    TAG,
                    "new BLE client connected while another session was active, but closing conn_id=%d failed: %s",
                    previous_connection_id,
                    esp_err_to_name(close_previous_err)
                );
            } else {
                ESP_LOGW(
                    TAG,
                    "new BLE client connected while another session was active, closing previous conn_id=%d",
                    previous_connection_id
                );
            }
        }

        motor_vis_connected = true;
        motor_vis_advertising = false;
        motor_vis_connection_id = param->connect.conn_id;
        // Reset subscription state so every new connection must explicitly enable each notification stream again.
        motor_vis_reset_all_cccd_state();

        memcpy(connection_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        connection_params.latency = 0;
        connection_params.min_int = 0x10;
        connection_params.max_int = 0x20;
        connection_params.timeout = 400;

        ESP_LOGI(
            TAG,
            "phone connected, conn_id=%d remote=" ESP_BD_ADDR_STR,
            motor_vis_connection_id,
            ESP_BD_ADDR_HEX(param->connect.remote_bda)
        );
        motor_vis_set_led_state(MOTOR_VIS_LED_STATE_CONNECTED);

        // Any successful BLE connection ends the visible pairing indication with a short success blink sequence.
        if (motor_vis_jacket_controls_ready) {
            esp_err_t pairing_led_err = jacket_controls_start_led_blink_sequence(
                JACKET_CONTROL_LED_PAIRING,
                2,
                150,
                JACKET_CONTROL_LED_OFF
            );
            if (pairing_led_err != ESP_OK) {
                ESP_LOGW(TAG, "failed to run pairing success LED sequence: %s", esp_err_to_name(pairing_led_err));
            } else {
                ESP_LOGI(TAG, "BLE connection established, blinking pairing LED twice before turning it off");
            }
        }

        esp_ble_gap_update_conn_params(&connection_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        if (motor_vis_connected &&
            param->disconnect.conn_id != motor_vis_connection_id) {
            // Ignore teardown from an older session after a newer connection already took ownership of the active BLE state.
            ESP_LOGI(
                TAG,
                "ignoring stale disconnect for non-active conn_id=%d remote=" ESP_BD_ADDR_STR,
                param->disconnect.conn_id,
                ESP_BD_ADDR_HEX(param->disconnect.remote_bda)
            );
            break;
        }

        motor_vis_connected = false;
        motor_vis_connection_id = 0;
        motor_vis_reset_all_cccd_state();
        ESP_LOGI(
            TAG,
            "phone disconnected, conn_id=%d reason=0x%02x remote=" ESP_BD_ADDR_STR,
            param->disconnect.conn_id,
            param->disconnect.reason,
            ESP_BD_ADDR_HEX(param->disconnect.remote_bda)
        );
        motor_vis_advertising = false;
        if (motor_vis_light_sleep_transition_active) {
            // Suppresses the normal reconnect path while the firmware is intentionally entering temporary Light-sleep.
            ESP_LOGI(TAG, "BLE disconnect occurred during Light-sleep transition; advertising restart deferred");
            motor_vis_status_led_apply_rgb(0, 0, 0);
        } else {
            motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ADVERTISING);
            start_advertising("disconnect");
        }
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "mtu updated to %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}

// Main App Init Function.
void app_main(void)
{
    esp_err_t err;
    // Tracks whether the PulseSensor manager initialized cleanly so the main loop only publishes live heart samples when ready.
    bool heart_sensor_ready = false;
    // TODO: Testing Code
    // bool alert_test_led_enabled = false;
    // Previous alert button validation toggled the alert LED directly before a cancellable alert state existed.

    ESP_LOGI(TAG, "booting MotoVis Feather BLE test firmware");
    // Initialize status LED first so we can indicate errors during setup.
    err = motor_vis_status_led_init();
    // If LED initialization fails, we can't indicate the error state, so we log it and stop.
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "status NeoPixel init failed: %s", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /**
     * Create FreeRTOS task to manage the LED state machine.
     * - motor_vis_status_led_task will run in an infinite loop, updating the LED based on the current state.
     * - "motor_vis_led" is the name of the task for debugging purposes.
     * - 2048 is the stack size allocated for the task, which should be sufficient for its needs.
     * - NULL is the parameter passed to the task, which is not used in this case.
     * - 3 is the priority of the task, which is set to a moderate level to ensure it runs smoothly without starving other tasks.
     * - NULL is the task handle, which is not needed since we don't need to reference the task after creation.
     */
    // TODO: Find optimal stack size and priority for the LED task, as it may need to be adjusted based on actual performance and responsiveness requirements.
    xTaskCreate(motor_vis_status_led_task, "motor_vis_led", 2048, NULL, 3, NULL);
    // Set initial LED state to booting while we initialize the system.
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_BOOTING);

    // Initialize NVS, which is required for BLE to store bonding information and other data.
    err = nvs_flash_init();
    // If NVS initialization fails due to no free pages or a new version, it is erased and tried again.
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // If NVS initialization still fails, we log the error and set the LED to error state.
    if (err != ESP_OK) {
        motor_vis_report_error("initialize NVS", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initializes the reusable GNSS manager before BLE enters its long-running supervision loop.
    err = motor_vis_gnss_init();
    if (err != ESP_OK) {
        // GNSS startup is non-fatal so BLE can keep advertising even if the GPS hardware is absent or still not ready.
        ESP_LOGE(TAG, "GNSS manager init failed, continuing BLE without live GPS: %s", esp_err_to_name(err));
        err = motor_vis_publish_gnss_fix(NULL, false);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to prime invalid GPS payload after GNSS init failure: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "GNSS manager initialized successfully");
    }

    // Initializes the separate jacket controls component so button debounce and external LED handling stay out of the BLE module.
    err = jacket_controls_init();

    if (err != ESP_OK) {
        // Control startup is a non-fatal task so BLE and GNSS can still run during early hardware bring-up.
        ESP_LOGE(
            TAG,
            "jacket controls init failed, continuing without external button validation: %s",
            esp_err_to_name(err)
        );
    } else {
        motor_vis_jacket_controls_ready = true;

        // Explicitly turn the external validation LEDs off at startup so button testing begins from a known state.
        err = jacket_controls_set_all_leds_off();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to clear jacket control LEDs at startup: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "jacket controls initialized successfully");
        }
    }

    // Initializes the heart sensor manager before BLE starts so the heart characteristic can be primed with real samples when beats arrive.
    err = motor_vis_heart_sensor_init();
    if (err != ESP_OK) {
        // Heart startup is non-fatal so GPS, controls, and BLE can still run if the PulseSensor is absent during bring-up.
        ESP_LOGE(TAG, "heart sensor init failed, continuing BLE without live heart data: %s", esp_err_to_name(err));
        err = motor_vis_publish_heart_sample(NULL, false);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to prime invalid heart payload after heart init failure: %s", esp_err_to_name(err));
        }
    } else {
        heart_sensor_ready = true;
        ESP_LOGI(TAG, "heart sensor manager initialized successfully");
    }

    // Initialize Bluetooth controller with default configuration.
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_BLE_INITIALIZING);
    // Since we only need BLE functionality, we can release memory allocated for Classic Bluetooth.
    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    // If memory release fails, we log the error and set the LED to error state.
    if (err != ESP_OK) {
        motor_vis_report_error("release classic BT memory", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initialize the Bluetooth controller with the default configuration. This sets up the necessary hardware and software components for BLE operation.
    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_config);
    if (err != ESP_OK) {
        motor_vis_report_error("initialize BT controller", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Enable the Bluetooth controller in BLE mode.
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        motor_vis_report_error("enable BLE controller", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    // Initialize Bluedroid.
    esp_bluedroid_config_t bluedroid_config = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    err = esp_bluedroid_init_with_cfg(&bluedroid_config);
    if (err != ESP_OK) {
        motor_vis_report_error("initialize Bluedroid", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Enable Bluedroid.
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        motor_vis_report_error("enable Bluedroid", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Register GAP and GATTS event handlers.
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        motor_vis_report_error("register GAP callback", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Registered GAP and GATTS event handlers");

    // Register GATTS callback to handle GATT server events such as read requests, CCCD writes, and connection events.
    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) {
        motor_vis_report_error("register GATTS callback", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Register the GATT application with the Bluetooth stack.
    err = esp_ble_gatts_app_register(MOTOR_VIS_APP_ID);
    if (err != ESP_OK) {
        motor_vis_report_error("register GATT app", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Set the local MTU to the maximum allowed value to support larger characteristic values and improve performance.
    err = esp_ble_gatt_set_local_mtu(247);
    if (err != ESP_OK) {
        motor_vis_report_error("set local MTU", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Marks BLE setup as complete so the GAP callback path and supervisor loop know advertising can safely start.
    motor_vis_ble_ready = true;

    // Main loop to monitor BLE state, restart advertising if needed, and publish new GNSS fixes into the GPS characteristic.
    ESP_LOGI(TAG, "waiting for BLE client connection");

    while (true) {
        // Restarts advertising if the firmware becomes idle without an active connection or an active advertisement.
        if (motor_vis_is_advertising_ready() &&
            !motor_vis_connected &&
            !motor_vis_advertising) {
            ESP_LOGW(TAG, "no active BLE connection or advertisement, retrying advertising");
            start_advertising("supervisor retry");
        }

        // Pulls the latest GNSS fix from the separate manager and only publish when the sequence number advances.
        motor_vis_gnss_fix_t latest_fix = {0};
        if (motor_vis_gnss_get_latest_fix(&latest_fix) &&
            latest_fix.sequence > motor_vis_last_published_gnss_sequence) {
            err = motor_vis_publish_gnss_fix(&latest_fix, true);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "failed to publish GPS characteristic update: %s", esp_err_to_name(err));
            } else {
                motor_vis_last_published_gnss_sequence = latest_fix.sequence;
                ESP_LOGI(
                    TAG,
                    "published GPS update seq=%lu valid=%u lat=%ld lon=%ld",
                    (unsigned long) latest_fix.sequence,
                    (unsigned int) latest_fix.fix_valid,
                    (long) latest_fix.latitude_e7,
                    (long) latest_fix.longitude_e7
                );
            }
        }

        // Pulls the latest heart sample from the Arduino-backed manager and only publishes when a new beat sequence arrives.
        if (heart_sensor_ready) {
            motor_vis_heart_sample_t latest_heart_sample = {0};

            if (motor_vis_heart_sensor_get_latest_sample(&latest_heart_sample) &&
                latest_heart_sample.sequence > motor_vis_last_published_heart_sequence) {
                // Logs the newly captured pulse sample as soon as the main firmware sees it so terminal output reflects
                // live sensor activity even before anyone inspects the BLE packet bytes.
                motor_vis_log_live_heart_sample(&latest_heart_sample);

                err = motor_vis_publish_heart_sample(&latest_heart_sample, true);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to publish heart characteristic update: %s", esp_err_to_name(err));
                } else {
                    motor_vis_last_published_heart_sequence = latest_heart_sample.sequence;
                    ESP_LOGI(
                        TAG,
                        "published heart update seq=%lu valid=%u bpm=%u raw_adc=%u",
                        (unsigned long) latest_heart_sample.sequence,
                        (unsigned int) latest_heart_sample.bpm_valid,
                        (unsigned int) latest_heart_sample.bpm,
                        (unsigned int) latest_heart_sample.raw_adc
                    );
                }
            }
        }

        // Drains all queued external button presses so quick hardware-validation taps are handled even if BLE work was busy first.
        if (motor_vis_jacket_controls_ready) {
            jacket_control_button_event_t control_event = {0};

            while (jacket_controls_get_next_event(&control_event)) {
                switch (control_event.button_id) {
                case JACKET_CONTROL_BUTTON_PAIRING:
                    ESP_LOGI(
                        TAG,
                        "pairing button pressed seq=%lu at %lu ms",
                        (unsigned long) control_event.sequence,
                        (unsigned long) control_event.timestamp_ms
                    );

                    // Pairing LED now follows the actual BLE advertising state, so this button requests a fresh advertising cycle whenever discoverability is needed.
                    if (motor_vis_advertising) {
                        ESP_LOGI(TAG, "pairing button pressed while advertising is already active");
                    } else if (motor_vis_is_advertising_ready()) {
                        if (motor_vis_connected) {
                            ESP_LOGW(TAG, "pairing button pressed while connected, restarting advertising so another device can connect");
                        } else {
                            ESP_LOGW(TAG, "pairing button triggered advertising restart");
                        }
                        start_advertising("User initiated pairing");
                    } else {
                        ESP_LOGW(TAG, "pairing button pressed before BLE advertising was ready");
                    }
                    break;

                case JACKET_CONTROL_BUTTON_ALERT_CANCEL:
                    // TODO: Testing Code
                    // alert_test_led_enabled = !alert_test_led_enabled;
                    // ESP_LOGI(
                    //     TAG,
                    //     "alert cancel button pressed seq=%lu at %lu ms, test LED now %s",
                    //     (unsigned long) control_event.sequence,
                    //     (unsigned long) control_event.timestamp_ms,
                    //     alert_test_led_enabled ? "ON" : "OFF"
                    // );
                    //
                    // err = jacket_controls_set_led_mode(
                    //     JACKET_CONTROL_LED_ALERT,
                    //     alert_test_led_enabled ? JACKET_CONTROL_LED_ON : JACKET_CONTROL_LED_OFF
                    // );
                    // if (err != ESP_OK) {
                    //     ESP_LOGW(TAG, "failed to update alert test LED: %s", esp_err_to_name(err));
                    // }
                    // Previous alert button validation toggled the alert LED on every press.

                    ESP_LOGI(
                        TAG,
                        "alert cancel button pressed seq=%lu at %lu ms",
                        (unsigned long) control_event.sequence,
                        (unsigned long) control_event.timestamp_ms
                    );

                    if (motor_vis_alert_cancel_window_active) {
                        // Cancels the pending alert locally; future crash logic will be responsible for opening this window.
                        motor_vis_alert_cancel_window_active = false;
                        ESP_LOGI(TAG, "pending alert cancelled before companion app notification was sent");

                        err = jacket_controls_set_led_mode(JACKET_CONTROL_LED_ALERT, JACKET_CONTROL_LED_OFF);
                        if (err != ESP_OK) {
                            ESP_LOGW(TAG, "failed to turn alert LED off after cancel: %s", esp_err_to_name(err));
                        }
                    } else {
                        ESP_LOGI(TAG, "alert cancel button ignored because no cancellable alert is active");
                    }
                    break;

                case JACKET_CONTROL_BUTTON_POWER:
                    ESP_LOGW(
                        TAG,
                        "power button pressed seq=%lu at %lu ms; entering temporary Light-sleep until real hardware power-off is available",
                        (unsigned long) control_event.sequence,
                        (unsigned long) control_event.timestamp_ms
                    );

                    err = motor_vis_enter_temporary_light_sleep();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "temporary Light-sleep transition failed: %s", esp_err_to_name(err));
                    }
                    break;

                default:
                    // Logs unexpected IDs so future control expansions are easier to debug during bring-up.
                    ESP_LOGW(TAG, "received unknown jacket control button id=%d", control_event.button_id);
                    break;
                }
            }
        }

        // TODO: Testing Code
        // vTaskDelay(pdMS_TO_TICKS(5000));
        // Previous slower supervisor loop cadence used before GNSS polling was added to the main BLE loop.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
