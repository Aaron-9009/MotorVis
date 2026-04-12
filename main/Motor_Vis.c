// Motor_Vis.c - BLE GATT server for MotoVis Jacket
//
// Implements a simple BLE GATT server that allows a connected phone to read and write
// a single characteristic. The device advertises itself as "MotoVis Jacket" and uses the onboard NeoPixel
// for status indication. 
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

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "led_strip.h"

// App specific definitions
#define MOTOR_VIS_APP_ID                0x55
#define MOTOR_VIS_CHARACTERISTIC_UUID   0xFFF1
#define MOTOR_VIS_NUM_HANDLES           4
#define MOTOR_VIS_DEVICE_NAME           "MotoVis Jacket"
#define MOTOR_VIS_MAX_VALUE_LENGTH      64
#define MOTOR_VIS_STATUS_LED_GPIO       GPIO_NUM_0
#define MOTOR_VIS_LED_POWER_GPIO        GPIO_NUM_2
#define MOTOR_VIS_NEOPIXEL_COUNT        1

// Tag for logging in terminal and error reporting
static const char *TAG = "MOTOR_VIS_BLE";

// 128-bit service UUID stored as a static array so it can be used by both the
// GATT service definition and the BLE advertising payload.
static const uint8_t MOTOR_VIS_SERVICE_UUID[16] = {
    0xf1, 0xff, 0xf0, 0xff, 0xf0, 0x00, 0xff, 0xff, 
    0xf0, 0xff, 0xf0, 0xff, 0x00, 0xff, 0xff, 0xff
};

// LED state machine for status indication
typedef enum {
    MOTOR_VIS_LED_STATE_BOOTING = 0,
    MOTOR_VIS_LED_STATE_BLE_INITIALIZING,
    MOTOR_VIS_LED_STATE_ADVERTISING,
    MOTOR_VIS_LED_STATE_CONNECTED,
    MOTOR_VIS_LED_STATE_ERROR,
} motor_vis_led_state_t;


static uint16_t motor_vis_service_handle = 0;
static uint16_t motor_vis_characteristic_handle = 0;
static esp_gatt_if_t motor_vis_gatts_if = ESP_GATT_IF_NONE;
static uint16_t motor_vis_connection_id = 0;
static bool motor_vis_connected = false;
static bool motor_vis_ble_ready = false;
static bool motor_vis_adv_data_configured = false;
static bool motor_vis_scan_rsp_configured = false;
static bool motor_vis_advertising = false;
static volatile motor_vis_led_state_t motor_vis_led_state = MOTOR_VIS_LED_STATE_BOOTING;
static led_strip_handle_t motor_vis_status_pixel = NULL;

static uint8_t motor_vis_characteristic_value[MOTOR_VIS_MAX_VALUE_LENGTH] = "MotoVis Ready";
static uint16_t motor_vis_characteristic_length = 13;

// GATT attribute value structure for the characteristic
static esp_attr_value_t motor_vis_attribute_value = {
    .attr_max_len = MOTOR_VIS_MAX_VALUE_LENGTH,
    .attr_len = 13,
    .attr_value = motor_vis_characteristic_value,
};

// Advertising data configuration
static esp_ble_adv_data_t advertising_data = {
    .set_scan_rsp = false,
    // .include_name = false,
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
// human-readable device name while the scan response carries the 128-bit
// service UUID for clients that inspect the full payload.
static esp_ble_adv_data_t scan_response_data = {
    .set_scan_rsp = true,
    // .include_name = true,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x0000,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    // .service_uuid_len = 0,
    .service_uuid_len = sizeof(MOTOR_VIS_SERVICE_UUID),
    // .p_service_uuid = NULL,
    .p_service_uuid = (uint8_t *)MOTOR_VIS_SERVICE_UUID,
    .flag = 0,
};

// Advertising parameters configuration
static esp_ble_adv_params_t advertising_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void start_advertising(const char *reason);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static esp_err_t motor_vis_status_led_init(void);
static void motor_vis_set_led_state(motor_vis_led_state_t state);
static void motor_vis_status_led_task(void *parameter);
static void motor_vis_report_error(const char *step, esp_err_t err);
static void motor_vis_status_led_apply_rgb(uint8_t red, uint8_t green, uint8_t blue);
static bool motor_vis_is_advertising_ready(void);


// Utility function to log errors and set LED to error state
static void motor_vis_report_error(const char *step, esp_err_t err)
{
    //Logs error
    ESP_LOGE(TAG, "%s failed: %s", step, esp_err_to_name(err)); 
    //Sets LED to error state
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

// Function to apply RGB color to the status LED
static void motor_vis_status_led_apply_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (motor_vis_status_pixel == NULL) {
        return;
    }

    led_strip_set_pixel(motor_vis_status_pixel, 0, red, green, blue);
    led_strip_refresh(motor_vis_status_pixel);
}

// Function to initialize the status LED (NeoPixel) and its power control
static esp_err_t motor_vis_status_led_init(void)
{
    // Configure the GPIO for NeoPixel power control
    gpio_config_t power_config = {
        .pin_bit_mask = 1ULL << MOTOR_VIS_LED_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    //Error handling 
    ESP_RETURN_ON_ERROR(gpio_config(&power_config), TAG, "failed to configure NeoPixel power pin");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOTOR_VIS_LED_POWER_GPIO, 1), TAG, "failed to enable NeoPixel power");

    // Configure the LED strip (NeoPixel) parameters
    led_strip_config_t strip_config = {
        .strip_gpio_num = MOTOR_VIS_STATUS_LED_GPIO,
        .max_leds = MOTOR_VIS_NEOPIXEL_COUNT,
    };

    // RMT configuration for the LED strip driver
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    // Create a new LED strip device using the RMT driver and the specified configuration
    ESP_RETURN_ON_ERROR(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &motor_vis_status_pixel),
        TAG,
        "failed to create NeoPixel device"
    );
    ESP_RETURN_ON_ERROR(led_strip_clear(motor_vis_status_pixel), TAG, "failed to clear NeoPixel");

    return ESP_OK;
}

// Function to update the LED state machine
static void motor_vis_set_led_state(motor_vis_led_state_t state)
{
    motor_vis_led_state = state;
}

// FreeRTOS task to manage the status LED based on the current state
static void motor_vis_status_led_task(void *parameter)
{
    bool led_on = false;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Default delay between LED updates, can be overridden by specific states
        TickType_t delay_ticks = pdMS_TO_TICKS(250);
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;

        // Switch statement to determine LED behavior based on the current state
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

        // Apply the determined RGB color to the LED based on the current state
        if (led_on) {
            motor_vis_status_led_apply_rgb(red, green, blue);
        } else {
            motor_vis_status_led_apply_rgb(0, 0, 0);
        }

        // Delay until the next update based on the determined delay for the current state
        vTaskDelayUntil(&last_wake_time, delay_ticks);
    }
}

// Function to start BLE advertising if the BLE setup is complete, otherwise logs a warning
static void start_advertising(const char *reason)
{
    // Check if BLE is ready and advertising data is configured before starting advertising
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

// Handler for GATT read events, sends the current characteristic value back to the client
static void handle_read_event(
    esp_gatt_if_t gatts_if, //Receives requets from the client 
    const esp_ble_gatts_cb_param_t *param //Contains details about the read request, including connection ID, transaction ID, and attribute handle
)
{
    //Initializes a response structure to hold the attribute value and metadata for the read response
    esp_gatt_rsp_t response = {0};


    response.attr_value.handle = param->read.handle;
    response.attr_value.len = motor_vis_characteristic_length;
    // Copies the current characteristic value into the response structure to be sent back to the client
    memcpy(
        response.attr_value.value,
        motor_vis_characteristic_value,
        motor_vis_characteristic_length
    );

    // Sends the read response back to the client using the GATT server API
    esp_err_t err = esp_ble_gatts_send_response(
        gatts_if,
        param->read.conn_id,
        param->read.trans_id,
        ESP_GATT_OK,
        &response
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to send read response: %s", esp_err_to_name(err));
    }
}

// Handler for GATT write events, updates the characteristic value based on the client's write request and sends a response if needed
static void handle_write_event(
    esp_gatt_if_t gatts_if,
    const esp_ble_gatts_cb_param_t *param
)
{
    esp_gatt_status_t status = ESP_GATT_OK;

    // Checks if the write operation is a prepared write, which is not supported in this implementation, and sets the status accordingly
    if (param->write.is_prep) {
        status = ESP_GATT_REQ_NOT_SUPPORTED;
    } else if (param->write.handle == motor_vis_characteristic_handle) {
        uint16_t copy_length = param->write.len;

        // Ensures that the length of the incoming data does not exceed the maximum allowed length for the characteristic value
        if (copy_length >= MOTOR_VIS_MAX_VALUE_LENGTH) {
            copy_length = MOTOR_VIS_MAX_VALUE_LENGTH - 1;
        }

        // Copies the incoming data from the client's write request into the characteristic value buffer, ensuring it is null-terminated and updates the length accordingly
        memcpy(motor_vis_characteristic_value, param->write.value, copy_length);
        motor_vis_characteristic_value[copy_length] = '\0';
        motor_vis_characteristic_length = copy_length;


        esp_err_t err = esp_ble_gatts_set_attr_value(
            motor_vis_characteristic_handle,
            motor_vis_characteristic_length,
            motor_vis_characteristic_value
        );


        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to update characteristic value: %s", esp_err_to_name(err));
            status = ESP_GATT_ERROR;
        } else {
            ESP_LOGI(
                TAG,
                "characteristic updated from phone: %.*s",
                motor_vis_characteristic_length,
                motor_vis_characteristic_value
            );
        }
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

// Handler for GAP events, manages advertising state and connection parameter updates
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
        } else {
            ESP_LOGE(TAG, "advertising failed to start, status=%d", param->adv_start_cmpl.status);
            motor_vis_advertising = false;
            motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ERROR);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "advertising stopped, status=%d", param->adv_stop_cmpl.status);
        motor_vis_advertising = false;
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

// Handler for GATT server events, manages service creation, characteristic read/write, and connection events
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //Switch statement to handle different GATT server events such as registration, service creation, characteristic addition, read/write requests, and connection events
    switch (event) {

    // Handles the event when the GATT server is registered, sets up the device name and advertising data, and creates the primary service
    case ESP_GATTS_REG_EVT: {

        esp_err_t err;

        ESP_LOGI(TAG, "gatt server registered, status=%d app_id=%d", param->reg.status, param->reg.app_id);
        motor_vis_gatts_if = gatts_if;

        err = esp_ble_gap_set_device_name(MOTOR_VIS_DEVICE_NAME);
        if (err != ESP_OK) {
            motor_vis_report_error("set device name", err);
            return;
        }

        //ESP_LOGI(TAG, "--TESTING--, Service UUID Len=%d ", advertising_data.service_uuid_len);

        err = esp_ble_gap_config_adv_data(&advertising_data);
        if (err != ESP_OK) {
            motor_vis_report_error("configure advertising data", err);
            return;
        }
        // ESP_LOGI(
        //     TAG,
        //     "--TESTING --, Service UUID: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        //     MOTOR_VIS_SERVICE_UUID[0], MOTOR_VIS_SERVICE_UUID[1], MOTOR_VIS_SERVICE_UUID[2], MOTOR_VIS_SERVICE_UUID[3],
        //     MOTOR_VIS_SERVICE_UUID[4], MOTOR_VIS_SERVICE_UUID[5],
        //     MOTOR_VIS_SERVICE_UUID[6], MOTOR_VIS_SERVICE_UUID[7],
        //     MOTOR_VIS_SERVICE_UUID[8], MOTOR_VIS_SERVICE_UUID[9],
        //     MOTOR_VIS_SERVICE_UUID[10], MOTOR_VIS_SERVICE_UUID[11], MOTOR_VIS_SERVICE_UUID[12], MOTOR_VIS_SERVICE_UUID[13],
        //     MOTOR_VIS_SERVICE_UUID[14], MOTOR_VIS_SERVICE_UUID[15]
        // );

        err = esp_ble_gap_config_adv_data(&scan_response_data);
        if (err != ESP_OK) {
            motor_vis_report_error("configure scan response data", err);
            return;
        }

        // Defines the service ID for the primary service to be created, including its UUID and instance ID, and indicates that it is a primary service
        esp_gatt_srvc_id_t service_id =  {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {
                    .len = ESP_UUID_LEN_128,
                },
            },
        };
        
        memcpy(service_id.id.uuid.uuid.uuid128, MOTOR_VIS_SERVICE_UUID, sizeof(MOTOR_VIS_SERVICE_UUID));
        
        // Creates the primary service with the defined service ID and specified number of handles, and checks for errors during service creation
        err = esp_ble_gatts_create_service(gatts_if, &service_id, MOTOR_VIS_NUM_HANDLES);
        if (err != ESP_OK) {
            motor_vis_report_error("create service", err);
        }
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        esp_err_t err;
        esp_bt_uuid_t characteristic_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = MOTOR_VIS_CHARACTERISTIC_UUID,
            },
        };
        esp_gatt_char_prop_t properties =
            ESP_GATT_CHAR_PROP_BIT_READ |
            ESP_GATT_CHAR_PROP_BIT_WRITE;

        motor_vis_service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "service created, handle=%d", motor_vis_service_handle);

        err = esp_ble_gatts_start_service(motor_vis_service_handle);
        if (err != ESP_OK) {
            motor_vis_report_error("start service", err);
            return;
        }

        err = esp_ble_gatts_add_char(
            motor_vis_service_handle,
            &characteristic_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            properties,
            &motor_vis_attribute_value,
            NULL
        );
        if (err != ESP_OK) {
            motor_vis_report_error("add characteristic", err);
        }
        break;
    }

    // Handles the event when a characteristic is added, stores the handle for the characteristic and logs its addition
    case ESP_GATTS_ADD_CHAR_EVT:
        motor_vis_characteristic_handle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "characteristic added, handle=%d uuid=0x%04x",
            motor_vis_characteristic_handle,
            param->add_char.char_uuid.uuid.uuid16);
        break;

    // Handles read requests from the client, logs the request and calls the function to handle the read event
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "read request on handle=%d", param->read.handle);
        handle_read_event(gatts_if, param);
        break;

    // Handles write requests from the client, logs the request and calls the function to handle the write event
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "write request on handle=%d len=%d", param->write.handle, param->write.len);
        handle_write_event(gatts_if, param);
        break;

    // Handles connection events, updates the connection parameters, and manages the advertising state and LED state based on whether a phone is connected or disconnected
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t connection_params = {0};

        motor_vis_connected = true;
        motor_vis_advertising = false;
        motor_vis_connection_id = param->connect.conn_id;

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

        esp_ble_gap_update_conn_params(&connection_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        motor_vis_connected = false;
        ESP_LOGI(
            TAG,
            "phone disconnected, reason=0x%02x remote=" ESP_BD_ADDR_STR,
            param->disconnect.reason,
            ESP_BD_ADDR_HEX(param->disconnect.remote_bda)
        );
        motor_vis_advertising = false;
        motor_vis_set_led_state(MOTOR_VIS_LED_STATE_ADVERTISING);
        start_advertising("disconnect");
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "mtu updated to %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}


// Main App Init Function
void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "booting MotoVis Feather BLE test firmware"); //LOG
    // Initialize status LED first so we can indicate errors during setup
    err = motor_vis_status_led_init();
    // If LED initialization fails, we can't indicate the error state, so we log it and stop
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "status NeoPixel init failed: %s", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /**Create FreeRTOS task to manage the LED state machine
    * - motor_vis_status_led_task will run in an infinite loop, updating the LED based on the current state 
    * - "motor_vis_led" is the name of the task for debugging purposes
    * - 2048 is the stack size allocated for the task, which should be sufficient for its needs
    * - NULL is the parameter passed to the task, which is not used in this case
    * - 3 is the priority of the task, which is set to a moderate level to ensure it runs smoothly without starving other tasks
    * - NULL is the task handle, which is not needed since we don't need to reference the task after creation
    */
   // TODO: Find optimal stack size and priority for the LED task, as it may need to be adjusted based on actual performance and responsiveness requirements
    xTaskCreate(motor_vis_status_led_task, "motor_vis_led", 2048, NULL, 3, NULL);
    // Set initial LED state to booting while we initialize the system
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_BOOTING);

    // Initialize NVS, which is required for BLE to store bonding information and other data
    err = nvs_flash_init();
    // If NVS initialization fails due to no free pages or a new version, it is erased and tried again
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // If NVS initialization still fails, we log the error and set the LED to error state
    if (err != ESP_OK) {
        motor_vis_report_error("initialize NVS", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initialize Bluetooth controller with default configuration
    motor_vis_set_led_state(MOTOR_VIS_LED_STATE_BLE_INITIALIZING);
    // Since we only need BLE functionality, we can release memory allocated for Classic Bluetooth
    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    // If memory release fails, we log the error and set the LED to error state
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

    // Initialize Bluedroid 
    esp_bluedroid_config_t bluedroid_config = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    err = esp_bluedroid_init_with_cfg(&bluedroid_config);
    if (err != ESP_OK) {
        motor_vis_report_error("initialize Bluedroid", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Enable Bluedroid
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        motor_vis_report_error("enable Bluedroid", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Register GAP and GATTS event handlers
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        motor_vis_report_error("register GAP callback", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Registered GAP and GATTS event handlers");

    //TODO: Error gets thrown around this area
    // Register GATTS callback to handle GATT server events such as read/write requests and connection events
    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) {
        motor_vis_report_error("register GATTS callback", err);

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Register the GATT application with the Bluetooth stack
    err = esp_ble_gatts_app_register(MOTOR_VIS_APP_ID);
    if (err != ESP_OK) {
        motor_vis_report_error("register GATT app", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Set the local MTU to the maximum allowed value to support larger characteristic values and improve performance
    err = esp_ble_gatt_set_local_mtu(247);
    if (err != ESP_OK) {
        motor_vis_report_error("set local MTU", err);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    motor_vis_ble_ready = true;

    // Main loop to monitor BLE state and restart advertising if needed
    ESP_LOGI(TAG, "waiting for LightBlue connection");

    while (true) {
        // if (motor_vis_ble_ready &&
        //     motor_vis_adv_data_configured &&
        //     !motor_vis_connected &&
        //     !motor_vis_advertising) {
        if (motor_vis_is_advertising_ready() &&
            !motor_vis_connected &&
            !motor_vis_advertising) {
            ESP_LOGW(TAG, "no active BLE connection or advertisement, retrying advertising");
            start_advertising("supervisor retry");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
