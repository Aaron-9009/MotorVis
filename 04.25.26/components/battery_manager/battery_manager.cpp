#include <cstring>

// C headers are wrapped in extern "C" so this Arduino-backed manager can still use ESP-IDF and FreeRTOS APIs cleanly.
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

// Arduino stays private to this component so Motor_Vis.c can publish battery data without owning ADC setup.
#include "Arduino.h"
#include "battery_manager.h"

// --------------------------------------- BATTERY ADC CONFIGURATION ---------------------------------------
// Battery manager logging tag for ADC setup and bring-up messages.
static const char *TAG = "BATTERY_MANAGER";

// The Adafruit ESP32 Feather V2 routes the battery voltage divider to GPIO35.
#define MOTOR_VIS_BATTERY_ADC_PIN               35
#define MOTOR_VIS_BATTERY_ADC_RESOLUTION_BITS   12
#define MOTOR_VIS_BATTERY_ADC_MAX_RAW           4095
#define MOTOR_VIS_BATTERY_DIVIDER_MULTIPLIER    2
#define MOTOR_VIS_BATTERY_SAMPLE_INTERVAL_MS    5000
#define MOTOR_VIS_BATTERY_TASK_STACK_SIZE       3072
#define MOTOR_VIS_BATTERY_TASK_PRIORITY         4

// Voltage bounds keep the BLE packet from reporting floating or clearly impossible battery readings as valid.
#define MOTOR_VIS_BATTERY_MIN_VALID_MV          2500
#define MOTOR_VIS_BATTERY_MAX_VALID_MV          5000
#define MOTOR_VIS_BATTERY_EMPTY_MV              3300
#define MOTOR_VIS_BATTERY_FULL_MV               4200

// --------------------------------------- BATTERY SHARED STATE ---------------------------------------
// Mutex protects the latest battery sample while the sampling task and C BLE code access it concurrently.
static SemaphoreHandle_t s_battery_mutex = NULL;
static TaskHandle_t s_battery_task_handle = NULL;
static bool s_battery_initialized = false;
static motor_vis_battery_sample_t s_latest_sample = {};

// Helper keeps all battery timestamps in the same millisecond time base used by the other BLE packets.
static uint32_t motor_vis_battery_now_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Converts millisecond timing constants into FreeRTOS ticks without allowing short waits to become a busy loop.
static TickType_t motor_vis_battery_ms_to_ticks_at_least_one(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);

    if (ticks == 0) {
        return 1;
    }

    return ticks;
}

// Converts battery voltage into an app-friendly LiPo percentage estimate.
static uint8_t motor_vis_battery_percent_from_mv(uint16_t voltage_mv)
{
    typedef struct {
        uint16_t mv;
        uint8_t percent;
    } motor_vis_battery_curve_point_t;

    static const motor_vis_battery_curve_point_t curve[] = {
        {4200, 100},
        {4110, 90},
        {4020, 80},
        {3950, 70},
        {3870, 60},
        {3790, 50},
        {3740, 40},
        {3680, 30},
        {3600, 20},
        {3500, 10},
        {3300, 0},
    };

    if (voltage_mv >= MOTOR_VIS_BATTERY_FULL_MV) {
        return 100;
    }

    if (voltage_mv <= MOTOR_VIS_BATTERY_EMPTY_MV) {
        return 0;
    }

    for (size_t i = 0; i < ((sizeof(curve) / sizeof(curve[0])) - 1U); i++) {
        uint16_t high_mv = curve[i].mv;
        uint16_t low_mv = curve[i + 1U].mv;

        if (voltage_mv <= high_mv && voltage_mv >= low_mv) {
            uint8_t high_percent = curve[i].percent;
            uint8_t low_percent = curve[i + 1U].percent;
            uint16_t mv_span = high_mv - low_mv;
            uint16_t mv_offset = voltage_mv - low_mv;
            uint8_t percent_span = high_percent - low_percent;

            return static_cast<uint8_t>(low_percent + ((mv_offset * percent_span) / mv_span));
        }
    }

    return 0;
}

// Stores one battery sample behind the mutex so BLE readers never see a partially updated packet.
static bool motor_vis_battery_store_sample(uint16_t voltage_mv, uint16_t raw_adc, uint8_t battery_valid)
{
    if (s_battery_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_battery_mutex, portMAX_DELAY) == pdTRUE) {
        s_latest_sample.timestamp_ms = motor_vis_battery_now_ms();
        s_latest_sample.voltage_mv = voltage_mv;
        s_latest_sample.raw_adc = raw_adc;
        s_latest_sample.battery_valid = battery_valid ? 1U : 0U;
        s_latest_sample.percentage = battery_valid ? motor_vis_battery_percent_from_mv(voltage_mv) : 0U;
        s_latest_sample.sequence += 1U;
        xSemaphoreGive(s_battery_mutex);
        return true;
    }

    return false;
}

// Reads the Feather V2 battery monitor and converts the divided ADC pin voltage back into battery millivolts.
static esp_err_t motor_vis_battery_read_once(uint16_t *out_voltage_mv, uint16_t *out_raw_adc, uint8_t *out_valid)
{
    uint16_t raw_adc = analogRead(MOTOR_VIS_BATTERY_ADC_PIN);
    uint32_t pin_voltage_mv = analogReadMilliVolts(MOTOR_VIS_BATTERY_ADC_PIN);

    if (raw_adc > MOTOR_VIS_BATTERY_ADC_MAX_RAW) {
        raw_adc = MOTOR_VIS_BATTERY_ADC_MAX_RAW;
    }

    uint32_t battery_voltage_mv = pin_voltage_mv * MOTOR_VIS_BATTERY_DIVIDER_MULTIPLIER;
    if (battery_voltage_mv > UINT16_MAX) {
        battery_voltage_mv = UINT16_MAX;
    }

    *out_voltage_mv = static_cast<uint16_t>(battery_voltage_mv);
    *out_raw_adc = raw_adc;
    *out_valid = (battery_voltage_mv >= MOTOR_VIS_BATTERY_MIN_VALID_MV &&
                  battery_voltage_mv <= MOTOR_VIS_BATTERY_MAX_VALID_MV) ? 1U : 0U;

    return ESP_OK;
}

// Battery task samples slowly because jacket battery level changes gradually and does not need high-rate BLE updates.
static void motor_vis_battery_task(void *arg)
{
    const TickType_t sample_delay_ticks = motor_vis_battery_ms_to_ticks_at_least_one(MOTOR_VIS_BATTERY_SAMPLE_INTERVAL_MS);

    while (true) {
        uint16_t voltage_mv = 0;
        uint16_t raw_adc = 0;
        uint8_t battery_valid = 0;
        esp_err_t err = motor_vis_battery_read_once(&voltage_mv, &raw_adc, &battery_valid);

        if (err == ESP_OK) {
            motor_vis_battery_store_sample(voltage_mv, raw_adc, battery_valid);
        } else {
            ESP_LOGW(TAG, "battery ADC read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(sample_delay_ticks);
    }
}

extern "C" esp_err_t motor_vis_battery_manager_init(void)
{
    if (s_battery_initialized) {
        // Repeated init requests are harmless because the manager has only one ADC pin and one task.
        return ESP_OK;
    }

    s_battery_mutex = xSemaphoreCreateMutex();
    if (s_battery_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create battery sample mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initializes Arduino before using its analog layer, which also keeps ADC ownership aligned with the PulseSensor manager.
    initArduino();
    analogReadResolution(MOTOR_VIS_BATTERY_ADC_RESOLUTION_BITS);
    analogSetPinAttenuation(MOTOR_VIS_BATTERY_ADC_PIN, ADC_11db);

    BaseType_t task_result = xTaskCreate(
        motor_vis_battery_task,
        "motor_vis_battery",
        MOTOR_VIS_BATTERY_TASK_STACK_SIZE,
        NULL,
        MOTOR_VIS_BATTERY_TASK_PRIORITY,
        &s_battery_task_handle
    );
    if (task_result != pdPASS) {
        vSemaphoreDelete(s_battery_mutex);
        s_battery_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_battery_initialized = true;
    ESP_LOGI(TAG, "battery manager initialized on GPIO%d", MOTOR_VIS_BATTERY_ADC_PIN);
    return ESP_OK;
}

extern "C" bool motor_vis_battery_manager_get_latest_sample(motor_vis_battery_sample_t *out_sample)
{
    if (!s_battery_initialized || out_sample == NULL || s_battery_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_battery_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    // Copies the full sample while locked so the main GATT server never sees a partially updated battery packet.
    std::memcpy(out_sample, &s_latest_sample, sizeof(*out_sample));
    xSemaphoreGive(s_battery_mutex);
    return true;
}
