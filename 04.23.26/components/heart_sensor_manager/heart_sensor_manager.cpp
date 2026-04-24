// heart_sensor_manager.cpp - Manages the PulseSensor Playground library and provides beat samples to the main GATT server through a simple C API.

#include <cstring>

// C headers are wrapped in extern "C" so the C++ manager can call ESP-IDF and FreeRTOS APIs cleanly.
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

// Arduino and PulseSensor Playground stay private to this component so Motor_Vis.c remains a native C GATT server.
#include "Arduino.h"
#include "PulseSensorPlayground.h"
#include "heart_sensor_manager.h"

// --------------------------------------- HEART SENSOR CONFIGURATION ---------------------------------------
// Heart manager logging tag is intentionally used only for initialization and manager health messages.
static const char *TAG = "HEART_MANAGER";

// PulseSensor Amped wiring and threshold come from the validated Arduino sketch used as the reference source.
#define MOTOR_VIS_HEART_SENSOR_PIN              34
#define MOTOR_VIS_HEART_SENSOR_THRESHOLD        550
#define MOTOR_VIS_HEART_POLL_MS                 10
#define MOTOR_VIS_HEART_TASK_STACK_SIZE         4096
#define MOTOR_VIS_HEART_TASK_PRIORITY           4

// --------------------------------------- HEART SENSOR SHARED STATE ---------------------------------------
// Official PulseSensor Playground object owns the beat detection algorithm and raw analog sampling.
static PulseSensorPlayground s_pulse_sensor;

// Mutex protects the latest beat sample while the Arduino-backed task and C BLE code access it concurrently.
static SemaphoreHandle_t s_sample_mutex = NULL;
static TaskHandle_t s_heart_task_handle = NULL;
static bool s_heart_initialized = false;
static motor_vis_heart_sample_t s_latest_sample = {};
static uint32_t s_detected_beat_count = 0;

// Helper keeps heart sample timestamps in the same millisecond time base used by the other MotoVis packets.
static uint32_t motor_vis_heart_now_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Converts millisecond timing constants into FreeRTOS ticks without allowing short waits to become a busy loop.
static TickType_t motor_vis_heart_ms_to_ticks_at_least_one(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);

    if (ticks == 0) {
        return 1;
    }

    return ticks;
}

// Stores one beat snapshot while holding the mutex so BLE readers always get a complete packet source.
static void motor_vis_heart_store_sample(uint16_t bpm, uint16_t raw_adc, uint8_t bpm_valid)
{
    if (s_sample_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_sample_mutex, portMAX_DELAY) == pdTRUE) {
        s_latest_sample.timestamp_ms = motor_vis_heart_now_ms();
        s_latest_sample.bpm = bpm;
        s_latest_sample.raw_adc = raw_adc;
        s_latest_sample.bpm_valid = bpm_valid;
        s_latest_sample.sequence += 1U;
        xSemaphoreGive(s_sample_mutex);
    }
}

// Polls the PulseSensor library for beat events and stores only new beat snapshots for the main GATT server.
static void motor_vis_heart_sensor_task(void *parameter)
{
    const TickType_t poll_delay_ticks = motor_vis_heart_ms_to_ticks_at_least_one(MOTOR_VIS_HEART_POLL_MS);

    while (true) {
        if (s_pulse_sensor.sawStartOfBeat()) {
            int bpm = s_pulse_sensor.getBeatsPerMinute();
            int raw_adc = s_pulse_sensor.getLatestSample();

            if (bpm < 0) {
                bpm = 0;
            }

            if (raw_adc < 0) {
                raw_adc = 0;
            }

            // The first detected beat does not have a previous beat interval, so it is stored but marked not yet valid.
            uint8_t bpm_valid = (s_detected_beat_count > 0U && bpm > 0) ? 1U : 0U;
            s_detected_beat_count += 1U;

            motor_vis_heart_store_sample(
                static_cast<uint16_t>(bpm),
                static_cast<uint16_t>(raw_adc),
                bpm_valid
            );
        }

        vTaskDelay(poll_delay_ticks);
    }
}

extern "C" esp_err_t motor_vis_heart_sensor_init(void)
{
    if (s_heart_initialized) {
        // Repeated init requests are harmless because the manager has only one sensor and one task.
        return ESP_OK;
    }

    s_sample_mutex = xSemaphoreCreateMutex();
    if (s_sample_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create heart sample mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initializes the Arduino runtime before the PulseSensor library touches analogRead, timers, or Arduino timing helpers.
    initArduino();

    s_pulse_sensor.analogInput(MOTOR_VIS_HEART_SENSOR_PIN);
    s_pulse_sensor.setThreshold(MOTOR_VIS_HEART_SENSOR_THRESHOLD);

    if (!s_pulse_sensor.begin()) {
        ESP_LOGE(TAG, "PulseSensor Playground failed to start on GPIO%d", MOTOR_VIS_HEART_SENSOR_PIN);
        vSemaphoreDelete(s_sample_mutex);
        s_sample_mutex = NULL;
        return ESP_FAIL;
    }

    BaseType_t task_result = xTaskCreate(
        motor_vis_heart_sensor_task,
        "heart_sensor",
        MOTOR_VIS_HEART_TASK_STACK_SIZE,
        NULL,
        MOTOR_VIS_HEART_TASK_PRIORITY,
        &s_heart_task_handle
    );
    if (task_result != pdPASS) {
        s_pulse_sensor.pause();
        vSemaphoreDelete(s_sample_mutex);
        s_sample_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_heart_initialized = true;
    ESP_LOGI(
        TAG,
        "PulseSensor Playground initialized on GPIO%d threshold=%d",
        MOTOR_VIS_HEART_SENSOR_PIN,
        MOTOR_VIS_HEART_SENSOR_THRESHOLD
    );
    return ESP_OK;
}

extern "C" bool motor_vis_heart_sensor_get_latest_sample(motor_vis_heart_sample_t *out_sample)
{
    if (!s_heart_initialized || out_sample == NULL || s_sample_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_sample_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    // Copies the full sample while locked so the main GATT server never sees a partially updated beat packet.
    std::memcpy(out_sample, &s_latest_sample, sizeof(*out_sample));
    xSemaphoreGive(s_sample_mutex);
    return true;
}
