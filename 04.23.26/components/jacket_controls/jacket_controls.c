// jacket_controls.c - Button and external LED manager for MotoVis jacket controls
//
// Owns the discrete pairing and alert buttons, the temporary power button log input,
// and the pairing/alert external LEDs.
// The component debounces button presses, queues button events for the main BLE
// firmware, and drives LED modes in a dedicated task so the hardware validation
// layer stays separated from the GATT server logic in Motor_Vis.c.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ---- ESP-IDF and FreeRTOS headers ----

// C module uses FreeRTOS queues, tasks, and mutexes to move button events safely across firmware sections.
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// GPIO driver controls the external buttons and discrete LEDs connected to the Feather.
#include "driver/gpio.h"

// Logging, error helpers, and timestamps support hardware bring-up and event sequencing.
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "jacket_controls.h"

// --------------------------------------- MODULE CONSTANTS ---------------------------------------
// Log tag used to separate jacket-control messages from BLE and GNSS logs.
static const char *TAG = "JACKET_CONTROLS";

// GPIO layout for the external pairing, alert, and temporary power-button hardware used during bring-up.
#define JACKET_CONTROL_GPIO_PAIRING_BUTTON         GPIO_NUM_4
#define JACKET_CONTROL_GPIO_PAIRING_LED            GPIO_NUM_12
#define JACKET_CONTROL_GPIO_ALERT_BUTTON           GPIO_NUM_27
#define JACKET_CONTROL_GPIO_ALERT_LED              GPIO_NUM_14
// Temporary power button input only; GPIO13 power LED remains intentionally unconfigured in this phase.
#define JACKET_CONTROL_GPIO_POWER_BUTTON           GPIO_NUM_5

// Timing constants define the software debounce window and LED blink periods for this validation phase.
#define JACKET_CONTROL_BUTTON_POLL_MS              10
// Four matching samples at a 6 ms poll interval guarantees the new state stays stable for at least 30 ms before acceptance.
#define JACKET_CONTROL_DEBOUNCE_SAMPLES            4
#define JACKET_CONTROL_LED_TICK_MS                 50
#define JACKET_CONTROL_LED_BLINK_SLOW_MS           500
#define JACKET_CONTROL_LED_BLINK_FAST_MS           150

// Queue and task sizing values are intentionally small because the component only manages three buttons and two LEDs.
#define JACKET_CONTROL_EVENT_QUEUE_LENGTH          8
#define JACKET_CONTROL_BUTTON_TASK_STACK_SIZE      3072
#define JACKET_CONTROL_LED_TASK_STACK_SIZE         2048
#define JACKET_CONTROL_BUTTON_TASK_PRIORITY        4
#define JACKET_CONTROL_LED_TASK_PRIORITY           3

// --------------------------------------- INTERNAL STATE TYPES ---------------------------------------
// Tracks the debounced state of one hardware button across repeated polling samples.
typedef struct {
    jacket_control_button_id_t button_id;
    gpio_num_t gpio_num;
    int stable_level;
    int last_sample_level;
    uint8_t stable_sample_count;
} jacket_control_button_state_t;

// Tracks the desired and current state of one external LED managed by the component.
typedef struct {
    gpio_num_t gpio_num;
    jacket_control_led_mode_t mode;
    bool current_output_level;
    uint32_t last_toggle_ms;
    bool sequence_active;
    uint32_t sequence_remaining_toggles;
    uint32_t sequence_toggle_period_ms;
    jacket_control_led_mode_t sequence_final_mode;
} jacket_control_led_state_t;

// --------------------------------------- MODULE RUNTIME STATE ---------------------------------------
// Stores the debounced buttons managed by the jacket controls component.
static jacket_control_button_state_t s_button_states[] = {
    {
        .button_id = JACKET_CONTROL_BUTTON_PAIRING,
        .gpio_num = JACKET_CONTROL_GPIO_PAIRING_BUTTON,
        .stable_level = 0,
        .last_sample_level = 0,
        .stable_sample_count = 0,
    },
    {
        .button_id = JACKET_CONTROL_BUTTON_ALERT_CANCEL,
        .gpio_num = JACKET_CONTROL_GPIO_ALERT_BUTTON,
        .stable_level = 0,
        .last_sample_level = 0,
        .stable_sample_count = 0,
    },
    {
        .button_id = JACKET_CONTROL_BUTTON_POWER,
        .gpio_num = JACKET_CONTROL_GPIO_POWER_BUTTON,
        .stable_level = 0,
        .last_sample_level = 0,
        .stable_sample_count = 0,
    },
};

// Stores the two external LEDs managed by the jacket controls component.
static jacket_control_led_state_t s_led_states[] = {
    {
        .gpio_num = JACKET_CONTROL_GPIO_PAIRING_LED,
        .mode = JACKET_CONTROL_LED_OFF,
        .current_output_level = false,
        .last_toggle_ms = 0,
        .sequence_active = false,
        .sequence_remaining_toggles = 0,
        .sequence_toggle_period_ms = 0,
        .sequence_final_mode = JACKET_CONTROL_LED_OFF,
    },
    {
        .gpio_num = JACKET_CONTROL_GPIO_ALERT_LED,
        .mode = JACKET_CONTROL_LED_OFF,
        .current_output_level = false,
        .last_toggle_ms = 0,
        .sequence_active = false,
        .sequence_remaining_toggles = 0,
        .sequence_toggle_period_ms = 0,
        .sequence_final_mode = JACKET_CONTROL_LED_OFF,
    },
};

// Queue carries debounced button-press events into the main BLE firmware without blocking the poll task.
static QueueHandle_t s_event_queue = NULL;
// Mutex protects shared LED mode state when the LED task and main firmware both access it.
static SemaphoreHandle_t s_led_mutex = NULL;
// Task handles are stored so startup failures can unwind cleanly if a later step fails.
static TaskHandle_t s_button_task_handle = NULL;
static TaskHandle_t s_led_task_handle = NULL;
// Tracks whether the component completed its one-time initialization successfully.
static bool s_controls_initialized = false;
// Monotonic event sequence increments every time a debounced active-low button press is accepted.
static uint32_t s_button_event_sequence = 0;

// --------------------------------------- FORWARD DECLARATIONS ---------------------------------------
static uint32_t jacket_controls_now_ms(void);
static bool jacket_controls_is_valid_led_id(jacket_control_led_id_t led_id);
static jacket_control_button_state_t *jacket_controls_find_button_state(jacket_control_button_id_t button_id);
static uint32_t jacket_controls_get_blink_period_ms(jacket_control_led_mode_t mode);
static void jacket_controls_button_task(void *parameter);
static void jacket_controls_led_task(void *parameter);
static void jacket_controls_initialize_button_samples(void);
static void jacket_controls_apply_led_output(jacket_control_led_id_t led_id, bool level);
static void jacket_controls_emit_button_event(jacket_control_button_id_t button_id);
static esp_err_t jacket_controls_configure_button_gpios(void);
static esp_err_t jacket_controls_configure_led_gpios(void);
static void jacket_controls_cleanup_failed_init(void);

// Returns the current millisecond timestamp used in button events and LED blink timing.
static uint32_t jacket_controls_now_ms(void)
{
    return (uint32_t) (esp_timer_get_time() / 1000ULL);
}

// Validates the caller-provided LED identifier before the component touches its state arrays.
static bool jacket_controls_is_valid_led_id(jacket_control_led_id_t led_id)
{
    return led_id >= JACKET_CONTROL_LED_PAIRING &&
        led_id <= JACKET_CONTROL_LED_ALERT;
}

// Finds the state record for one button so helper APIs can inspect or re-synchronize the same debounced state machine used by the poll task.
static jacket_control_button_state_t *jacket_controls_find_button_state(jacket_control_button_id_t button_id)
{
    size_t button_count = sizeof(s_button_states) / sizeof(s_button_states[0]);

    for (size_t i = 0; i < button_count; i++) {
        if (s_button_states[i].button_id == button_id) {
            return &s_button_states[i];
        }
    }

    return NULL;
}

// Maps each LED mode to the blink period used by the LED task.
static uint32_t jacket_controls_get_blink_period_ms(jacket_control_led_mode_t mode)
{
    switch (mode) {
    case JACKET_CONTROL_LED_BLINK_SLOW:
        return JACKET_CONTROL_LED_BLINK_SLOW_MS;

    case JACKET_CONTROL_LED_BLINK_FAST:
        return JACKET_CONTROL_LED_BLINK_FAST_MS;

    default:
        return 0;
    }
}

// Captures the initial raw button levels so startup does not accidentally emit a false press event.
static void jacket_controls_initialize_button_samples(void)
{
    size_t button_count = sizeof(s_button_states) / sizeof(s_button_states[0]);

    for (size_t i = 0; i < button_count; i++) {
        int initial_level = gpio_get_level(s_button_states[i].gpio_num);
        s_button_states[i].stable_level = initial_level;
        s_button_states[i].last_sample_level = initial_level;
        s_button_states[i].stable_sample_count = 1;
    }
}

// Drives one external LED to a concrete output level using the configured active-high assumption.
static void jacket_controls_apply_led_output(jacket_control_led_id_t led_id, bool level)
{
    gpio_set_level(s_led_states[led_id].gpio_num, level ? 1 : 0);
}

// Queues one debounced button press event for the main firmware without blocking the polling task.
static void jacket_controls_emit_button_event(jacket_control_button_id_t button_id)
{
    jacket_control_button_event_t event = {
        .button_id = button_id,
        .timestamp_ms = jacket_controls_now_ms(),
        .sequence = ++s_button_event_sequence,
    };

    if (s_event_queue == NULL) {
        return;
    }

    if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "button event queue full, dropping event for button=%d", (int) button_id);
    }
}

// Configures the discrete pairing, alert, and temporary power buttons as input-only GPIOs with internal pull-up resistors.
static esp_err_t jacket_controls_configure_button_gpios(void)
{
    gpio_config_t button_config = {
        .pin_bit_mask =
            (1ULL << JACKET_CONTROL_GPIO_PAIRING_BUTTON) |
            (1ULL << JACKET_CONTROL_GPIO_ALERT_BUTTON) |
            (1ULL << JACKET_CONTROL_GPIO_POWER_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&button_config);
}

// Configures the discrete pairing and alert LEDs as active-high output GPIOs and forces them off initially.
static esp_err_t jacket_controls_configure_led_gpios(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask =
            (1ULL << JACKET_CONTROL_GPIO_PAIRING_LED) |
            (1ULL << JACKET_CONTROL_GPIO_ALERT_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&led_config);
    if (err != ESP_OK) {
        return err;
    }

    for (int led_id = JACKET_CONTROL_LED_PAIRING; led_id <= JACKET_CONTROL_LED_ALERT; led_id++) {
        jacket_controls_apply_led_output((jacket_control_led_id_t) led_id, false);
        s_led_states[led_id].mode = JACKET_CONTROL_LED_OFF;
        s_led_states[led_id].current_output_level = false;
        s_led_states[led_id].last_toggle_ms = jacket_controls_now_ms();
        s_led_states[led_id].sequence_active = false;
        s_led_states[led_id].sequence_remaining_toggles = 0;
        s_led_states[led_id].sequence_toggle_period_ms = 0;
        s_led_states[led_id].sequence_final_mode = JACKET_CONTROL_LED_OFF;
    }

    return ESP_OK;
}

// Tears down partially created runtime resources when initialization fails before the component is fully usable.
static void jacket_controls_cleanup_failed_init(void)
{
    if (s_button_task_handle != NULL) {
        vTaskDelete(s_button_task_handle);
        s_button_task_handle = NULL;
    }

    if (s_led_task_handle != NULL) {
        vTaskDelete(s_led_task_handle);
        s_led_task_handle = NULL;
    }

    if (s_event_queue != NULL) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }

    if (s_led_mutex != NULL) {
        vSemaphoreDelete(s_led_mutex);
        s_led_mutex = NULL;
    }
}

// Polls each external button, applies debounce, and emits one event for each accepted active-low press.
static void jacket_controls_button_task(void *parameter)
{
    size_t button_count = sizeof(s_button_states) / sizeof(s_button_states[0]);

    while (true) {
        for (size_t i = 0; i < button_count; i++) {
            int raw_level = gpio_get_level(s_button_states[i].gpio_num);

            if (raw_level == s_button_states[i].last_sample_level) {
                if (s_button_states[i].stable_sample_count < UINT8_MAX) {
                    s_button_states[i].stable_sample_count++;
                }
            } else {
                s_button_states[i].last_sample_level = raw_level;
                s_button_states[i].stable_sample_count = 1;
            }

            if (s_button_states[i].stable_sample_count >= JACKET_CONTROL_DEBOUNCE_SAMPLES &&
                s_button_states[i].stable_level != raw_level) {
                s_button_states[i].stable_level = raw_level;

                if (raw_level == 0) {
                    jacket_controls_emit_button_event(s_button_states[i].button_id);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(JACKET_CONTROL_BUTTON_POLL_MS));
    }
}

// Periodically applies the requested LED modes so external indicators blink independently of the main BLE loop.
static void jacket_controls_led_task(void *parameter)
{
    while (true) {
        uint32_t now_ms = jacket_controls_now_ms();

        if (xSemaphoreTake(s_led_mutex, portMAX_DELAY) == pdTRUE) {
            for (int led_id = JACKET_CONTROL_LED_PAIRING; led_id <= JACKET_CONTROL_LED_ALERT; led_id++) {
                uint32_t blink_period_ms = jacket_controls_get_blink_period_ms(s_led_states[led_id].mode);

                // A short finite sequence can temporarily override the steady LED mode so the main firmware can signal events like successful pairing.
                if (s_led_states[led_id].sequence_active) {
                    if ((now_ms - s_led_states[led_id].last_toggle_ms) >= s_led_states[led_id].sequence_toggle_period_ms) {
                        s_led_states[led_id].current_output_level = !s_led_states[led_id].current_output_level;
                        s_led_states[led_id].last_toggle_ms = now_ms;

                        if (s_led_states[led_id].sequence_remaining_toggles > 0) {
                            s_led_states[led_id].sequence_remaining_toggles--;
                        }

                        if (s_led_states[led_id].sequence_remaining_toggles == 0) {
                            s_led_states[led_id].sequence_active = false;
                            s_led_states[led_id].mode = s_led_states[led_id].sequence_final_mode;

                            switch (s_led_states[led_id].mode) {
                            case JACKET_CONTROL_LED_OFF:
                                s_led_states[led_id].current_output_level = false;
                                break;

                            case JACKET_CONTROL_LED_ON:
                                s_led_states[led_id].current_output_level = true;
                                break;

                            case JACKET_CONTROL_LED_BLINK_SLOW:
                            case JACKET_CONTROL_LED_BLINK_FAST:
                                s_led_states[led_id].current_output_level = true;
                                s_led_states[led_id].last_toggle_ms = now_ms;
                                break;

                            default:
                                s_led_states[led_id].mode = JACKET_CONTROL_LED_OFF;
                                s_led_states[led_id].current_output_level = false;
                                break;
                            }
                        }
                    }
                } else {
                    switch (s_led_states[led_id].mode) {
                    case JACKET_CONTROL_LED_OFF:
                        s_led_states[led_id].current_output_level = false;
                        break;

                    case JACKET_CONTROL_LED_ON:
                        s_led_states[led_id].current_output_level = true;
                        break;

                    case JACKET_CONTROL_LED_BLINK_SLOW:
                    case JACKET_CONTROL_LED_BLINK_FAST:
                        if ((now_ms - s_led_states[led_id].last_toggle_ms) >= blink_period_ms) {
                            s_led_states[led_id].current_output_level = !s_led_states[led_id].current_output_level;
                            s_led_states[led_id].last_toggle_ms = now_ms;
                        }
                        break;

                    default:
                        s_led_states[led_id].mode = JACKET_CONTROL_LED_OFF;
                        s_led_states[led_id].current_output_level = false;
                        break;
                    }
                }

                jacket_controls_apply_led_output(
                    (jacket_control_led_id_t) led_id,
                    s_led_states[led_id].current_output_level
                );
            }

            xSemaphoreGive(s_led_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(JACKET_CONTROL_LED_TICK_MS));
    }
}

// Performs one-time button and LED setup and starts the background tasks that own the controls hardware.
esp_err_t jacket_controls_init(void)
{
    if (s_controls_initialized) {
        return ESP_OK;
    }

    s_event_queue = xQueueCreate(JACKET_CONTROL_EVENT_QUEUE_LENGTH, sizeof(jacket_control_button_event_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_led_mutex = xSemaphoreCreateMutex();
    if (s_led_mutex == NULL) {
        jacket_controls_cleanup_failed_init();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = jacket_controls_configure_button_gpios();
    if (err != ESP_OK) {
        jacket_controls_cleanup_failed_init();
        return err;
    }

    err = jacket_controls_configure_led_gpios();
    if (err != ESP_OK) {
        jacket_controls_cleanup_failed_init();
        return err;
    }

    jacket_controls_initialize_button_samples();

    BaseType_t task_result = xTaskCreate(
        jacket_controls_button_task,
        "jacket_buttons",
        JACKET_CONTROL_BUTTON_TASK_STACK_SIZE,
        NULL,
        JACKET_CONTROL_BUTTON_TASK_PRIORITY,
        &s_button_task_handle
    );
    if (task_result != pdPASS) {
        jacket_controls_cleanup_failed_init();
        return ESP_ERR_NO_MEM;
    }

    task_result = xTaskCreate(
        jacket_controls_led_task,
        "jacket_leds",
        JACKET_CONTROL_LED_TASK_STACK_SIZE,
        NULL,
        JACKET_CONTROL_LED_TASK_PRIORITY,
        &s_led_task_handle
    );
    if (task_result != pdPASS) {
        jacket_controls_cleanup_failed_init();
        return ESP_ERR_NO_MEM;
    }

    s_controls_initialized = true;
    ESP_LOGI(TAG, "jacket controls initialized for pairing, alert, and temporary power-button hardware");
    return ESP_OK;
}

// Returns the next queued button event without blocking so the main BLE loop can drain all pending control activity.
bool jacket_controls_get_next_event(jacket_control_button_event_t *out_event)
{
    if (!s_controls_initialized || out_event == NULL || s_event_queue == NULL) {
        return false;
    }

    return xQueueReceive(s_event_queue, out_event, 0) == pdTRUE;
}

// Reads the raw GPIO level for one button and converts it into the component's active-low pressed/not-pressed meaning.
bool jacket_controls_is_button_pressed(jacket_control_button_id_t button_id)
{
    jacket_control_button_state_t *button_state = jacket_controls_find_button_state(button_id);

    if (!s_controls_initialized || button_state == NULL) {
        return false;
    }

    return gpio_get_level(button_state->gpio_num) == 0;
}

// Rebuilds one button's debounced state after wake-up and drops any queued stale events so the same wake press is not replayed later.
void jacket_controls_sync_button_state(jacket_control_button_id_t button_id)
{
    jacket_control_button_state_t *button_state = jacket_controls_find_button_state(button_id);

    if (!s_controls_initialized || button_state == NULL || s_event_queue == NULL) {
        return;
    }

    int current_level = gpio_get_level(button_state->gpio_num);
    button_state->stable_level = current_level;
    button_state->last_sample_level = current_level;
    button_state->stable_sample_count = 1;

    jacket_control_button_event_t kept_events[JACKET_CONTROL_EVENT_QUEUE_LENGTH];
    size_t kept_event_count = 0;
    jacket_control_button_event_t pending_event = {0};

    while (xQueueReceive(s_event_queue, &pending_event, 0) == pdTRUE) {
        if (pending_event.button_id != button_id &&
            kept_event_count < JACKET_CONTROL_EVENT_QUEUE_LENGTH) {
            kept_events[kept_event_count++] = pending_event;
        }
    }

    for (size_t i = 0; i < kept_event_count; i++) {
        xQueueSend(s_event_queue, &kept_events[i], 0);
    }
}

// Updates the requested LED mode and immediately applies steady states or the initial blink edge.
esp_err_t jacket_controls_set_led_mode(
    jacket_control_led_id_t led_id,
    jacket_control_led_mode_t mode
)
{
    if (!s_controls_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!jacket_controls_is_valid_led_id(led_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_led_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    s_led_states[led_id].sequence_active = false;
    s_led_states[led_id].sequence_remaining_toggles = 0;
    s_led_states[led_id].sequence_toggle_period_ms = 0;
    s_led_states[led_id].sequence_final_mode = JACKET_CONTROL_LED_OFF;
    s_led_states[led_id].mode = mode;
    s_led_states[led_id].last_toggle_ms = jacket_controls_now_ms();

    switch (mode) {
    case JACKET_CONTROL_LED_OFF:
        s_led_states[led_id].current_output_level = false;
        break;

    case JACKET_CONTROL_LED_ON:
        s_led_states[led_id].current_output_level = true;
        break;

    case JACKET_CONTROL_LED_BLINK_SLOW:
    case JACKET_CONTROL_LED_BLINK_FAST:
        s_led_states[led_id].current_output_level = true;
        break;

    default:
        xSemaphoreGive(s_led_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    jacket_controls_apply_led_output(led_id, s_led_states[led_id].current_output_level);
    xSemaphoreGive(s_led_mutex);
    return ESP_OK;
}

// Starts a short finite blink sequence so the main firmware can signal events without blocking the BLE loop.
esp_err_t jacket_controls_start_led_blink_sequence(
    jacket_control_led_id_t led_id,
    uint32_t blink_count,
    uint32_t blink_period_ms,
    jacket_control_led_mode_t final_mode
)
{
    if (!s_controls_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!jacket_controls_is_valid_led_id(led_id) ||
        blink_count == 0 ||
        blink_period_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (final_mode != JACKET_CONTROL_LED_OFF &&
        final_mode != JACKET_CONTROL_LED_ON &&
        final_mode != JACKET_CONTROL_LED_BLINK_SLOW &&
        final_mode != JACKET_CONTROL_LED_BLINK_FAST) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_led_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    // The sequence begins with the LED immediately on, then counts the remaining toggles needed to complete the requested visible blinks.
    s_led_states[led_id].sequence_active = true;
    s_led_states[led_id].sequence_remaining_toggles = (blink_count * 2U) - 1U;
    s_led_states[led_id].sequence_toggle_period_ms = blink_period_ms;
    s_led_states[led_id].sequence_final_mode = final_mode;
    s_led_states[led_id].last_toggle_ms = jacket_controls_now_ms();
    s_led_states[led_id].current_output_level = true;

    jacket_controls_apply_led_output(led_id, true);
    xSemaphoreGive(s_led_mutex);
    return ESP_OK;
}

// Forces every externally managed LED off so the main firmware can reset the bring-up hardware state quickly.
esp_err_t jacket_controls_set_all_leds_off(void)
{
    if (!s_controls_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_led_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int led_id = JACKET_CONTROL_LED_PAIRING; led_id <= JACKET_CONTROL_LED_ALERT; led_id++) {
        s_led_states[led_id].mode = JACKET_CONTROL_LED_OFF;
        s_led_states[led_id].current_output_level = false;
        s_led_states[led_id].last_toggle_ms = jacket_controls_now_ms();
        s_led_states[led_id].sequence_active = false;
        s_led_states[led_id].sequence_remaining_toggles = 0;
        s_led_states[led_id].sequence_toggle_period_ms = 0;
        s_led_states[led_id].sequence_final_mode = JACKET_CONTROL_LED_OFF;
        jacket_controls_apply_led_output((jacket_control_led_id_t) led_id, false);
    }

    xSemaphoreGive(s_led_mutex);
    return ESP_OK;
}
