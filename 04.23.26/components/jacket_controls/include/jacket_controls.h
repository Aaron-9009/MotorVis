#ifndef JACKET_CONTROLS_H
#define JACKET_CONTROLS_H

// Exposes the small C-facing contract used by the main BLE firmware to read button events and drive external test LEDs.
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
// C linkage keeps the exported jacket controls API callable from the main firmware written in C.
extern "C" {
#endif

// Identifies which external button generated an event for the main firmware.
typedef enum {
    JACKET_CONTROL_BUTTON_PAIRING = 0,
    JACKET_CONTROL_BUTTON_ALERT_CANCEL,
    // Temporary power-button event used only for logging until the real power-cut hardware path is designed.
    JACKET_CONTROL_BUTTON_POWER,
} jacket_control_button_id_t;

// Identifies which external LED the main firmware wants to control.
typedef enum {
    JACKET_CONTROL_LED_PAIRING = 0,
    JACKET_CONTROL_LED_ALERT,
} jacket_control_led_id_t;

// Defines the supported LED drive modes owned by the jacket controls component.
typedef enum {
    JACKET_CONTROL_LED_OFF = 0,
    JACKET_CONTROL_LED_ON,
    JACKET_CONTROL_LED_BLINK_SLOW,
    JACKET_CONTROL_LED_BLINK_FAST,
} jacket_control_led_mode_t;

// Carries one debounced button-press event from the controls component to the main firmware.
typedef struct {
    // Identifies which button produced this event.
    jacket_control_button_id_t button_id;
    // Millisecond timestamp captured when the debounced press was accepted.
    uint32_t timestamp_ms;
    // Monotonic sequence value so the main firmware can log button activity in order.
    uint32_t sequence;
} jacket_control_button_event_t;

// Performs one-time button and LED hardware setup and starts the internal poll/update tasks.
esp_err_t jacket_controls_init(void);

// Returns the next queued debounced button event without blocking when no event is available.
bool jacket_controls_get_next_event(jacket_control_button_event_t *out_event);

// Returns the immediate pressed/released state of one button so other firmware modules can coordinate temporary sleep flows.
bool jacket_controls_is_button_pressed(jacket_control_button_id_t button_id);

// Re-synchronizes one button's debounced state and removes any queued stale events for that button after an external state transition.
void jacket_controls_sync_button_state(jacket_control_button_id_t button_id);

// Changes the requested external LED into the specified drive mode.
esp_err_t jacket_controls_set_led_mode(
    jacket_control_led_id_t led_id,
    jacket_control_led_mode_t mode
);

// Starts a short blink sequence on one external LED and then leaves that LED in the requested final mode.
esp_err_t jacket_controls_start_led_blink_sequence(
    jacket_control_led_id_t led_id,
    uint32_t blink_count,
    uint32_t blink_period_ms,
    jacket_control_led_mode_t final_mode
);

// Forces all externally managed jacket LEDs off.
esp_err_t jacket_controls_set_all_leds_off(void);

#ifdef __cplusplus
}
#endif

#endif // JACKET_CONTROLS_H
