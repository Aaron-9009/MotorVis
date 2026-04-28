#include <cmath>
#include <cstdio>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "accelerometer_manager.h"
#include "accel_calibration_data.h"

// This demo isolates the ADXL345 path from the production BLE firmware.
// It is meant to verify the sensor as an accelerometer for the jacket: axis calibration,
// resting gravity readings, sudden acceleration changes, and rough impact-threshold behavior.
// Speedometer behavior belongs to the GNSS path, not this accelerometer-only demo.

static const char *TAG = "ACCEL_DEMO";

// Set this to 1 when collecting the six-face calibration values for the ADXL345.
// Set it back to 0 after pasting the printed initializer into accel_calibration_data.cpp.
#define ACCEL_DEMO_RUN_SIX_FACE_CALIBRATION  0

#define ACCEL_DEMO_FACE_COUNT                6
#define ACCEL_DEMO_FACE_SETTLE_MS            5000
#define ACCEL_DEMO_FACE_SAMPLES              100
#define ACCEL_DEMO_CALIBRATION_COPY_PAUSE_MS 300000
#define ACCEL_DEMO_SPEED_WINDOW_SAMPLES      100
#define ACCEL_DEMO_SPEED_TREND_MIN_SAMPLES   20
#define ACCEL_DEMO_SPEED_TREND_EPS_MPS       0.05f
#define ACCEL_DEMO_DASHBOARD_PERIOD_MS       500
#define ACCEL_DEMO_SAMPLE_WAIT_TIMEOUT_MS    5000
#define ACCEL_DEMO_G_TO_MPS2                 9.80665f
#define ACCEL_DEMO_MG_TO_MPS2                (ACCEL_DEMO_G_TO_MPS2 / 1000.0f)
#define ACCEL_DEMO_MPS_TO_MPH                2.2369363f
#define ACCEL_DEMO_ACCEL_DEADBAND_MPS2       0.16f
#define ACCEL_DEMO_STATIONARY_LIMIT_MPS2     0.25f
#define ACCEL_DEMO_STATIONARY_VARIANCE_LIMIT 0.035f
#define ACCEL_DEMO_STATIONARY_CONFIRM_SAMPLES 5
#define ACCEL_DEMO_WINDOW_VALID_MIN_SAMPLES  3
#define ACCEL_DEMO_LINEAR_BIAS_ALPHA         0.03f
#define ACCEL_DEMO_SPEED_SLOPE_EPS_MPS2      0.025f
#define ACCEL_DEMO_MAX_INTEGRATION_DT_S      0.50f
#define ACCEL_DEMO_VELOCITY_DECAY            0.78f
#define ACCEL_DEMO_VELOCITY_ZERO_LIMIT_MPS   0.03f
#define ACCEL_DEMO_GRAVITY_TARGET_MG         1000.0f
#define ACCEL_DEMO_GRAVITY_MAG_TOLERANCE_MG  140.0f
#define ACCEL_DEMO_ORIENTATION_CHANGE_DEG    8.0f
#define ACCEL_DEMO_GRAVITY_ALPHA_NORMAL      0.04f
#define ACCEL_DEMO_GRAVITY_ALPHA_FAST        0.18f
#define ACCEL_DEMO_RAD_TO_DEG                57.2957795f
#define ACCEL_DEMO_AXIS_OFFSET_TOLERANCE_MG  180.0f
#define ACCEL_DEMO_IMPACT_MAG_THRESHOLD_MG   2500.0f
#define ACCEL_DEMO_DYNAMIC_SPIKE_THRESHOLD_MG 1500.0f
#define ACCEL_DEMO_SUDDEN_DELTA_THRESHOLD_MG 800.0f

typedef struct {
    float x_mg;
    float y_mg;
    float z_mg;
} accel_demo_vector_mg_t;

typedef struct {
    float x_mps2;
    float y_mps2;
    float z_mps2;
} accel_demo_vector_mps2_t;

typedef struct {
    float vx_mps;
    float vy_mps;
    float vz_mps;
} accel_demo_velocity_t;

typedef enum {
    ACCEL_DEMO_SAMPLE_STATUS_MOVING_ESTIMATE = 0,
    ACCEL_DEMO_SAMPLE_STATUS_STATIONARY,
    ACCEL_DEMO_SAMPLE_STATUS_ORIENTATION_CHANGING,
    ACCEL_DEMO_SAMPLE_STATUS_LOW_CONFIDENCE,
} accel_demo_sample_status_t;

typedef struct {
    const char *label;
    const char *instruction;
} accel_demo_calibration_face_t;

typedef struct {
    uint32_t timestamp_ms;
    accel_demo_vector_mg_t corrected_mg;
    accel_demo_vector_mps2_t linear_mps2;
    accel_demo_vector_mps2_t bias_corrected_linear_mps2;
    float linear_accel_mag_mps2;
    accel_demo_sample_status_t status;
    bool valid_for_speed;
} accel_demo_speed_window_sample_t;

typedef struct {
    accel_demo_speed_window_sample_t samples[ACCEL_DEMO_SPEED_WINDOW_SAMPLES];
    int next_index;
    int count;
} accel_demo_speed_window_t;

// The rolling speed window is intentionally file-static instead of local to app_main().
// Keeping 100 samples on the main task stack can overflow the ESP32 main task before the demo starts.
[[maybe_unused]] static accel_demo_speed_window_t s_speed_window = {};

// The bias estimate is only updated while the board looks stationary.
// This lets the demo slowly learn small sensor offsets without learning real motion as "zero."
static accel_demo_vector_mps2_t s_linear_bias_mps2 = {};
static int s_linear_bias_sample_count = 0;
[[maybe_unused]] static int s_stationary_candidate_count = 0;
static accel_demo_vector_mg_t s_previous_corrected_mg = {};
static bool s_previous_corrected_valid = false;
static float s_peak_total_mag_mg = 0.0f;
static float s_peak_dynamic_mag_mg = 0.0f;
static float s_peak_delta_mag_mg = 0.0f;
static uint32_t s_impact_event_count = 0;

typedef struct {
    int window_samples;
    uint32_t window_duration_ms;
    int stationary_sample_count;
    int valid_integration_sample_count;
    float linear_accel_variance_mps2;
    accel_demo_velocity_t current_window_velocity;
    float current_window_speed_mps;
    float window_peak_speed_mps;
    float window_delta_speed_mps;
    float speed_slope_mps2;
    const char *speed_trend;
    const char *confidence_label;
} accel_demo_speed_window_result_t;

typedef struct {
    const char *face_label;
    accel_demo_vector_mg_t expected_mg;
    accel_demo_vector_mg_t error_mg;
    float fit_error_mg;
    bool within_tolerance;
} accel_demo_axis_check_t;

typedef struct {
    float total_mag_mg;
    float dynamic_mag_mg;
    float sample_delta_mg;
    float peak_total_mag_mg;
    float peak_dynamic_mag_mg;
    float peak_delta_mag_mg;
    bool impact_threshold_crossed;
    bool sudden_delta_crossed;
    uint32_t impact_event_count;
    const char *impact_state;
} accel_demo_validation_result_t;

// Previous one-orientation baseline behavior:
// typedef struct {
//     float ax_mg;
//     float ay_mg;
//     float az_mg;
// } accel_demo_baseline_t;
//
// The older demo averaged one startup position and subtracted that same baseline forever.
// That helped only while the board stayed in the exact same orientation. Once the board was tilted,
// gravity moved onto a different axis and the old speed logic could integrate gravity as fake motion.
// The six-face calibration below replaces that approach by correcting each axis before gravity removal.

// Waits for a fresh manager sample so the demo redraws only when new sensor data exists.
static bool accel_demo_wait_for_next_sample(
    uint32_t *last_sequence,
    motor_vis_accel_sample_t *out_sample,
    uint32_t timeout_ms
)
{
    int64_t start_us = esp_timer_get_time();

    while (((esp_timer_get_time() - start_us) / 1000) < timeout_ms) {
        motor_vis_accel_sample_t sample = {};
        if (motor_vis_accel_manager_get_latest_sample(&sample) &&
            sample.sequence > *last_sequence) {
            *last_sequence = sample.sequence;
            *out_sample = sample;
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false;
}

#if ACCEL_DEMO_RUN_SIX_FACE_CALIBRATION
static const accel_demo_calibration_face_t ACCEL_DEMO_CALIBRATION_FACES[ACCEL_DEMO_FACE_COUNT] = {
    {"+X up", "place the sensor so the +X axis points upward and hold it still"},
    {"-X up", "place the sensor so the -X axis points upward and hold it still"},
    {"+Y up", "place the sensor so the +Y axis points upward and hold it still"},
    {"-Y up", "place the sensor so the -Y axis points upward and hold it still"},
    {"+Z up", "place the sensor so the +Z axis points upward and hold it still"},
    {"-Z up", "place the sensor so the -Z axis points upward and hold it still"},
};
#endif

// Keeps angle math safe if sensor noise pushes a ratio slightly outside the normal -1 to 1 range.
static float accel_demo_clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }

    if (value < -1.0f) {
        return -1.0f;
    }

    return value;
}

// Ignores very small acceleration changes so sensor noise does not immediately become fake velocity.
static float accel_demo_apply_deadband(float value_mps2)
{
    if (fabsf(value_mps2) < ACCEL_DEMO_ACCEL_DEADBAND_MPS2) {
        return 0.0f;
    }

    return value_mps2;
}

static float accel_demo_vector_mag_mg(const accel_demo_vector_mg_t *vector)
{
    return sqrtf(
        vector->x_mg * vector->x_mg +
        vector->y_mg * vector->y_mg +
        vector->z_mg * vector->z_mg
    );
}

[[maybe_unused]] static float accel_demo_vector_mag_mps2(const accel_demo_vector_mps2_t *vector)
{
    return sqrtf(
        vector->x_mps2 * vector->x_mps2 +
        vector->y_mps2 * vector->y_mps2 +
        vector->z_mps2 * vector->z_mps2
    );
}

static float accel_demo_velocity_mag_mps(const accel_demo_velocity_t *velocity)
{
    return sqrtf(
        velocity->vx_mps * velocity->vx_mps +
        velocity->vy_mps * velocity->vy_mps +
        velocity->vz_mps * velocity->vz_mps
    );
}

[[maybe_unused]] static accel_demo_vector_mps2_t accel_demo_zero_vector_mps2(void)
{
    accel_demo_vector_mps2_t zero = {};
    return zero;
}

// Learns a tiny linear-acceleration offset only when the sensor appears still.
// This is the accelerometer-only version of a zero-velocity update: still samples become the reference.
[[maybe_unused]] static void accel_demo_update_linear_bias_from_stationary(
    const accel_demo_vector_mps2_t *linear_accel
)
{
    if (s_linear_bias_sample_count == 0) {
        s_linear_bias_mps2 = *linear_accel;
        s_linear_bias_sample_count = 1;
        return;
    }

    s_linear_bias_mps2.x_mps2 =
        (s_linear_bias_mps2.x_mps2 * (1.0f - ACCEL_DEMO_LINEAR_BIAS_ALPHA)) +
        (linear_accel->x_mps2 * ACCEL_DEMO_LINEAR_BIAS_ALPHA);
    s_linear_bias_mps2.y_mps2 =
        (s_linear_bias_mps2.y_mps2 * (1.0f - ACCEL_DEMO_LINEAR_BIAS_ALPHA)) +
        (linear_accel->y_mps2 * ACCEL_DEMO_LINEAR_BIAS_ALPHA);
    s_linear_bias_mps2.z_mps2 =
        (s_linear_bias_mps2.z_mps2 * (1.0f - ACCEL_DEMO_LINEAR_BIAS_ALPHA)) +
        (linear_accel->z_mps2 * ACCEL_DEMO_LINEAR_BIAS_ALPHA);

    s_linear_bias_sample_count++;
}

// Removes the learned stationary bias before integration so small offsets do not become fake speed.
[[maybe_unused]] static accel_demo_vector_mps2_t accel_demo_apply_linear_bias(
    const accel_demo_vector_mps2_t *linear_accel
)
{
    accel_demo_vector_mps2_t corrected = {};
    corrected.x_mps2 = accel_demo_apply_deadband(linear_accel->x_mps2 - s_linear_bias_mps2.x_mps2);
    corrected.y_mps2 = accel_demo_apply_deadband(linear_accel->y_mps2 - s_linear_bias_mps2.y_mps2);
    corrected.z_mps2 = accel_demo_apply_deadband(linear_accel->z_mps2 - s_linear_bias_mps2.z_mps2);
    return corrected;
}

[[maybe_unused]] static const char *accel_demo_sample_status_to_string(accel_demo_sample_status_t status)
{
    switch (status) {
        case ACCEL_DEMO_SAMPLE_STATUS_STATIONARY:
            return "STATIONARY_ZEROED";
        case ACCEL_DEMO_SAMPLE_STATUS_ORIENTATION_CHANGING:
            return "ORIENTATION_CHANGING";
        case ACCEL_DEMO_SAMPLE_STATUS_LOW_CONFIDENCE:
            return "LOW_CONFIDENCE_DYNAMIC_ACCEL";
        case ACCEL_DEMO_SAMPLE_STATUS_MOVING_ESTIMATE:
        default:
            return "MOVING_ESTIMATE";
    }
}

// Resets the recent sample history whenever the current motion state would make old samples misleading.
[[maybe_unused]] static void accel_demo_speed_window_clear(accel_demo_speed_window_t *window)
{
    window->next_index = 0;
    window->count = 0;
}

// Adds the newest corrected and gravity-filtered sample to a 100-sample history buffer.
[[maybe_unused]] static void accel_demo_speed_window_push(
    accel_demo_speed_window_t *window,
    uint32_t timestamp_ms,
    const accel_demo_vector_mg_t *corrected_mg,
    const accel_demo_vector_mps2_t *linear_mps2,
    const accel_demo_vector_mps2_t *bias_corrected_linear_mps2,
    float linear_accel_mag_mps2,
    accel_demo_sample_status_t status,
    bool valid_for_speed
)
{
    window->samples[window->next_index].timestamp_ms = timestamp_ms;
    window->samples[window->next_index].corrected_mg = *corrected_mg;
    window->samples[window->next_index].linear_mps2 = *linear_mps2;
    window->samples[window->next_index].bias_corrected_linear_mps2 = *bias_corrected_linear_mps2;
    window->samples[window->next_index].linear_accel_mag_mps2 = linear_accel_mag_mps2;
    window->samples[window->next_index].status = status;
    window->samples[window->next_index].valid_for_speed = valid_for_speed;

    window->next_index = (window->next_index + 1) % ACCEL_DEMO_SPEED_WINDOW_SAMPLES;
    if (window->count < ACCEL_DEMO_SPEED_WINDOW_SAMPLES) {
        window->count++;
    }
}

static const accel_demo_speed_window_sample_t *accel_demo_speed_window_get_ordered(
    const accel_demo_speed_window_t *window,
    int ordered_index
)
{
    int oldest_index = window->next_index - window->count;
    while (oldest_index < 0) {
        oldest_index += ACCEL_DEMO_SPEED_WINDOW_SAMPLES;
    }

    int physical_index = (oldest_index + ordered_index) % ACCEL_DEMO_SPEED_WINDOW_SAMPLES;
    return &window->samples[physical_index];
}

// Previous 100-sample integration behavior:
// The first window version integrated every non-orientation sample and reported |delta-v| as speed.
// That made the code simple, but it still let bias and low-confidence motion accumulate inside the
// window. The constrained estimator below keeps this idea, but adds bias correction, ZUPT-style
// stationary resets, validity checks, and a least-squares speed slope for trend.
//
// static accel_demo_velocity_t accel_demo_integrate_window_range(...)
// {
//     for each sample pair:
//         skip orientation changes
//         integrate raw linear_mps2 with trapezoidal integration
//     return delta_velocity;
// }
//
// static accel_demo_speed_window_result_t accel_demo_calculate_speed_window_result(...)
// {
//     window_speed_mps = |integrated_delta_velocity|;
//     speed_trend = compare first-half speed against second-half speed;
// }

// Calculates how much the linear acceleration has been moving around inside the window.
// Low variance plus low current acceleration is used as one confidence check for stationary detection.
static float accel_demo_calculate_window_linear_variance(const accel_demo_speed_window_t *window)
{
    if (window->count < 2) {
        return 0.0f;
    }

    float sum = 0.0f;
    float sum_sq = 0.0f;
    int usable_count = 0;

    for (int index = 0; index < window->count; index++) {
        const accel_demo_speed_window_sample_t *sample = accel_demo_speed_window_get_ordered(window, index);
        if (sample->status == ACCEL_DEMO_SAMPLE_STATUS_ORIENTATION_CHANGING) {
            continue;
        }

        sum += sample->linear_accel_mag_mps2;
        sum_sq += sample->linear_accel_mag_mps2 * sample->linear_accel_mag_mps2;
        usable_count++;
    }

    if (usable_count < 2) {
        return 0.0f;
    }

    float mean = sum / (float) usable_count;
    float variance = (sum_sq / (float) usable_count) - (mean * mean);
    if (variance < 0.0f) {
        variance = 0.0f;
    }

    return variance;
}

// Calculates the constrained speed estimate and trend from the full 100-sample history.
// Stationary samples act like zero-velocity updates so the estimate cannot drift forever.
[[maybe_unused]] static accel_demo_speed_window_result_t accel_demo_calculate_speed_window_result(
    const accel_demo_speed_window_t *window
)
{
    accel_demo_speed_window_result_t result = {};
    result.window_samples = window->count;
    result.speed_trend = "COLLECTING";
    result.confidence_label = "COLLECTING_WINDOW";
    result.linear_accel_variance_mps2 = accel_demo_calculate_window_linear_variance(window);

    if (window->count < 2) {
        return result;
    }

    const accel_demo_speed_window_sample_t *oldest = accel_demo_speed_window_get_ordered(window, 0);
    const accel_demo_speed_window_sample_t *newest = accel_demo_speed_window_get_ordered(window, window->count - 1);
    if (newest->timestamp_ms >= oldest->timestamp_ms) {
        result.window_duration_ms = newest->timestamp_ms - oldest->timestamp_ms;
    }

    accel_demo_velocity_t velocity = {};
    float first_speed_mps = -1.0f;
    float newest_speed_mps = 0.0f;
    double sum_t = 0.0;
    double sum_speed = 0.0;
    double sum_tt = 0.0;
    double sum_ts = 0.0;
    int slope_sample_count = 0;

    for (int index = 0; index < window->count; index++) {
        const accel_demo_speed_window_sample_t *current = accel_demo_speed_window_get_ordered(window, index);

        if (current->status == ACCEL_DEMO_SAMPLE_STATUS_STATIONARY) {
            velocity = {};
            result.stationary_sample_count++;
        } else if (index > 0) {
            const accel_demo_speed_window_sample_t *previous = accel_demo_speed_window_get_ordered(window, index - 1);
            bool pair_is_valid =
                previous->valid_for_speed &&
                current->valid_for_speed &&
                previous->status != ACCEL_DEMO_SAMPLE_STATUS_ORIENTATION_CHANGING &&
                current->status != ACCEL_DEMO_SAMPLE_STATUS_ORIENTATION_CHANGING &&
                current->timestamp_ms > previous->timestamp_ms;

            if (pair_is_valid) {
                float dt_s = ((float) (current->timestamp_ms - previous->timestamp_ms)) / 1000.0f;
                if (dt_s > 0.0f && dt_s <= ACCEL_DEMO_MAX_INTEGRATION_DT_S) {
                    velocity.vx_mps +=
                        ((previous->bias_corrected_linear_mps2.x_mps2 + current->bias_corrected_linear_mps2.x_mps2) * 0.5f) * dt_s;
                    velocity.vy_mps +=
                        ((previous->bias_corrected_linear_mps2.y_mps2 + current->bias_corrected_linear_mps2.y_mps2) * 0.5f) * dt_s;
                    velocity.vz_mps +=
                        ((previous->bias_corrected_linear_mps2.z_mps2 + current->bias_corrected_linear_mps2.z_mps2) * 0.5f) * dt_s;
                    result.valid_integration_sample_count++;
                }
            }
        }

        if (!current->valid_for_speed) {
            continue;
        }

        float speed_mps = accel_demo_velocity_mag_mps(&velocity);
        if (first_speed_mps < 0.0f) {
            first_speed_mps = speed_mps;
        }

        if (speed_mps > result.window_peak_speed_mps) {
            result.window_peak_speed_mps = speed_mps;
        }

        newest_speed_mps = speed_mps;
        double t_s = ((double) (current->timestamp_ms - oldest->timestamp_ms)) / 1000.0;
        sum_t += t_s;
        sum_speed += (double) speed_mps;
        sum_tt += t_s * t_s;
        sum_ts += t_s * (double) speed_mps;
        slope_sample_count++;
    }

    result.current_window_velocity = velocity;
    result.current_window_speed_mps = newest_speed_mps;
    if (first_speed_mps >= 0.0f) {
        result.window_delta_speed_mps = newest_speed_mps - first_speed_mps;
    }

    if (slope_sample_count >= ACCEL_DEMO_SPEED_TREND_MIN_SAMPLES) {
        double denominator = ((double) slope_sample_count * sum_tt) - (sum_t * sum_t);
        if (fabs(denominator) > 0.000001) {
            result.speed_slope_mps2 = (float) ((((double) slope_sample_count * sum_ts) - (sum_t * sum_speed)) / denominator);
        }

        if (result.speed_slope_mps2 > ACCEL_DEMO_SPEED_SLOPE_EPS_MPS2) {
            result.speed_trend = "INCREASING";
        } else if (result.speed_slope_mps2 < -ACCEL_DEMO_SPEED_SLOPE_EPS_MPS2) {
            result.speed_trend = "DECREASING";
        } else {
            result.speed_trend = "STABLE";
        }
    }

    if (result.valid_integration_sample_count < ACCEL_DEMO_WINDOW_VALID_MIN_SAMPLES) {
        result.confidence_label = "LOW_VALID_SAMPLE_COUNT";
    } else if (result.stationary_sample_count > 0 && result.current_window_speed_mps < ACCEL_DEMO_VELOCITY_ZERO_LIMIT_MPS) {
        result.confidence_label = "ZUPT_ZEROED";
    } else if (result.linear_accel_variance_mps2 <= ACCEL_DEMO_STATIONARY_VARIANCE_LIMIT) {
        result.confidence_label = "LOW_VARIANCE_STABLE";
    } else {
        result.confidence_label = "SHORT_WINDOW_ESTIMATE";
    }

    return result;
}

// Compares two acceleration directions so tilt can be treated differently from real linear motion.
[[maybe_unused]] static float accel_demo_angle_between_vectors_deg(
    const accel_demo_vector_mg_t *first,
    const accel_demo_vector_mg_t *second
)
{
    float first_mag = accel_demo_vector_mag_mg(first);
    float second_mag = accel_demo_vector_mag_mg(second);

    if (first_mag < 1.0f || second_mag < 1.0f) {
        return 0.0f;
    }

    float dot =
        first->x_mg * second->x_mg +
        first->y_mg * second->y_mg +
        first->z_mg * second->z_mg;
    float cos_angle = accel_demo_clamp_unit(dot / (first_mag * second_mag));

    return acosf(cos_angle) * ACCEL_DEMO_RAD_TO_DEG;
}

// Applies the stored six-face calibration before the reading is used by the speed estimate.
static accel_demo_vector_mg_t accel_demo_apply_calibration(
    const motor_vis_accel_sample_t *sample,
    const accel_demo_calibration_t *calibration
)
{
    accel_demo_vector_mg_t corrected = {};
    corrected.x_mg = ((float) sample->ax_mg - calibration->x_offset_mg) * calibration->x_scale;
    corrected.y_mg = ((float) sample->ay_mg - calibration->y_offset_mg) * calibration->y_scale;
    corrected.z_mg = ((float) sample->az_mg - calibration->z_offset_mg) * calibration->z_scale;
    return corrected;
}

// Finds which calibrated axis is closest to pointing with gravity.
// This is a quick bench check for whether offset/scale calibration still makes physical sense.
static accel_demo_axis_check_t accel_demo_calculate_axis_check(const accel_demo_vector_mg_t *corrected)
{
    accel_demo_axis_check_t check = {};
    float abs_x = fabsf(corrected->x_mg);
    float abs_y = fabsf(corrected->y_mg);
    float abs_z = fabsf(corrected->z_mg);

    if (abs_x >= abs_y && abs_x >= abs_z) {
        if (corrected->x_mg >= 0.0f) {
            check.face_label = "+X UP";
            check.expected_mg.x_mg = ACCEL_DEMO_GRAVITY_TARGET_MG;
        } else {
            check.face_label = "-X UP";
            check.expected_mg.x_mg = -ACCEL_DEMO_GRAVITY_TARGET_MG;
        }
    } else if (abs_y >= abs_x && abs_y >= abs_z) {
        if (corrected->y_mg >= 0.0f) {
            check.face_label = "+Y UP";
            check.expected_mg.y_mg = ACCEL_DEMO_GRAVITY_TARGET_MG;
        } else {
            check.face_label = "-Y UP";
            check.expected_mg.y_mg = -ACCEL_DEMO_GRAVITY_TARGET_MG;
        }
    } else {
        if (corrected->z_mg >= 0.0f) {
            check.face_label = "+Z UP";
            check.expected_mg.z_mg = ACCEL_DEMO_GRAVITY_TARGET_MG;
        } else {
            check.face_label = "-Z UP";
            check.expected_mg.z_mg = -ACCEL_DEMO_GRAVITY_TARGET_MG;
        }
    }

    check.error_mg.x_mg = corrected->x_mg - check.expected_mg.x_mg;
    check.error_mg.y_mg = corrected->y_mg - check.expected_mg.y_mg;
    check.error_mg.z_mg = corrected->z_mg - check.expected_mg.z_mg;
    check.fit_error_mg = accel_demo_vector_mag_mg(&check.error_mg);
    check.within_tolerance = check.fit_error_mg <= ACCEL_DEMO_AXIS_OFFSET_TOLERANCE_MG;

    return check;
}

static float accel_demo_vector_delta_mag_mg(
    const accel_demo_vector_mg_t *previous,
    const accel_demo_vector_mg_t *current
)
{
    accel_demo_vector_mg_t delta = {};
    delta.x_mg = current->x_mg - previous->x_mg;
    delta.y_mg = current->y_mg - previous->y_mg;
    delta.z_mg = current->z_mg - previous->z_mg;
    return accel_demo_vector_mag_mg(&delta);
}

// Tracks jacket-safety style checks: total acceleration, gravity deviation, and sudden sample change.
static accel_demo_validation_result_t accel_demo_calculate_validation_result(
    const accel_demo_vector_mg_t *corrected,
    float corrected_mag_mg
)
{
    accel_demo_validation_result_t result = {};
    result.total_mag_mg = corrected_mag_mg;
    result.dynamic_mag_mg = fabsf(corrected_mag_mg - ACCEL_DEMO_GRAVITY_TARGET_MG);

    if (s_previous_corrected_valid) {
        result.sample_delta_mg = accel_demo_vector_delta_mag_mg(&s_previous_corrected_mg, corrected);
    }

    result.impact_threshold_crossed =
        corrected_mag_mg >= ACCEL_DEMO_IMPACT_MAG_THRESHOLD_MG ||
        result.dynamic_mag_mg >= ACCEL_DEMO_DYNAMIC_SPIKE_THRESHOLD_MG;
    result.sudden_delta_crossed =
        s_previous_corrected_valid &&
        result.sample_delta_mg >= ACCEL_DEMO_SUDDEN_DELTA_THRESHOLD_MG;

    if (result.impact_threshold_crossed || result.sudden_delta_crossed) {
        s_impact_event_count++;
    }

    if (corrected_mag_mg > s_peak_total_mag_mg) {
        s_peak_total_mag_mg = corrected_mag_mg;
    }

    if (result.dynamic_mag_mg > s_peak_dynamic_mag_mg) {
        s_peak_dynamic_mag_mg = result.dynamic_mag_mg;
    }

    if (result.sample_delta_mg > s_peak_delta_mag_mg) {
        s_peak_delta_mag_mg = result.sample_delta_mg;
    }

    result.peak_total_mag_mg = s_peak_total_mag_mg;
    result.peak_dynamic_mag_mg = s_peak_dynamic_mag_mg;
    result.peak_delta_mag_mg = s_peak_delta_mag_mg;
    result.impact_event_count = s_impact_event_count;

    if (result.impact_threshold_crossed && result.sudden_delta_crossed) {
        result.impact_state = "IMPACT_AND_SUDDEN_DELTA";
    } else if (result.impact_threshold_crossed) {
        result.impact_state = "IMPACT_THRESHOLD";
    } else if (result.sudden_delta_crossed) {
        result.impact_state = "SUDDEN_DELTA";
    } else {
        result.impact_state = "NORMAL";
    }

    s_previous_corrected_mg = *corrected;
    s_previous_corrected_valid = true;

    return result;
}

#if ACCEL_DEMO_RUN_SIX_FACE_CALIBRATION
// Averages one physical face while the board is held still during six-face calibration.
static bool accel_demo_collect_face_average(
    const accel_demo_calibration_face_t *face,
    uint32_t *last_sequence,
    accel_demo_face_average_t *out_average
)
{
    ESP_LOGI(TAG, "calibration face: %s", face->label);
    ESP_LOGI(TAG, "%s", face->instruction);
    ESP_LOGI(TAG, "collection starts in %d seconds", ACCEL_DEMO_FACE_SETTLE_MS / 1000);

    for (int seconds_left = ACCEL_DEMO_FACE_SETTLE_MS / 1000; seconds_left > 0; seconds_left--) {
        printf("Hold %s still... starting in %d\r", face->label, seconds_left);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n");

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (int sample_index = 0; sample_index < ACCEL_DEMO_FACE_SAMPLES; sample_index++) {
        motor_vis_accel_sample_t sample = {};
        if (!accel_demo_wait_for_next_sample(last_sequence, &sample, ACCEL_DEMO_SAMPLE_WAIT_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "timed out while collecting %s calibration sample", face->label);
            return false;
        }

        sum_x += (float) sample.ax_mg;
        sum_y += (float) sample.ay_mg;
        sum_z += (float) sample.az_mg;

        printf(
            "collecting %-5s %03d/%03d raw_mg=(%d, %d, %d)\r",
            face->label,
            sample_index + 1,
            ACCEL_DEMO_FACE_SAMPLES,
            (int) sample.ax_mg,
            (int) sample.ay_mg,
            (int) sample.az_mg
        );
        fflush(stdout);
    }
    printf("\n");

    out_average->x_mg = sum_x / (float) ACCEL_DEMO_FACE_SAMPLES;
    out_average->y_mg = sum_y / (float) ACCEL_DEMO_FACE_SAMPLES;
    out_average->z_mg = sum_z / (float) ACCEL_DEMO_FACE_SAMPLES;

    ESP_LOGI(
        TAG,
        "%s average complete: x=%.2f mg y=%.2f mg z=%.2f mg",
        face->label,
        (double) out_average->x_mg,
        (double) out_average->y_mg,
        (double) out_average->z_mg
    );

    return true;
}

static float accel_demo_calculate_axis_offset(float positive_face_mg, float negative_face_mg)
{
    return (positive_face_mg + negative_face_mg) / 2.0f;
}

static float accel_demo_calculate_axis_scale(float positive_face_mg, float negative_face_mg)
{
    float half_span_mg = (positive_face_mg - negative_face_mg) / 2.0f;

    if (fabsf(half_span_mg) < 1.0f) {
        return 1.0f;
    }

    return ACCEL_DEMO_GRAVITY_TARGET_MG / half_span_mg;
}

// Prints the exact source initializer to paste into accel_calibration_data.cpp after calibration.
static void accel_demo_print_calibration_initializer(const accel_demo_calibration_t *calibration)
{
    printf("\n\n");
    printf("Copy the initializer below into main/accel_calibration_data.cpp:\n\n");
    printf("const accel_demo_calibration_t ACCEL_DEMO_STORED_CALIBRATION = {\n");
    printf("    %.6ff,     // x_offset_mg\n", (double) calibration->x_offset_mg);
    printf("    %.6ff,     // y_offset_mg\n", (double) calibration->y_offset_mg);
    printf("    %.6ff,     // z_offset_mg\n\n", (double) calibration->z_offset_mg);
    printf("    %.8ff,     // x_scale\n", (double) calibration->x_scale);
    printf("    %.8ff,     // y_scale\n", (double) calibration->y_scale);
    printf("    %.8ff,     // z_scale\n\n", (double) calibration->z_scale);
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // plus_x_up_average_mg\n",
        (double) calibration->plus_x_up_average_mg.x_mg,
        (double) calibration->plus_x_up_average_mg.y_mg,
        (double) calibration->plus_x_up_average_mg.z_mg
    );
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // minus_x_up_average_mg\n",
        (double) calibration->minus_x_up_average_mg.x_mg,
        (double) calibration->minus_x_up_average_mg.y_mg,
        (double) calibration->minus_x_up_average_mg.z_mg
    );
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // plus_y_up_average_mg\n",
        (double) calibration->plus_y_up_average_mg.x_mg,
        (double) calibration->plus_y_up_average_mg.y_mg,
        (double) calibration->plus_y_up_average_mg.z_mg
    );
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // minus_y_up_average_mg\n",
        (double) calibration->minus_y_up_average_mg.x_mg,
        (double) calibration->minus_y_up_average_mg.y_mg,
        (double) calibration->minus_y_up_average_mg.z_mg
    );
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // plus_z_up_average_mg\n",
        (double) calibration->plus_z_up_average_mg.x_mg,
        (double) calibration->plus_z_up_average_mg.y_mg,
        (double) calibration->plus_z_up_average_mg.z_mg
    );
    printf(
        "    {%.6ff, %.6ff, %.6ff},    // minus_z_up_average_mg\n",
        (double) calibration->minus_z_up_average_mg.x_mg,
        (double) calibration->minus_z_up_average_mg.y_mg,
        (double) calibration->minus_z_up_average_mg.z_mg
    );
    printf("};\n\n");
    printf(
        "The calibration results will stay on screen for %d minutes so they can be copied down.\n",
        ACCEL_DEMO_CALIBRATION_COPY_PAUSE_MS / 60000
    );
    fflush(stdout);

    // This pause is only used in calibration mode so the terminal does not jump into the live dashboard
    // before the measured constants can be copied into accel_calibration_data.cpp.
    for (int seconds_left = ACCEL_DEMO_CALIBRATION_COPY_PAUSE_MS / 1000; seconds_left > 0; seconds_left--) {
        printf("Copy pause remaining: %03d seconds\r", seconds_left);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("\nCopy pause complete. Continuing into the live demo using the newly measured values.\n");
    fflush(stdout);
}

// Walks through all six orientations and calculates offset/scale constants for the exact sensor module.
static bool accel_demo_run_six_face_calibration(
    uint32_t *last_sequence,
    accel_demo_calibration_t *out_calibration
)
{
    ESP_LOGW(TAG, "six-face calibration mode is active");
    ESP_LOGW(TAG, "keep each requested orientation still until that face finishes collecting samples");

    accel_demo_face_average_t averages[ACCEL_DEMO_FACE_COUNT] = {};
    for (int face_index = 0; face_index < ACCEL_DEMO_FACE_COUNT; face_index++) {
        if (!accel_demo_collect_face_average(&ACCEL_DEMO_CALIBRATION_FACES[face_index], last_sequence, &averages[face_index])) {
            return false;
        }
    }

    out_calibration->plus_x_up_average_mg = averages[0];
    out_calibration->minus_x_up_average_mg = averages[1];
    out_calibration->plus_y_up_average_mg = averages[2];
    out_calibration->minus_y_up_average_mg = averages[3];
    out_calibration->plus_z_up_average_mg = averages[4];
    out_calibration->minus_z_up_average_mg = averages[5];

    out_calibration->x_offset_mg = accel_demo_calculate_axis_offset(averages[0].x_mg, averages[1].x_mg);
    out_calibration->y_offset_mg = accel_demo_calculate_axis_offset(averages[2].y_mg, averages[3].y_mg);
    out_calibration->z_offset_mg = accel_demo_calculate_axis_offset(averages[4].z_mg, averages[5].z_mg);

    out_calibration->x_scale = accel_demo_calculate_axis_scale(averages[0].x_mg, averages[1].x_mg);
    out_calibration->y_scale = accel_demo_calculate_axis_scale(averages[2].y_mg, averages[3].y_mg);
    out_calibration->z_scale = accel_demo_calculate_axis_scale(averages[4].z_mg, averages[5].z_mg);

    accel_demo_print_calibration_initializer(out_calibration);
    return true;
}
#endif

// Previous running-velocity helpers:
// These were used before the demo switched to a 100-sample speed window. They are left here
// as a reference for the older approach, but the active code now clears or rebuilds the window
// instead of carrying one velocity value forever.
// static void accel_demo_reset_velocity(accel_demo_velocity_t *velocity)
// {
//     velocity->vx_mps = 0.0f;
//     velocity->vy_mps = 0.0f;
//     velocity->vz_mps = 0.0f;
// }
//
// static void accel_demo_decay_velocity_if_stationary(
//     accel_demo_velocity_t *velocity,
//     float linear_accel_mag_mps2
// )
// {
//     if (linear_accel_mag_mps2 >= ACCEL_DEMO_STATIONARY_LIMIT_MPS2) {
//         return;
//     }
//
//     velocity->vx_mps *= ACCEL_DEMO_VELOCITY_DECAY;
//     velocity->vy_mps *= ACCEL_DEMO_VELOCITY_DECAY;
//     velocity->vz_mps *= ACCEL_DEMO_VELOCITY_DECAY;
//
//     float speed_mps = sqrtf(
//         velocity->vx_mps * velocity->vx_mps +
//         velocity->vy_mps * velocity->vy_mps +
//         velocity->vz_mps * velocity->vz_mps
//     );
//
//     if (speed_mps < ACCEL_DEMO_VELOCITY_ZERO_LIMIT_MPS) {
//         accel_demo_reset_velocity(velocity);
//     }
// }

[[maybe_unused]] static accel_demo_vector_mps2_t accel_demo_calculate_linear_accel_mps2(
    const accel_demo_vector_mg_t *corrected,
    const accel_demo_vector_mg_t *gravity
)
{
    accel_demo_vector_mps2_t linear = {};
    linear.x_mps2 = accel_demo_apply_deadband((corrected->x_mg - gravity->x_mg) * ACCEL_DEMO_MG_TO_MPS2);
    linear.y_mps2 = accel_demo_apply_deadband((corrected->y_mg - gravity->y_mg) * ACCEL_DEMO_MG_TO_MPS2);
    linear.z_mps2 = accel_demo_apply_deadband((corrected->z_mg - gravity->z_mg) * ACCEL_DEMO_MG_TO_MPS2);
    return linear;
}

// Updates the estimated gravity vector. A fast update is used when the board is likely being reoriented
// so tilting does not get integrated as hundreds of miles per hour.
[[maybe_unused]] static void accel_demo_update_gravity_estimate(
    accel_demo_vector_mg_t *gravity,
    const accel_demo_vector_mg_t *corrected,
    bool orientation_changing
)
{
    float alpha = orientation_changing ? ACCEL_DEMO_GRAVITY_ALPHA_FAST : ACCEL_DEMO_GRAVITY_ALPHA_NORMAL;
    gravity->x_mg = (gravity->x_mg * (1.0f - alpha)) + (corrected->x_mg * alpha);
    gravity->y_mg = (gravity->y_mg * (1.0f - alpha)) + (corrected->y_mg * alpha);
    gravity->z_mg = (gravity->z_mg * (1.0f - alpha)) + (corrected->z_mg * alpha);
}

// Redraws a compact terminal dashboard focused on the checks that matter for jacket safety firmware.
static void accel_demo_print_dashboard(
    const motor_vis_accel_sample_t *sample,
    const accel_demo_calibration_t *calibration,
    const accel_demo_vector_mg_t *corrected,
    const accel_demo_axis_check_t *axis_check,
    const accel_demo_validation_result_t *validation
)
{
    printf("\033[2J\033[H");
    printf("MotoVis ADXL345 Accelerometer Validation Demo\n");
    printf("=============================================\n");
    printf("Purpose: verify axis calibration and impact-style accelerometer behavior.\n");
    printf("Speed note: this demo no longer estimates speed; GNSS should own speedometer data.\n");
    printf("Calibration: six-face offsets/scales are applied before threshold checks.\n\n");

    printf("sample_seq:      %lu\n", (unsigned long) sample->sequence);
    printf("timestamp_ms:    %lu\n", (unsigned long) sample->timestamp_ms);
    printf("manager_alert:   %u\n\n", (unsigned int) sample->alert);

    printf("raw accel mg:    ax=%7d  ay=%7d  az=%7d\n", (int) sample->ax_mg, (int) sample->ay_mg, (int) sample->az_mg);
    printf(
        "corrected mg:    ax=%7.2f  ay=%7.2f  az=%7.2f  |a|=%7.2f\n",
        (double) corrected->x_mg,
        (double) corrected->y_mg,
        (double) corrected->z_mg,
        (double) validation->total_mag_mg
    );
    printf(
        "cal offsets mg:  x=%7.2f  y=%7.2f  z=%7.2f\n",
        (double) calibration->x_offset_mg,
        (double) calibration->y_offset_mg,
        (double) calibration->z_offset_mg
    );
    printf(
        "cal scale:       x=%7.4f  y=%7.4f  z=%7.4f\n\n",
        (double) calibration->x_scale,
        (double) calibration->y_scale,
        (double) calibration->z_scale
    );

    printf("rest axis check: closest_face=%s  status=%s\n", axis_check->face_label, axis_check->within_tolerance ? "PASS" : "CHECK_SENSOR_OR_CAL");
    printf(
        "expected mg:     ex=%7.2f  ey=%7.2f  ez=%7.2f\n",
        (double) axis_check->expected_mg.x_mg,
        (double) axis_check->expected_mg.y_mg,
        (double) axis_check->expected_mg.z_mg
    );
    printf(
        "axis error mg:   dx=%7.2f  dy=%7.2f  dz=%7.2f  fit_error=%7.2f  limit=%7.2f\n\n",
        (double) axis_check->error_mg.x_mg,
        (double) axis_check->error_mg.y_mg,
        (double) axis_check->error_mg.z_mg,
        (double) axis_check->fit_error_mg,
        (double) ACCEL_DEMO_AXIS_OFFSET_TOLERANCE_MG
    );

    printf(
        "impact checks:   total_g=%6.3f g  dynamic=%7.2f mg  sample_delta=%7.2f mg\n",
        (double) (validation->total_mag_mg / ACCEL_DEMO_GRAVITY_TARGET_MG),
        (double) validation->dynamic_mag_mg,
        (double) validation->sample_delta_mg
    );
    printf(
        "thresholds:      total_mag>=%7.2f mg  dynamic>=%7.2f mg  delta>=%7.2f mg\n",
        (double) ACCEL_DEMO_IMPACT_MAG_THRESHOLD_MG,
        (double) ACCEL_DEMO_DYNAMIC_SPIKE_THRESHOLD_MG,
        (double) ACCEL_DEMO_SUDDEN_DELTA_THRESHOLD_MG
    );
    printf(
        "impact state:    %s  events=%lu\n",
        validation->impact_state,
        (unsigned long) validation->impact_event_count
    );
    printf(
        "peak values:     total=%7.2f mg  dynamic=%7.2f mg  delta=%7.2f mg\n\n",
        (double) validation->peak_total_mag_mg,
        (double) validation->peak_dynamic_mag_mg,
        (double) validation->peak_delta_mag_mg
    );

    printf("Testing tip: rotate through each physical face and look for PASS near +/-1000 mg.\n");
    printf("Tap or bump the board lightly to verify delta and impact thresholds respond.\n");
    fflush(stdout);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "starting standalone accelerometer demo firmware");
    ESP_LOGI(TAG, "this demo checks calibrated axes and impact-style acceleration thresholds");
    ESP_LOGI(TAG, "speedometer testing has been moved back to the GNSS path");

    esp_err_t err = motor_vis_accel_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "accelerometer manager init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check ADXL345 power, ground, SDA GPIO21, SCL GPIO22, and I2C address 0x53");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uint32_t last_sequence = 0;
    accel_demo_calibration_t active_calibration = ACCEL_DEMO_STORED_CALIBRATION;

#if ACCEL_DEMO_RUN_SIX_FACE_CALIBRATION
    if (!accel_demo_run_six_face_calibration(&last_sequence, &active_calibration)) {
        ESP_LOGE(TAG, "six-face calibration failed; stopping demo so readings are not misleading");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#else
    ESP_LOGI(TAG, "using stored six-face calibration from accel_calibration_data.cpp");
#endif

    motor_vis_accel_sample_t first_sample = {};
    if (!accel_demo_wait_for_next_sample(&last_sequence, &first_sample, ACCEL_DEMO_SAMPLE_WAIT_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "no accelerometer sample received after initialization");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    s_previous_corrected_mg = accel_demo_apply_calibration(&first_sample, &active_calibration);
    s_previous_corrected_valid = true;
    uint32_t last_dashboard_ms = 0;

    while (true) {
        motor_vis_accel_sample_t sample = {};
        if (!accel_demo_wait_for_next_sample(&last_sequence, &sample, ACCEL_DEMO_SAMPLE_WAIT_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "no new accelerometer samples received for %d ms", ACCEL_DEMO_SAMPLE_WAIT_TIMEOUT_MS);
            continue;
        }

        accel_demo_vector_mg_t corrected = accel_demo_apply_calibration(&sample, &active_calibration);
        float corrected_mag_mg = accel_demo_vector_mag_mg(&corrected);
        accel_demo_axis_check_t axis_check = accel_demo_calculate_axis_check(&corrected);
        accel_demo_validation_result_t validation = accel_demo_calculate_validation_result(&corrected, corrected_mag_mg);

        // Previous running-velocity behavior used one long-lived accumulator and updated it each sample.
        // TODO: Testing Code - The speed estimate was useful for learning why accelerometer-only speed
        // drifts, but production jacket behavior uses GNSS for speed and the ADXL345 for impact evidence.
        // velocity.vx_mps += linear_accel.x_mps2 * dt_s;
        // velocity.vy_mps += linear_accel.y_mps2 * dt_s;
        // velocity.vz_mps += linear_accel.z_mps2 * dt_s;

        if (last_dashboard_ms == 0 || sample.timestamp_ms - last_dashboard_ms >= ACCEL_DEMO_DASHBOARD_PERIOD_MS) {
            last_dashboard_ms = sample.timestamp_ms;
            accel_demo_print_dashboard(
                &sample,
                &active_calibration,
                &corrected,
                &axis_check,
                &validation
            );
        }

        // The dashboard writes a lot of serial text. Yielding here keeps the ESP32 idle task running
        // so the watchdog does not interpret terminal output as a stuck main task.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
