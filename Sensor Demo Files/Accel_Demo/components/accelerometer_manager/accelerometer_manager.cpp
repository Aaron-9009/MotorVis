#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// Production integration originally replaced the legacy driver/i2c.h path with the ESP-IDF 5.5 master-bus API.
// #include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "accelerometer_manager.h"

// Hardware configuration for the ADXL345 module being tested by this standalone demo.
#define MOTOR_VIS_ACCEL_I2C_PORT          I2C_NUM_0
#define MOTOR_VIS_ACCEL_I2C_SDA_PIN       21
#define MOTOR_VIS_ACCEL_I2C_SCL_PIN       22
#define MOTOR_VIS_ACCEL_I2C_FREQ_HZ       400000
#define MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS    10

// ADXL345 default 7-bit I2C address when the ALT ADDRESS pin is not pulled high.
#define MOTOR_VIS_ADXL345_ADDR            0x53

// ADXL345 register map entries used during setup and sample reads.
#define MOTOR_VIS_ADXL345_REG_BW_RATE     0x2C
#define MOTOR_VIS_ADXL345_REG_POWER_CTL   0x2D
#define MOTOR_VIS_ADXL345_REG_DATA_FORMAT 0x31
#define MOTOR_VIS_ADXL345_REG_DATAX0      0x32

// Register values chosen for the first accelerometer hardware bring-up.
#define MOTOR_VIS_ADXL345_MEASURE         0x08
#define MOTOR_VIS_ADXL345_RANGE_2G        0x00
#define MOTOR_VIS_ADXL345_ODR_100HZ       0x0A

// In +/-2g mode the ADXL345 reports about 4 mg per least-significant bit.
#define MOTOR_VIS_ADXL345_MG_PER_LSB      4

// The manager samples at 10 Hz so the terminal demo updates at the same speed as production publishing.
#define MOTOR_VIS_ACCEL_SAMPLE_INTERVAL_MS 100

// V1 alert threshold compares the change between consecutive samples, not a full crash-validation model yet.
#define MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG 500

#define MOTOR_VIS_ACCEL_TASK_STACK_SIZE    3072
#define MOTOR_VIS_ACCEL_TASK_PRIORITY      4

// Shared module state owned by the manager task and protected before app_main() reads it.
static const char *TAG = "ACCEL_MANAGER";

static SemaphoreHandle_t s_accel_mutex = NULL;
static TaskHandle_t s_accel_task_handle = NULL;
static bool s_accel_initialized = false;
static motor_vis_accel_sample_t s_latest_sample = {};

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_adxl345_handle = NULL;

static int16_t s_prev_ax_mg = 0;
static int16_t s_prev_ay_mg = 0;
static int16_t s_prev_az_mg = 0;
static bool s_first_sample = true;

// Uses the ESP timer as the shared millisecond clock for sample timestamps.
static uint32_t motor_vis_accel_now_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Releases I2C resources if initialization fails partway through setup.
static void motor_vis_accel_cleanup_i2c(void)
{
    if (s_adxl345_handle != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(s_adxl345_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to remove ADXL345 I2C device during cleanup: %s", esp_err_to_name(err));
        }
        s_adxl345_handle = NULL;
    }

    if (s_i2c_bus_handle != NULL) {
        esp_err_t err = i2c_del_master_bus(s_i2c_bus_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to delete accelerometer I2C bus during cleanup: %s", esp_err_to_name(err));
        }
        s_i2c_bus_handle = NULL;
    }
}

// Writes one ADXL345 register through the ESP-IDF 5.5 master-bus I2C API.
static esp_err_t motor_vis_adxl345_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};

    // The legacy i2c_master_write_to_device() call is intentionally not used in this demo.
    return i2c_master_transmit(
        s_adxl345_handle,
        buf,
        sizeof(buf),
        MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS
    );
}

// Reads a consecutive ADXL345 register block, starting with the register address byte.
static esp_err_t motor_vis_adxl345_read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    // The legacy i2c_master_write_read_device() call is intentionally not used in this demo.
    return i2c_master_transmit_receive(
        s_adxl345_handle,
        &start_reg,
        1,
        buf,
        len,
        MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS
    );
}

// Converts the ADXL345 little-endian raw X/Y/Z registers into milli-g values.
static esp_err_t motor_vis_adxl345_read_xyz(int16_t *ax_mg, int16_t *ay_mg, int16_t *az_mg)
{
    uint8_t raw[6] = {0};
    esp_err_t err = motor_vis_adxl345_read_regs(MOTOR_VIS_ADXL345_REG_DATAX0, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t raw_x = static_cast<int16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));
    int16_t raw_y = static_cast<int16_t>(raw[2] | (static_cast<uint16_t>(raw[3]) << 8));
    int16_t raw_z = static_cast<int16_t>(raw[4] | (static_cast<uint16_t>(raw[5]) << 8));

    *ax_mg = static_cast<int16_t>(raw_x * MOTOR_VIS_ADXL345_MG_PER_LSB);
    *ay_mg = static_cast<int16_t>(raw_y * MOTOR_VIS_ADXL345_MG_PER_LSB);
    *az_mg = static_cast<int16_t>(raw_z * MOTOR_VIS_ADXL345_MG_PER_LSB);

    return ESP_OK;
}

// Stores the latest sample behind a mutex so app_main() never reads a partially updated value.
static void motor_vis_accel_store_sample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg, uint8_t alert)
{
    if (s_accel_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_accel_mutex, portMAX_DELAY) == pdTRUE) {
        s_latest_sample.timestamp_ms = motor_vis_accel_now_ms();
        s_latest_sample.ax_mg = ax_mg;
        s_latest_sample.ay_mg = ay_mg;
        s_latest_sample.az_mg = az_mg;
        s_latest_sample.alert = alert;
        s_latest_sample.sequence += 1U;
        xSemaphoreGive(s_accel_mutex);
    }
}

// Background task that continuously samples the ADXL345 and updates the shared latest-sample struct.
static void motor_vis_accel_task(void *arg)
{
    (void) arg;
    const TickType_t delay = pdMS_TO_TICKS(MOTOR_VIS_ACCEL_SAMPLE_INTERVAL_MS);

    while (true) {
        int16_t ax_mg = 0;
        int16_t ay_mg = 0;
        int16_t az_mg = 0;

        esp_err_t err = motor_vis_adxl345_read_xyz(&ax_mg, &ay_mg, &az_mg);

        if (err == ESP_OK) {
            uint8_t alert = 0U;

            if (!s_first_sample) {
                int32_t dx = static_cast<int32_t>(ax_mg) - static_cast<int32_t>(s_prev_ax_mg);
                int32_t dy = static_cast<int32_t>(ay_mg) - static_cast<int32_t>(s_prev_ay_mg);
                int32_t dz = static_cast<int32_t>(az_mg) - static_cast<int32_t>(s_prev_az_mg);
                int32_t delta_sq = dx * dx + dy * dy + dz * dz;
                int32_t limit_sq = static_cast<int32_t>(MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG) *
                                   static_cast<int32_t>(MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG);

                if (delta_sq > limit_sq) {
                    alert = 1U;
                    ESP_LOGW(
                        TAG,
                        "accelerometer v1 alert: dx=%ld dy=%ld dz=%ld ax=%d ay=%d az=%d",
                        (long) dx,
                        (long) dy,
                        (long) dz,
                        (int) ax_mg,
                        (int) ay_mg,
                        (int) az_mg
                    );
                }
            } else {
                s_first_sample = false;
            }

            s_prev_ax_mg = ax_mg;
            s_prev_ay_mg = ay_mg;
            s_prev_az_mg = az_mg;

            motor_vis_accel_store_sample(ax_mg, ay_mg, az_mg, alert);
        } else {
            ESP_LOGW(TAG, "ADXL345 I2C read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(delay);
    }
}

extern "C" esp_err_t motor_vis_accel_manager_init(void)
{
    if (s_accel_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = MOTOR_VIS_ACCEL_I2C_PORT;
    bus_config.sda_io_num = static_cast<gpio_num_t>(MOTOR_VIS_ACCEL_I2C_SDA_PIN);
    bus_config.scl_io_num = static_cast<gpio_num_t>(MOTOR_VIS_ACCEL_I2C_SCL_PIN);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t device_config = {};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = MOTOR_VIS_ADXL345_ADDR;
    device_config.scl_speed_hz = MOTOR_VIS_ACCEL_I2C_FREQ_HZ;

    err = i2c_master_bus_add_device(s_i2c_bus_handle, &device_config, &s_adxl345_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 I2C device add failed: %s", esp_err_to_name(err));
        motor_vis_accel_cleanup_i2c();
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_BW_RATE, MOTOR_VIS_ADXL345_ODR_100HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 BW_RATE write failed: %s", esp_err_to_name(err));
        motor_vis_accel_cleanup_i2c();
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_DATA_FORMAT, MOTOR_VIS_ADXL345_RANGE_2G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 DATA_FORMAT write failed: %s", esp_err_to_name(err));
        motor_vis_accel_cleanup_i2c();
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_POWER_CTL, MOTOR_VIS_ADXL345_MEASURE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 POWER_CTL write failed: %s", esp_err_to_name(err));
        motor_vis_accel_cleanup_i2c();
        return err;
    }

    s_accel_mutex = xSemaphoreCreateMutex();
    if (s_accel_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create accel sample mutex");
        motor_vis_accel_cleanup_i2c();
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_result = xTaskCreate(
        motor_vis_accel_task,
        "motor_vis_accel",
        MOTOR_VIS_ACCEL_TASK_STACK_SIZE,
        NULL,
        MOTOR_VIS_ACCEL_TASK_PRIORITY,
        &s_accel_task_handle
    );
    if (task_result != pdPASS) {
        vSemaphoreDelete(s_accel_mutex);
        s_accel_mutex = NULL;
        motor_vis_accel_cleanup_i2c();
        return ESP_ERR_NO_MEM;
    }

    s_accel_initialized = true;
    ESP_LOGI(
        TAG,
        "ADXL345 initialized: I2C port=%d SDA=GPIO%d SCL=GPIO%d addr=0x%02X range=+/-2g ODR=100Hz crash_threshold=%dmg",
        (int) MOTOR_VIS_ACCEL_I2C_PORT,
        (int) MOTOR_VIS_ACCEL_I2C_SDA_PIN,
        (int) MOTOR_VIS_ACCEL_I2C_SCL_PIN,
        MOTOR_VIS_ADXL345_ADDR,
        MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG
    );
    return ESP_OK;
}

extern "C" bool motor_vis_accel_manager_get_latest_sample(motor_vis_accel_sample_t *out_sample)
{
    if (!s_accel_initialized || out_sample == NULL || s_accel_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_accel_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    std::memcpy(out_sample, &s_latest_sample, sizeof(*out_sample));
    xSemaphoreGive(s_accel_mutex);
    return true;
}
