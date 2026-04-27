#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "accelerometer_manager.h"

//Hardware Config

// I2C bus wiring
#define MOTOR_VIS_ACCEL_I2C_PORT         I2C_NUM_0
#define MOTOR_VIS_ACCEL_I2C_SDA_PIN      21
#define MOTOR_VIS_ACCEL_I2C_SCL_PIN      22
#define MOTOR_VIS_ACCEL_I2C_FREQ_HZ      400000
#define MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS   10

#define MOTOR_VIS_ADXL345_ADDR           0x53

// ADXL345 Register Map 
#define MOTOR_VIS_ADXL345_REG_BW_RATE     0x2C   // Output data rate control
#define MOTOR_VIS_ADXL345_REG_POWER_CTL   0x2D   // Power mode control
#define MOTOR_VIS_ADXL345_REG_DATA_FORMAT 0x31   // Range and resolution config
#define MOTOR_VIS_ADXL345_REG_DATAX0      0x32   

// Register values
#define MOTOR_VIS_ADXL345_MEASURE         0x08   
#define MOTOR_VIS_ADXL345_RANGE_2G        0x00   
#define MOTOR_VIS_ADXL345_ODR_100HZ       0x0A  

#define MOTOR_VIS_ADXL345_MG_PER_LSB      4

// Sampling 
#define MOTOR_VIS_ACCEL_SAMPLE_INTERVAL_MS   100


#define MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG   500

#define MOTOR_VIS_ACCEL_TASK_STACK_SIZE      3072
#define MOTOR_VIS_ACCEL_TASK_PRIORITY        4

// Shared State
static const char *TAG = "ACCEL_MANAGER";

static SemaphoreHandle_t        s_accel_mutex       = NULL;
static TaskHandle_t             s_accel_task_handle = NULL;
static bool                     s_accel_initialized = false;
static motor_vis_accel_sample_t s_latest_sample     = {};

static int16_t s_prev_ax_mg    = 0;
static int16_t s_prev_ay_mg    = 0;
static int16_t s_prev_az_mg    = 0;
static bool    s_first_sample  = true;


static uint32_t motor_vis_accel_now_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

static TickType_t motor_vis_accel_ms_to_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return (ticks == 0) ? 1 : ticks;
}

static esp_err_t motor_vis_adxl345_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(
        MOTOR_VIS_ACCEL_I2C_PORT,
        MOTOR_VIS_ADXL345_ADDR,
        buf,
        sizeof(buf),
        motor_vis_accel_ms_to_ticks(MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS)
    );
}

static esp_err_t motor_vis_adxl345_read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(
        MOTOR_VIS_ACCEL_I2C_PORT,
        MOTOR_VIS_ADXL345_ADDR,
        &start_reg,
        1,
        buf,
        len,
        motor_vis_accel_ms_to_ticks(MOTOR_VIS_ACCEL_I2C_TIMEOUT_MS)
    );
}

static esp_err_t motor_vis_adxl345_read_xyz(int16_t *ax_mg, int16_t *ay_mg, int16_t *az_mg)
{
    uint8_t raw[6] = {0};
    esp_err_t err = motor_vis_adxl345_read_regs(MOTOR_VIS_ADXL345_REG_DATAX0, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    // ADXL345 little endian storage
    int16_t raw_x = static_cast<int16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));
    int16_t raw_y = static_cast<int16_t>(raw[2] | (static_cast<uint16_t>(raw[3]) << 8));
    int16_t raw_z = static_cast<int16_t>(raw[4] | (static_cast<uint16_t>(raw[5]) << 8));

    *ax_mg = static_cast<int16_t>(raw_x * MOTOR_VIS_ADXL345_MG_PER_LSB);
    *ay_mg = static_cast<int16_t>(raw_y * MOTOR_VIS_ADXL345_MG_PER_LSB);
    *az_mg = static_cast<int16_t>(raw_z * MOTOR_VIS_ADXL345_MG_PER_LSB);

    return ESP_OK;
}

static void motor_vis_accel_store_sample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg, uint8_t alert)
{
    if (s_accel_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_accel_mutex, portMAX_DELAY) == pdTRUE) {
        s_latest_sample.timestamp_ms = motor_vis_accel_now_ms();
        s_latest_sample.ax_mg        = ax_mg;
        s_latest_sample.ay_mg        = ay_mg;
        s_latest_sample.az_mg        = az_mg;
        s_latest_sample.alert        = alert;
        s_latest_sample.sequence    += 1U;
        xSemaphoreGive(s_accel_mutex);
    }
}

static void motor_vis_accel_task(void *arg)
{
    const TickType_t delay = motor_vis_accel_ms_to_ticks(MOTOR_VIS_ACCEL_SAMPLE_INTERVAL_MS);

    while (true) {
        int16_t ax_mg = 0;
        int16_t ay_mg = 0;
        int16_t az_mg = 0;

        esp_err_t err = motor_vis_adxl345_read_xyz(&ax_mg, &ay_mg, &az_mg);

        if (err == ESP_OK) {
            uint8_t alert = 0U;

            if (!s_first_sample) {
                int32_t dx       = static_cast<int32_t>(ax_mg) - static_cast<int32_t>(s_prev_ax_mg);
                int32_t dy       = static_cast<int32_t>(ay_mg) - static_cast<int32_t>(s_prev_ay_mg);
                int32_t dz       = static_cast<int32_t>(az_mg) - static_cast<int32_t>(s_prev_az_mg);
                int32_t delta_sq = dx * dx + dy * dy + dz * dz;
                int32_t limit_sq = static_cast<int32_t>(MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG) *
                                   static_cast<int32_t>(MOTOR_VIS_ACCEL_CRASH_THRESHOLD_MG);

                if (delta_sq > limit_sq) {
                    alert = 1U;
                    ESP_LOGW(
                        TAG,
                        "crash alert triggered: dx=%ld dy=%ld dz=%ld ax=%d ay=%d az=%d",
                        (long) dx, (long) dy, (long) dz,
                        (int) ax_mg, (int) ay_mg, (int) az_mg
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

    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = MOTOR_VIS_ACCEL_I2C_SDA_PIN;
    conf.scl_io_num       = MOTOR_VIS_ACCEL_I2C_SCL_PIN;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = MOTOR_VIS_ACCEL_I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(MOTOR_VIS_ACCEL_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(MOTOR_VIS_ACCEL_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_BW_RATE, MOTOR_VIS_ADXL345_ODR_100HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 BW_RATE write failed: %s", esp_err_to_name(err));
        i2c_driver_delete(MOTOR_VIS_ACCEL_I2C_PORT);
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_DATA_FORMAT, MOTOR_VIS_ADXL345_RANGE_2G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 DATA_FORMAT write failed: %s", esp_err_to_name(err));
        i2c_driver_delete(MOTOR_VIS_ACCEL_I2C_PORT);
        return err;
    }

    err = motor_vis_adxl345_write_reg(MOTOR_VIS_ADXL345_REG_POWER_CTL, MOTOR_VIS_ADXL345_MEASURE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 POWER_CTL write failed: %s", esp_err_to_name(err));
        i2c_driver_delete(MOTOR_VIS_ACCEL_I2C_PORT);
        return err;
    }

    s_accel_mutex = xSemaphoreCreateMutex();
    if (s_accel_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create accel sample mutex");
        i2c_driver_delete(MOTOR_VIS_ACCEL_I2C_PORT);
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
        i2c_driver_delete(MOTOR_VIS_ACCEL_I2C_PORT);
        return ESP_ERR_NO_MEM;
    }

    s_accel_initialized = true;
    ESP_LOGI(
        TAG,
        "ADXL345 initialized: I2C port=%d SDA=GPIO%d SCL=GPIO%d addr=0x%02X range=±2g ODR=100Hz crash_threshold=%dmg",
        MOTOR_VIS_ACCEL_I2C_PORT,
        MOTOR_VIS_ACCEL_I2C_SDA_PIN,
        MOTOR_VIS_ACCEL_I2C_SCL_PIN,
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
