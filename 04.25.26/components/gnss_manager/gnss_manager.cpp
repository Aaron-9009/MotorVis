#include <cmath>
#include <cstdio>
#include <cstring>

// C headers are wrapped in extern "C" so this C++ component can call ESP-IDF and FreeRTOS APIs without name-mangling issues.
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "TinyGPS++.h"
#include "gnss_manager.h"

// --------------------------------------- GNSS UART CONFIGURATION ---------------------------------------
// GNSS manager logging tag for UART and parser status messages.
static const char *TAG = "GNSS_MANAGER";

// UART settings live in the GNSS manager so the BLE firmware can reuse the GPS parser without owning pin setup here.
#define MOTOR_VIS_GNSS_UART_PORT            UART_NUM_2
#define MOTOR_VIS_GNSS_TX_PIN               33
#define MOTOR_VIS_GNSS_RX_PIN               32
#define MOTOR_VIS_GNSS_BAUD_RATE            9600
#define MOTOR_VIS_GNSS_UART_BUFFER_SIZE     2048
#define MOTOR_VIS_GNSS_READ_CHUNK_SIZE      128
#define MOTOR_VIS_GNSS_TASK_STACK_SIZE      4096
#define MOTOR_VIS_GNSS_TASK_PRIORITY        5
#define MOTOR_VIS_GNSS_STATUS_LOG_INTERVAL_MS 5000
#define MOTOR_VIS_GNSS_NMEA_PREVIEW_LENGTH  96

// --------------------------------------- GNSS SHARED STATE ---------------------------------------
// TinyGPS++ parser instance now lives inside the GNSS manager instead of a standalone app_main().
static TinyGPSPlus s_gps;

// Mutex protects the latest fix while the UART read task and BLE code access it concurrently.
static SemaphoreHandle_t s_fix_mutex = NULL;
static TaskHandle_t s_gnss_task_handle = NULL;
static bool s_gnss_initialized = false;
static motor_vis_gnss_fix_t s_latest_fix{};
 
// Helper keeps all GNSS timestamps in the same millisecond time base used by the BLE packet.
static uint32_t motor_vis_gnss_now_ms(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Helper to convert floating-point coordinates into e7 integer format for BLE compatibility.
static int32_t motor_vis_gnss_to_e7(double coordinate)
{
    return static_cast<int32_t>(llround(coordinate * 10000000.0));
}

// Small helper copies the latest raw NMEA preview into a stable buffer for periodic diagnostic logs.
static void motor_vis_gnss_copy_text(char *destination, size_t destination_size, const char *source)
{
    if (destination == NULL || destination_size == 0 || source == NULL) {
        return;
    }

    std::strncpy(destination, source, destination_size - 1);
    destination[destination_size - 1] = '\0';
}

// Captures one printable NMEA sentence preview so GNSS bring-up logs can show what the UART stream looks like.
static void motor_vis_gnss_capture_sentence_preview(
    char incoming_char,
    char *active_sentence,
    size_t *active_sentence_length,
    char *latest_sentence,
    size_t latest_sentence_size
)
{
    if (active_sentence == NULL ||
        active_sentence_length == NULL ||
        latest_sentence == NULL ||
        latest_sentence_size == 0) {
        return;
    }

    if (incoming_char == '\r') {
        return;
    }

    if (incoming_char == '$') {
        *active_sentence_length = 0;
    }

    if (incoming_char == '\n') {
        if (*active_sentence_length > 0) {
            active_sentence[*active_sentence_length] = '\0';
            motor_vis_gnss_copy_text(latest_sentence, latest_sentence_size, active_sentence);
        }
        *active_sentence_length = 0;
        return;
    }

    if ((unsigned char) incoming_char < 32U || (unsigned char) incoming_char > 126U) {
        return;
    }

    if (*active_sentence_length < (MOTOR_VIS_GNSS_NMEA_PREVIEW_LENGTH - 1U)) {
        active_sentence[*active_sentence_length] = incoming_char;
        *active_sentence_length += 1U;
    }
}

// Formats the parsed UTC time so periodic status logs can show whether the GNSS module is decoding real timing data.
static void motor_vis_gnss_format_utc_time(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }

    if (s_gps.time.isValid()) {
        std::snprintf(
            buffer,
            buffer_size,
            "%02u:%02u:%02u",
            static_cast<unsigned int>(s_gps.time.hour()),
            static_cast<unsigned int>(s_gps.time.minute()),
            static_cast<unsigned int>(s_gps.time.second())
        );
    } else {
        motor_vis_gnss_copy_text(buffer, buffer_size, "n/a");
    }
}

// Formats HDOP text for logs without forcing the main read task to worry about TinyGPS++ field formatting.
static void motor_vis_gnss_format_hdop(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return;
    }

    if (s_gps.hdop.isValid()) {
        std::snprintf(buffer, buffer_size, "%.2f", s_gps.hdop.hdop());
    } else {
        motor_vis_gnss_copy_text(buffer, buffer_size, "n/a");
    }
}

// Periodic parser status log explains whether the module is silent, streaming bad UART data, or still waiting on a sky fix.
static void motor_vis_gnss_log_parser_status(uint32_t rx_bytes_since_log, const char *latest_sentence)
{
    char utc_time_text[16] = {0};
    char hdop_text[16] = {0};
    const char *nmea_preview = (latest_sentence != NULL && latest_sentence[0] != '\0') ? latest_sentence : "n/a";
    uint32_t satellites = s_gps.satellites.isValid() ? s_gps.satellites.value() : 0U;

    motor_vis_gnss_format_utc_time(utc_time_text, sizeof(utc_time_text));
    motor_vis_gnss_format_hdop(hdop_text, sizeof(hdop_text));

    if (s_gps.passedChecksum() == 0 && s_gps.failedChecksum() > 0) {
        ESP_LOGW(
            TAG,
            "GNSS UART data is arriving but NMEA validation is failing: rx_bytes=%lu chars=%lu passed_checksum=%lu failed_checksum=%lu satellites=%lu hdop=%s utc_time=%s last_nmea=\"%s\". Verify baud rate and signal integrity.",
            static_cast<unsigned long>(rx_bytes_since_log),
            static_cast<unsigned long>(s_gps.charsProcessed()),
            static_cast<unsigned long>(s_gps.passedChecksum()),
            static_cast<unsigned long>(s_gps.failedChecksum()),
            static_cast<unsigned long>(satellites),
            hdop_text,
            utc_time_text,
            nmea_preview
        );
    } else if (!s_gps.location.isValid()) {
        ESP_LOGW(
            TAG,
            "GNSS is receiving NMEA but a valid fix is not available yet: rx_bytes=%lu chars=%lu passed_checksum=%lu failed_checksum=%lu satellites=%lu hdop=%s utc_time=%s last_nmea=\"%s\". Move the module outdoors with a clear view of the sky if this persists.",
            static_cast<unsigned long>(rx_bytes_since_log),
            static_cast<unsigned long>(s_gps.charsProcessed()),
            static_cast<unsigned long>(s_gps.passedChecksum()),
            static_cast<unsigned long>(s_gps.failedChecksum()),
            static_cast<unsigned long>(satellites),
            hdop_text,
            utc_time_text,
            nmea_preview
        );
    } else {
        ESP_LOGI(
            TAG,
            "GNSS UART parser status: rx_bytes=%lu chars=%lu passed_checksum=%lu failed_checksum=%lu sentences_with_fix=%lu satellites=%lu hdop=%s utc_time=%s last_nmea=\"%s\"",
            static_cast<unsigned long>(rx_bytes_since_log),
            static_cast<unsigned long>(s_gps.charsProcessed()),
            static_cast<unsigned long>(s_gps.passedChecksum()),
            static_cast<unsigned long>(s_gps.failedChecksum()),
            static_cast<unsigned long>(s_gps.sentencesWithFix()),
            static_cast<unsigned long>(satellites),
            hdop_text,
            utc_time_text,
            nmea_preview
        );
    }
}

// Storage helper so every fix update consistently refreshes timestamp, validity, and sequence.
static bool motor_vis_gnss_store_fix(bool fix_valid, motor_vis_gnss_fix_t *stored_fix)
{
    if (s_fix_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_fix_mutex, portMAX_DELAY) == pdTRUE) {
        // Stores the capture time in milliseconds so BLE can publish when this fix snapshot was recorded.
        s_latest_fix.timestamp_ms = motor_vis_gnss_now_ms();
        s_latest_fix.fix_valid = fix_valid ? 1U : 0U;

        if (fix_valid) {
            s_latest_fix.latitude_e7 = motor_vis_gnss_to_e7(s_gps.location.lat());
            s_latest_fix.longitude_e7 = motor_vis_gnss_to_e7(s_gps.location.lng());
        } else {
            // Invalid fixes are stored as zeroed coordinates so the BLE packet still has a predictable layout.
            s_latest_fix.latitude_e7 = 0;
            s_latest_fix.longitude_e7 = 0;
        }

        // Increments the sequence for every stored update so BLE can detect and publish only new fixes.
        s_latest_fix.sequence += 1U;
        if (stored_fix != NULL) {
            // Copies the stored snapshot while the mutex is still held so diagnostic logs match what BLE will read.
            std::memcpy(stored_fix, &s_latest_fix, sizeof(*stored_fix));
        }
        xSemaphoreGive(s_fix_mutex);
        return true;
    }

    return false;
}

// GNSS read task continuously parses incoming UART bytes from the GPS module.
static void motor_vis_gnss_read_task(void *arg)
{
    uint8_t data[MOTOR_VIS_GNSS_READ_CHUNK_SIZE];
    char active_nmea_sentence[MOTOR_VIS_GNSS_NMEA_PREVIEW_LENGTH] = {0};
    char latest_nmea_sentence[MOTOR_VIS_GNSS_NMEA_PREVIEW_LENGTH] = {0};
    size_t active_nmea_sentence_length = 0;
    uint32_t bytes_since_status_log = 0;
    uint32_t last_rx_ms = 0;
    uint32_t last_uart_status_log_ms = 0;
    uint32_t last_no_data_log_ms = 0;
    bool saw_uart_activity = false;

    while (true) {
        // Pulls any available NMEA bytes from the GNSS UART without blocking forever if no data is ready yet.
        int len = uart_read_bytes( MOTOR_VIS_GNSS_UART_PORT, data, sizeof(data), pdMS_TO_TICKS(100) );
        uint32_t now_ms = motor_vis_gnss_now_ms();

        if (len > 0) {
            if (!saw_uart_activity) {
                // First-byte log confirms that the GPS module is physically reaching the MCU before a valid fix exists.
                ESP_LOGI(TAG, "GNSS UART activity detected on UART2; NMEA bytes are reaching the ESP32 parser");
                saw_uart_activity = true;
            }

            last_rx_ms = now_ms;
            bytes_since_status_log += static_cast<uint32_t>(len);

            if ((now_ms - last_uart_status_log_ms) >= MOTOR_VIS_GNSS_STATUS_LOG_INTERVAL_MS) {
                // Periodic parser status now differentiates between silent wiring, checksum trouble, and no-fix-yet field conditions.
                motor_vis_gnss_log_parser_status(bytes_since_status_log, latest_nmea_sentence);
                bytes_since_status_log = 0;
                last_uart_status_log_ms = now_ms;
            }
        } else if ((last_rx_ms == 0 || (now_ms - last_rx_ms) >= MOTOR_VIS_GNSS_STATUS_LOG_INTERVAL_MS) &&
                   (now_ms - last_no_data_log_ms) >= MOTOR_VIS_GNSS_STATUS_LOG_INTERVAL_MS) {
            // No-data logs are meant for hardware bring-up so wiring or baud-rate issues show up in the serial monitor.
            ESP_LOGW(
                TAG,
                "GNSS UART has not received data recently; verify module power, GNSS TX -> ESP32 GPIO32, common GND, and 9600 baud"
            );
            last_no_data_log_ms = now_ms;
        }

        // Feeds each byte into TinyGPS++ so complete NMEA sentences can be assembled across repeated UART reads.
        for (int i = 0; i < len; i++) {
            char incoming_char = static_cast<char>(data[i]);
            motor_vis_gnss_capture_sentence_preview(
                incoming_char,
                active_nmea_sentence,
                &active_nmea_sentence_length,
                latest_nmea_sentence,
                sizeof(latest_nmea_sentence)
            );
            s_gps.encode(incoming_char);
        }

        // Only refreshes the shared fix snapshot when TinyGPS++ reports that the decoded location changed.
        if (s_gps.location.isUpdated()) {
            bool fix_valid = s_gps.location.isValid();
            motor_vis_gnss_fix_t stored_fix{};

            if (motor_vis_gnss_store_fix(fix_valid, &stored_fix) && fix_valid) {
                // TODO: Testing Code
                // ESP_LOGI(
                //     TAG,
                //     "GNSS fix updated: lat=%ld lng=%ld seq=%lu",
                //     static_cast<long>(s_latest_fix.latitude_e7),
                //     static_cast<long>(s_latest_fix.longitude_e7),
                //     static_cast<unsigned long>(s_latest_fix.sequence)
                // );
                // Previous diagnostic log showed only scaled coordinates instead of the fields that feed the GPS GATT packet.
                ESP_LOGI(
                    TAG,
                    "GNSS fix stored for GPS packet: timestamp_ms=%lu latitude_e7=%ld longitude_e7=%ld fix_valid=%u sequence=%lu",
                    static_cast<unsigned long>(stored_fix.timestamp_ms),
                    static_cast<long>(stored_fix.latitude_e7),
                    static_cast<long>(stored_fix.longitude_e7),
                    static_cast<unsigned int>(stored_fix.fix_valid),
                    static_cast<unsigned long>(stored_fix.sequence)
                );
            } else {
                char utc_time_text[16] = {0};
                char hdop_text[16] = {0};
                uint32_t satellites = s_gps.satellites.isValid() ? s_gps.satellites.value() : 0U;

                motor_vis_gnss_format_utc_time(utc_time_text, sizeof(utc_time_text));
                motor_vis_gnss_format_hdop(hdop_text, sizeof(hdop_text));

                ESP_LOGI(
                    TAG,
                    "GNSS parser received a location update but the fix is still invalid: satellites=%lu hdop=%s utc_time=%s last_nmea=\"%s\"",
                    static_cast<unsigned long>(satellites),
                    hdop_text,
                    utc_time_text,
                    latest_nmea_sentence[0] != '\0' ? latest_nmea_sentence : "n/a"
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" esp_err_t motor_vis_gnss_init(void)
{
    if (s_gnss_initialized) {
        // Any repeated init requests are harmless in the integrated firmware.
        return ESP_OK;
    }

    // Mutex creation is before the read task starts updating shared fix data.
    s_fix_mutex = xSemaphoreCreateMutex();
    if (s_fix_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // UART framing and baud rate match the GNSS module used during bring-up.
    uart_config_t uart_config = {};
    uart_config.baud_rate = MOTOR_VIS_GNSS_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    uart_config.flags.backup_before_sleep = 0;

    // GNSS manager now owns the UART initialization that previously lived in the standalone GPS app.
    /*
    uart_num – UART port number, the max port number is (UART_NUM_MAX -1).
    rx_buffer_size – UART RX ring buffer size.
    tx_buffer_size – UART TX ring buffer size. If set to zero, driver will not use TX buffer, TX function will block task until all data have been sent out.
    queue_size – UART event queue size/depth.
    uart_queue – UART event queue handle (out param). On success, a new queue handle is written here to provide access to UART events. If set to NULL, driver will not use an event queue.
    intr_alloc_flags – Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info. Do not set ESP_INTR_FLAG_IRAM here (the driver's ISR handler is not located in IRAM)
    */
    // Installs the UART driver and receive buffer so the GNSS task has a live stream of incoming NMEA data.
    esp_err_t err = uart_driver_install(
        MOTOR_VIS_GNSS_UART_PORT,
        MOTOR_VIS_GNSS_UART_BUFFER_SIZE,
        0,
        0,
        NULL,
        0
    );

    // If the UART driver cannot start, tear the mutex back down and return the failure to the caller.
    if (err != ESP_OK) {
        vSemaphoreDelete(s_fix_mutex);
        s_fix_mutex = NULL;
        return err;
    }

    // Applies the UART framing and baud settings before the board pins are connected to the peripheral.
    err = uart_param_config(MOTOR_VIS_GNSS_UART_PORT, &uart_config);
    if (err != ESP_OK) {
        uart_driver_delete(MOTOR_VIS_GNSS_UART_PORT);
        vSemaphoreDelete(s_fix_mutex);
        s_fix_mutex = NULL;
        return err;
    }

    // Binds the configured UART peripheral to the GPS wiring used by the current Feather and GNSS module setup.
    err = uart_set_pin(
        MOTOR_VIS_GNSS_UART_PORT,
        MOTOR_VIS_GNSS_TX_PIN,
        MOTOR_VIS_GNSS_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (err != ESP_OK) {
        uart_driver_delete(MOTOR_VIS_GNSS_UART_PORT);
        vSemaphoreDelete(s_fix_mutex);
        s_fix_mutex = NULL;
        return err;
    }

    // Starts the long-running task that continuously reads UART bytes and refreshes the latest shared fix snapshot.
    // GNSS manager starts its own background read task so GPS parsing stays inside this component instead of becoming a second firmware entrypoint.
    BaseType_t task_result = xTaskCreate(
        motor_vis_gnss_read_task,
        "motor_vis_gnss",
        MOTOR_VIS_GNSS_TASK_STACK_SIZE,
        NULL,
        MOTOR_VIS_GNSS_TASK_PRIORITY,
        &s_gnss_task_handle
    );
    if (task_result != pdPASS) {
        uart_driver_delete(MOTOR_VIS_GNSS_UART_PORT);
        vSemaphoreDelete(s_fix_mutex);
        s_fix_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_gnss_initialized = true;
    ESP_LOGI(TAG, "GNSS manager initialized on UART2 (TX=%d RX=%d)", MOTOR_VIS_GNSS_TX_PIN, MOTOR_VIS_GNSS_RX_PIN);
    return ESP_OK;
}

extern "C" bool motor_vis_gnss_get_latest_fix(motor_vis_gnss_fix_t *out_fix)
{
    if (!s_gnss_initialized || out_fix == NULL || s_fix_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_fix_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    // Copies the full fix while the mutex is held so BLE readers never see a partially updated snapshot.
    std::memcpy(out_fix, &s_latest_fix, sizeof(*out_fix));
    xSemaphoreGive(s_fix_mutex);
    return true;
}
