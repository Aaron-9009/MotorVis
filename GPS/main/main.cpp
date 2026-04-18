#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "TinyGPS++.h"

#define UART_PORT UART_NUM_2
#define TX 33
#define RX 32

TinyGPSPlus gps;

void gps_init() {
    uart_config_t uart_config{};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    uart_config.flags.backup_before_sleep = 0;

    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void gps_read_task(void *arg) {
    uint8_t data[128];
    double lat;
    double lng;

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(100));

        for (int i = 0; i < len; i++) {
            gps.encode((char)data[i]);
        }

        if (gps.location.isUpdated()) {
            if (gps.location.isValid()) {
                lat = gps.location.lat();
                lng = gps.location.lng();
                printf("Latitude: %.6f, Longitude: %.6f\n", lat, lng);


            } else {
                printf("GPS data received, but no valid fix yet.\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void) {
    gps_init();
    xTaskCreate(gps_read_task, "gps", 4096, NULL, 5, NULL);
}