# Heart sensor manager keeps Arduino/PulseSensor details isolated from the C GATT server.
idf_component_register(
    SRCS "heart_sensor_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        arduino-esp32
        esp_timer
        log
        PulseSensorPlayground
)

# TODO: Testing Code
# idf_component_register(
#     SRCS "heart_sensor_manager.cpp"
#     INCLUDE_DIRS "include"
#     REQUIRES
#         esp_adc
#         esp_timer
#         log
# )
# Previous active build configuration used the ESP-IDF native ADC fallback so the manager could build on IDF 6.0.
