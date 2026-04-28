idf_component_register(
    SRCS "accelerometer_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_timer log
)
