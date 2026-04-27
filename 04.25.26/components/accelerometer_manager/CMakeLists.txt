idf_component_register(
    SRCS "accelerometer_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer log
)
