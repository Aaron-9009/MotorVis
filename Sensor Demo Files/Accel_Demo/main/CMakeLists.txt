idf_component_register(
    SRCS "accel_demo_main.cpp" "accel_calibration_data.cpp"
    INCLUDE_DIRS "."
    REQUIRES accelerometer_manager esp_timer log
)
