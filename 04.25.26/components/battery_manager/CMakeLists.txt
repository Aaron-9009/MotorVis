idf_component_register(
    SRCS "battery_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES arduino-esp32 esp_timer log
)
