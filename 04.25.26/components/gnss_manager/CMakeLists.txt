idf_component_register(
    SRCS "gnss_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_timer log TinyGPSPlus-ESPIDF
)
