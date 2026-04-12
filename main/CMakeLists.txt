idf_component_register(
    SRCS "Motor_Vis.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_gpio
)
