# TODO: Testing Code
# idf_component_register(
#     SRCS "Motor_Vis.c"
#     INCLUDE_DIRS "."
#     REQUIRES bt nvs_flash esp_driver_gpio
# )
# Previous component registration only built the BLE test source before the shared UUID module
# and GNSS manager component were integrated.

# This registration builds the main BLE firmware together with the shared UUID source
# and the manager components that feed live sensor data into the GATT server.
# TODO: Testing Code
# idf_component_register(
#     SRCS "Motor_Vis.c" "motor_vis_ble_uuids.c"
#     INCLUDE_DIRS "."
#     REQUIRES bt nvs_flash esp_driver_gpio gnss_manager
# )
# Previous active component registration built the BLE main source, shared UUID source,
# and GNSS manager before the external jacket button and LED validation component existed.

# jacket_controls keeps the button and discrete LED logic out of Motor_Vis.c,
# heart_sensor_manager keeps the Arduino pulse sensor code behind a simple C-style API,
# and battery_manager keeps Feather battery voltage measurement out of the BLE server.
idf_component_register(
    SRCS "Motor_Vis.c" "motor_vis_ble_uuids.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_gpio gnss_manager jacket_controls heart_sensor_manager battery_manager
)
