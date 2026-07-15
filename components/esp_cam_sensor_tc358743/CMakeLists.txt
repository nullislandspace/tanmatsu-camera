set(srcs)
set(include_dirs)
set(priv_include_dirs)

if(CONFIG_CAMERA_TC358743)
    list(APPEND srcs "sensors/tc358743/tc358743.c")
    list(APPEND include_dirs "sensors/tc358743/include")
    list(APPEND priv_include_dirs "sensors/tc358743/private_include")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    PRIV_INCLUDE_DIRS ${priv_include_dirs}
    REQUIRES esp_sccb_intf espressif__esp_cam_sensor
    PRIV_REQUIRES esp_driver_gpio esp_timer
)

# Force the linker to keep the ESP_CAM_SENSOR_DETECT_FN registration so
# the sensor is visible to the auto-detect loop. Without this the driver
# is dead-stripped because nothing else references its symbols.
if(CONFIG_CAMERA_TC358743_AUTO_DETECT_MIPI_INTERFACE_SENSOR)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u tc358743_detect")
endif()
