idf_component_register(
    SRCS 
        src/gap.c
        src/gatt_svc.c
        src/sensor_task.c
        src/dfu_service.c
        main.c
    INCLUDE_DIRS "include"
    REQUIRES imu app_update
    PRIV_REQUIRES bt nvs_flash esp_driver_gpio spi_flash
)