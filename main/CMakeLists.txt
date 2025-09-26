# main/CMakeLists.txt - Fixed linking for ESP-IDF 5.5

idf_component_register(
    SRCS 
        "main.c"
        "src/gap.c"
        "src/gatt_svc.c"
        "src/sensor_task.c"
        "src/dfu_service.c"
        "src/battery.c"
        "src/i2c_shim.c"
    INCLUDE_DIRS 
        "."
        "include"
    REQUIRES 
        nvs_flash
        bt
        driver
        esp_timer
        app_update
        imu
        # Add these for NimBLE linking
        freertos
        esp_hw_support
        esp_pm
        hal
    PRIV_REQUIRES
        esp_system
)

# Add ALL NimBLE include directories for ESP-IDF 5.5
idf_build_get_property(idf_path IDF_PATH)

target_include_directories(${COMPONENT_LIB} PUBLIC
    # Core NimBLE paths
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/include"
    "${idf_path}/components/bt/host/nimble/nimble/porting/nimble/include"
    "${idf_path}/components/bt/host/nimble/nimble/porting/npl/freertos/include"
    "${idf_path}/components/bt/host/nimble/port/include"
    
    # Transport layer
    "${idf_path}/components/bt/host/nimble/nimble/nimble/transport/include"
    
    # Services
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/gap/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/gatt/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/ans/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/bas/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/dis/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/services/ias/include"
    
    # Utilities and stores
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/util/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/store/config/include"
    "${idf_path}/components/bt/host/nimble/nimble/nimble/host/store/ram/include"
    
    # ESP specific
    "${idf_path}/components/bt/host/nimble/esp-hci/include"
)

# Force link NimBLE libraries
idf_component_get_property(bt_lib bt COMPONENT_LIB)
target_link_libraries(${COMPONENT_LIB} PUBLIC ${bt_lib})
