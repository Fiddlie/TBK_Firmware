# components/imu/CMakeLists.txt

idf_component_register(
    SRCS 
        "imu.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver
    PRIV_REQUIRES
        esp_system
)