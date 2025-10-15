idf_component_register(
    SRCS
        main.cpp
        display.cpp
        lux.cpp
        wifi.cpp
    PRIV_REQUIRES
        driver
        log
        esp_wifi
        nvs_flash
        freertos
        spi_flash
    INCLUDE_DIRS
        .
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -Wno-unused-variable)