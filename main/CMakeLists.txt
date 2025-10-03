idf_component_register(
    SRCS
        main.cpp
        tlc_5948.cpp
        i2c_task.cpp
        veml_3235.cpp
        mcp_4017.cpp
        high_side.cpp
    PRIV_REQUIRES
        driver
        log
        freertos
        spi_flash
    INCLUDE_DIRS
        .
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -Wno-unused-variable)