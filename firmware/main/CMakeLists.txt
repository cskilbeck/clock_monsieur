idf_component_register(
    SRCS
        main.cpp
        display.cpp
        graphics.cpp
        lux.cpp
        button.cpp
        wifi.cpp
        clock_time.cpp
        settings.cpp
        ota.cpp
        util.cpp
        http.cpp
        state.cpp
        console.cpp
        ping.cpp
        timezone.cpp
        timezone_data.cpp
        font_5x7.cpp
        font_5x7_narrow.cpp
        font_5x6.cpp
        square_font.cpp
        big_caps.cpp
        menu.cpp
        timezone_data.cpp
    PRIV_REQUIRES
        driver
        bt
        nvs_flash
        mbedtls
        wifi_provisioning
        json
        app_update
        esp_http_client
    INCLUDE_DIRS
        .
)
if (CONFIG_PROJECT_BUILD_DEBUG)
    # Define the macro 'DEBUG' for all C/C++ files in your components
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "DEBUG")
    message(STATUS "Build Configuration: DEBUG is defined")
else()
    message(STATUS "Build Configuration: DEBUG is NOT defined")
endif()


target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -Wno-format-truncation
        -Wno-missing-field-initializers
        -Wno-unused-variable)
