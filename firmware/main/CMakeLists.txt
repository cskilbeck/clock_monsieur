idf_component_register(
    SRCS
        main.cpp
        display.cpp
        font.cpp
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
        menu.cpp
        timezone_data.cpp
        buzzer.cpp
        fonts/font_5x7.cpp
        fonts/font_5x7_narrow_modern.cpp
        fonts/big_caps.cpp
        fonts/square_font.cpp
    PRIV_REQUIRES
        driver
        esp_driver_rmt
        bt
        nvs_flash
        mbedtls
        wifi_provisioning
        json
        app_update
        esp_http_client
        esp_adc
    INCLUDE_DIRS
        . fonts/
)

if (CONFIG_PROJECT_BUILD_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "DEBUG")
    message(STATUS "Build Configuration DEBUG")
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "NDEBUG")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "RELEASE")
    message(STATUS "Build Configuration RELEASE")
endif()

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -Wno-format-truncation
        -Wno-missing-field-initializers
        -Wno-unused-variable)
