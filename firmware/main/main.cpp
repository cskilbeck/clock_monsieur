//////////////////////////////////////////////////////////////////////

#include <cmath>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "util.h"
#include "display.h"
#include "provisioning.h"
#include "lux.h"
#include "button.h"
#include "clock_time.h"

#include "sdkconfig.h"

LOG_CONTEXT("main");

namespace
{
    char const boot_msg[] = "     CLOCK MONSIEUR!      ";

    // approximate pow(2.2) for 11 bit fixed point: x^2 - (x^2 - x^3) / 4
    // and convert endianness for display buffer
    uint16_t gamma_get(uint16_t x)
    {
        int x2 = (x * x) >> 11;
        int x3 = (x * x2) >> 11;
        int x2_2 = x2 - ((x2 - x3) >> 2);
        return __builtin_bswap16(x2_2);
    }

    uint16_t gamma_getf(float x)
    {
        float x2 = x * x;
        float x3 = x2 * x;
        float x2_2 = x2 - ((x2 - x3) * 0.25f);
        return __builtin_bswap16(x2_2 * 2047.9f);
    }

    float smoothed_ambient = 0.0f;
    float smooth_factor = 0.01f;

    int update_ambient()
    {
        // ambient light response
        float ambient = (float)lux_get() * (1.0f / 65535);

        smoothed_ambient = smooth_factor * ambient + (1.0f - smooth_factor) * smoothed_ambient;

        // ghetto inverse gamma ramp
        float t = 1.0f - smoothed_ambient;
        t = 1.0f - t * t * t;

        // scale lux to two 7 bit numbers
        int base = 2;
        int max = 255;
        int range = max - base;
        return (int)(t * range) + base;
    }

    int constexpr TAIL_LENGTH = 59;
    int constexpr NUM_LEDS = 60;

    void seconds_tail(display_t &display, int current_second, int current_microseconds)
    {
        float T = (float)current_microseconds / 1000000.0f;

        for(int i = 0; i < NUM_LEDS; i++) {
            int D = (current_second - i + NUM_LEDS) % NUM_LEDS;
            float intensity = 0.0f;
            if(D == 0) {
                intensity = T;
            } else if(D >= 1 && D <= TAIL_LENGTH) {
                float x = (float)D - 1.0f + T;
                intensity = 1.0f - (x / TAIL_LENGTH);
            } else {
                intensity = 0.0f;
            }
            display.set_second(gamma_getf(intensity), i);
        }
    }

}    // namespace

extern "C" void app_main()
{
    // disable brownout detector
    REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);

    provisioning_init();

    xTaskCreate(clock_time_task, "clock_time", 4096, NULL, 5, NULL);

    lux_init();
    display_init();
    button_init();

    button_t buttons[NUM_BUTTONS] = {};

    int frames = 0;
    while(true) {
        display_t &display = display_update();
        button_get(buttons);

        int b;
        if(frames < sizeof(boot_msg) * 6) {
            b = 255;
        } else {
            b = update_ambient();
            // if((frames & 63) == 0) {
            //     LOG_INFO("%d", b);
            // }
        }
        if(buttons[2].held) {
            b = 255;
        }
        display.set_ambient(b);

        display.cls(0);

        if(frames < sizeof(boot_msg) * 6 + 30) {
            display.draw_string(boot_msg, 30 - frames, 0, 2047);
        } else {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int hours = tv_now.tv_sec / 3600;
            int hour = tv_now.tv_sec % 3600;
            int minutes = hour / 60;
            int seconds = hour % 60;
            display.cls(0);
            display.draw_time(hours, minutes, gamma_get(2000), gamma_get(2000));
            seconds_tail(display, seconds, tv_now.tv_usec);
        }
        frames += 1;
    }
}