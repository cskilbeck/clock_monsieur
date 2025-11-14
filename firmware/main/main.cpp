//////////////////////////////////////////////////////////////////////

#include <cmath>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "util.h"
#include "display.h"
#include "graphics.h"
#include "provisioning.h"
#include "lux.h"
#include "button.h"
#include "clock_time.h"

#include "sdkconfig.h"

LOG_CONTEXT("main");

namespace
{
    char const boot_msg[] = "Clock Monsieur!";

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
        int base = 3;
        int max = 255;
        int range = max - base;
        return (int)(t * range) + base;
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

    int boot_msg_width = measure_string(boot_msg);

    int frames = 0;
    while(true) {
        display_t &display = display_update();
        button_get(buttons);

        if(buttons[0].pressed) {
            frames = 0;
        }

        int x = 30 - frames / 2;

        int b;
        if(x >= -boot_msg_width) {
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

        graphics_t gfx(display);

        gfx.cls(0);

        if(x >= -(boot_msg_width + 20)) {
            gfx.draw_string(boot_msg, x, 0, 2047);
        } else {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int hours = tv_now.tv_sec / 3600;
            int hour = tv_now.tv_sec % 3600;
            int minutes = hour / 60;
            int seconds = hour % 60;
            gfx.cls(0);
            gfx.draw_time(hours, minutes, gamma_get(2000), gamma_get(2000));
            gfx.seconds_tail(seconds, tv_now.tv_usec);
        }
        frames += 1;
    }
}