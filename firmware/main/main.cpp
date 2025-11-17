//////////////////////////////////////////////////////////////////////

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <sys/time.h>
#include <unistd.h>
#include <sys/select.h>
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

    display_t display_temp1;
    display_t display_temp2;

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

    bool full_brightness = false;

    int frames = 0;
    while(true) {
        display_t &display = display_update();
        button_get(buttons);

        if(buttons[0].pressed) {
            frames = 0;
        }

        if(buttons[2].pressed) {
            full_brightness = !full_brightness;
        }

        int x = 30 - frames / 2;

        int ambient = update_ambient();

        if(x >= -boot_msg_width || full_brightness) {
            ambient = 255;
        }
        display.set_ambient(ambient);

        graphics_t gfx(display);

        gfx.cls(0);

        if(x >= -(boot_msg_width + 20)) {
            gfx.draw_string(boot_msg, x, 0, 2047);
        } else {
            // To make the minutes fade from one to the next:
            // Create a fake time, one second ahead
            // Draw both WITHOUT gamma correction
            // Then fade between them based on fraction of current second
            // Then gamma correct into current buffer
            struct timeval tv_now;
            get_time(&tv_now);
            int hours = tv_now.tv_sec / 3600;
            int hour_seconds = tv_now.tv_sec % 3600;
            int minutes = hour_seconds / 60;

            int seconds = hour_seconds % 60;
            gfx.draw_seconds(seconds, tv_now.tv_usec);

            int hours2 = (tv_now.tv_sec + 1) / 3600;
            int hour_seconds2 = (tv_now.tv_sec + 1) % 3600;
            int minutes2 = hour_seconds2 / 60;

            graphics_t gfx_temp1(display_temp1);
            graphics_t gfx_temp2(display_temp2);

            gfx_temp1.cls(0);
            gfx_temp2.cls(0);

            gfx_temp1.draw_time(hours, minutes, 2047, 2047);
            gfx_temp2.draw_time(hours2, minutes2, 2047, 2047);

            // fade between current and next time
            int scale = tv_now.tv_usec / (1000000 / 3200);
            if(scale > 2048) {
                scale = 2048;
            }
            gfx.fade_matrix(gfx_temp1, gfx_temp2, scale);
        }
        frames += 1;
    }
}