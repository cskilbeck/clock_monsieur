//////////////////////////////////////////////////////////////////////

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <sys/time.h>
#include <unistd.h>
#include <sys/select.h>
#include <freertos/FreeRTOS.h>

#include "esp_task_wdt.h"
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
#include "settings.h"
#include "sdkconfig.h"
#include "version.h"
#include "ota.h"
#include "state.h"

LOG_CONTEXT("main");

namespace
{
    char const constexpr boot_msg[] = "Clock Monsieur!";

    float smoothed_ambient = 0.0f;
    float smooth_factor = 0.01f;

    int update_ambient()
    {
        // ambient light response
        float ambient = (float)lux_get() * (1.0f / 65535);

        smoothed_ambient = smooth_factor * ambient + (1.0f - smooth_factor) * smoothed_ambient;

        // ghetto inverse gamma ramp
        // float t = 1.0f - smoothed_ambient;
        // t = 1.0f - t * t * t;
        float t = smoothed_ambient;

        // scale lux to two 7 bit numbers
        int base, max;
        switch(settings.brightness_mode) {
        default:
        case brightness_mode_t::high:
            base = 6;
            max = 255;
            break;
        case brightness_mode_t::medium:
            base = 3;
            max = 192;
            break;
        case brightness_mode_t::low:
            base = 1;
            max = 128;
            break;
        }
        int range = max - base;
        return (int)(t * range) + base;
    }

    void console_read_task(void *pvParameter)
    {
        delay_ms(200);

        char cmd[16];
        int cmdlen = 0;

        char buffer[16];
        while(true) {
            if(fgets(buffer, sizeof(buffer), stdin) != NULL) {
                int l = strlen(buffer);
                bool enter = buffer[l - 1] == '\n';
                if(enter) {
                    l -= 1;
                }
                int cmd_remain = sizeof(cmd) - cmdlen - 1;
                if(l >= cmd_remain) {
                    l = cmd_remain;
                }
                memcpy(cmd + cmdlen, buffer, l);
                cmdlen += l;
                cmd[cmdlen] = 0;
                if(enter) {
                    LOG_INFO("CMD: \"%s\" (%d)", cmd, cmdlen);
                    cmdlen = 0;
                }
            }
            delay_ms(100);
        }
    }

}    // namespace

extern "C" void app_main()
{
    // disable brownout detector
    // REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);
    // REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);

    LOG_INFO("----- CLOCK MONSIEUR, FIRMWARE VERSION: %s -----", VERSION_STR);

    button_init();

    bool reset_nvs = buttons[BUTTON_UP].held && buttons[BUTTON_DOWN].held;

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND || reset_nvs) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);

    lux_init();
    display_init();

    provisioning_init(buttons[2].held);

    xTaskCreatePinnedToCore(clock_time_task, "clock_time", 1024 * 6, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(console_read_task, "console", 2048, NULL, 1, NULL, 0);

    // got this far, probably fine...
    ota_mark_app_valid();

    bool full_brightness = false;

    state_init();
    state_set(boot_state);

    while(true) {
        display_update();
        button_update();

        if(button_down.pressed) {
            full_brightness = !full_brightness;
        }

        int ambient = update_ambient();

        if(full_brightness) {
            ambient = 255;
        }
        display->set_ambient(ambient);

        state_update();

        display->update_ambient();

        if(button_left.pressed) {
            char buffer[512];
            vTaskList(buffer);
            printf("%s\n", buffer);
            auto x = esp_get_minimum_free_heap_size();
            printf("HEAP: %lu\n", x);
        }
    }
}
