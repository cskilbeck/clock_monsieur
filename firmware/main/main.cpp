//////////////////////////////////////////////////////////////////////

#include <cmath>

#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "util.h"
#include "display.h"
#include "wifi.h"
#include "lux.h"
#include "button.h"

LOG_CONTEXT("main");

#define PWM_MAX 2048
#define GAMMA_VALUE 2.2f    // Standard display gamma

namespace
{
    // approximate pow(2.2) for 11 bit fixed point
    // and convert endianness for display buffer
    uint16_t gamma_get(uint16_t x)
    {
        int x2 = (x * x) >> 11;
        int x3 = (x * x2) >> 11;
        return __builtin_bswap16(x2 - ((x2 - x3) >> 2));
    }

    int16_t a[256];
    int16_t b[256];

    int v()
    {
        return (rand() & 31) + 5;
    }

    float lux = 0.0f;

    void update_ambient(display_data_t &display)
    {
        // ambient light response
        float target = (float)lux_get();
        lux += (target - lux) * .01f;

        // ghetto inverse gamma ramp
        float constexpr recip = 1.0f / 65535.0f;
        float t = lux * recip;
        t = 1.0f - t;
        t = 1.0f - t * t * t;

        // scale lux to two 7 bit numbers
        int base = 1;
        int max = 255;
        int range = max - base;
        int b = (int)(t * range) + base;
        uint8_t c = (uint8_t)(b / 2);
        display.fcontrol.global_bc = c;
        uint8_t d = (uint8_t)((b + 1) / 2);
        for(int i = 0; i < 16; ++i) {
            display.fcontrol.set_dc(i, d);
        }
    }

}    // namespace

extern "C" void app_main()
{
    // disable brownout detector
    // REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);
    // REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);

    // start the wifi connecting

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);

    wifi_init();
    lux_init();
    display_init();
    button_init();

    button_t buttons[NUM_BUTTONS] = {};

    for(int i = 0; i < 256; ++i) {
        a[i] = 0;
        b[i] = v();
    }

    int frames = 0;
    int scroll = 0;
    while(true) {
        display_data_t &dd = display_update();
        button_get(buttons);

        update_ambient(dd);

        uint16_t *backbuffer = dd.grayscale_buffer;
#if 1
        if(buttons[0].held & ((frames & 1) == 0)) {
            scroll -= 1;
        }
        memset(backbuffer, 0, 512);
        for(int i = 0; i < 16; ++i) {
            int x = gamma_get(((scroll + i) & 15) * 2047 / 15);
            backbuffer[i] = x;
        }
#else
        for(int i = 0; i < 16; ++i) {
            int x = a[i] + b[i];
            if(x < 0) {
                x = b[i] = v();
            } else if(x > 2040) {
                x = 2040 + (b[i] = -v());
            }
            a[i] = (int16_t)x;
            backbuffer[i] = gamma_get((int16_t)x);
        }
#endif
        frames += 1;
    }
}