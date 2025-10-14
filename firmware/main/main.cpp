//////////////////////////////////////////////////////////////////////

#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"

#include "util.h"
#include "display.h"
#include "i2c_task.h"
#include "wifi.h"

static char const *TAG = "main";

LOG_CONTEXT("main");

int16_t a[256];
int16_t b[256];

uint16_t gamma(uint16_t x)
{
    x = (x * x) >> 11;
    return (x * x) >> 11;
}

int v()
{
    return ((rand() & 15) + 3) << 1;
}

extern "C" void app_main()
{
    // disable brownout detector
    // REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);
    // REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);

    for(int i = 0; i < 256; ++i) {
        a[i] = 0;
        b[i] = v();
    }

    // start the wifi connecting

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);

    wifi_init();

    display_init();

    int frames = 0;
    while(true) {
        uint16_t *backbuffer = display_update();
        for(int i = 0; i < 256; ++i) {
            backbuffer[i] = 6000;
            // int x = a[i] + b[i];
            // if(x < 0) {
            //     x = 0;
            //     b[i] = v();
            // } else if(x > 2040) {
            //     x = 2040;
            //     b[i] = -v();
            // }
            // a[i] = (int16_t)x;
            // backbuffer[i] = gamma(a[i]);
        }
        frames += 1;
    }
}