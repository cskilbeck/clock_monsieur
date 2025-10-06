//////////////////////////////////////////////////////////////////////

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "tlc_5948.h"
#include "i2c_task.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

//////////////////////////////////////////////////////////////////////

uint16_t a[256];

uint16_t gamma(uint16_t x)
{
    x = (x * x) >> 11;
    return (x * x) >> 11;
}

extern "C" void app_main()
{
    display_init();

    int frames = 0;
    while(true) {
        display_waitvb();
        uint8_t b = brightness;
        tlc5948_control.fcntrl.global_bc = b;
        for(int i = 0; i < 16; ++i) {
            tlc5948_control.set_dc(i, b);
        }
        int mul = 0;
        if((frames & 15) == 0) {
            mul = 120;
        }
        for(int i = 0; i < 16; ++i) {
            tlc5948_control.set_brightness(i, gamma((i + 1) * mul));
        }
        display_update();
        frames += 1;
    }
}