//////////////////////////////////////////////////////////////////////

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

//////////////////////////////////////////////////////////////////////

int gamma(uint32_t x)
{
    x = (x * x) >> 16;
    return (x * x) >> 16;
}

//////////////////////////////////////////////////////////////////////

uint16_t a[256];

extern "C" void app_main()
{
    display_init();

    int frames = 0;
    int x = 0;
    while(true) {
        display_waitvb();
        tlc5948_control.set_brightness(x, 0x0);
        x = (frames >> 9) & 15;
        tlc5948_control.set_brightness(x, 0xffff);
        display_set_grayscale();
        frames += 1;
    }
}