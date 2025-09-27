//////////////////////////////////////////////////////////////////////

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "tlc_5948.h"
#include "display.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

//////////////////////////////////////////////////////////////////////

int gamma(int x)
{
    x = (x * x) >> 16;
    return (x * x) >> 16;
}

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    init_display();

    int frame = 0;
    uint32_t x = 0;

    while(true) {
        waitvb();
        for(int i = 0; i < 16; ++i) {
            int level = i * 4096;
            int bright = (level - x) & 0xffff;
            if(bright > 65535) {
                bright = 65535;
            }
            tlc5948_control.brightness[i] = gamma(bright);
        }
        x = (x + 173) % 65536;
    }
}