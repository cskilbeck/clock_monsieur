//////////////////////////////////////////////////////////////////////

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
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

    int x = 0;
    while(true) {
        display_waitvb();
        for(int i = 0; i < 256; ++i) {
            int level = i * 256;
            a[i] = (level - x) & 0xffff;
        }
        int b = 0;
        for(int i = 0; i < 256; i += 16) {
            int e = i + 16;
            uint32_t t = 0;
            for(int j = i; j < e; ++j) {
                t += a[j];
            }
            tlc5948_control.brightness[b++] = gamma(t >> 4);
        }
        display_set_grayscale();
        x = (x + 27) % 65536;
        if(x < 28) {
            ESP_LOGI(TAG, "!!");
        }
    }
}