//////////////////////////////////////////////////////////////////////

#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"

#include "i2c_task.h"
#include "high_side.h"
#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

EventGroupHandle_t main_event_group;

#define ON_FRAME_BIT (1 << 0)

//////////////////////////////////////////////////////////////////////

int gamma(int x)
{
    int sq = (x * x) >> 16;
    int g = (sq * sq) >> 16;
    return g;
}

//////////////////////////////////////////////////////////////////////

void on_frame(void *)
{
    tlc5948_set_grayscale();
    xEventGroupSetBits(main_event_group, ON_FRAME_BIT);
}

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    tlc5948_init();
    i2c_task_start();

    main_event_group = xEventGroupCreate();

    uint32_t x = 0;

    const esp_timer_create_args_t periodic_timer_args = { .callback = &on_frame, .name = "on_frame" };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2000));

    int frame = 0;

    while(true) {
        EventBits_t uxBits = xEventGroupWaitBits(main_event_group, ON_FRAME_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if((uxBits & ON_FRAME_BIT) != 0) {
            for(int i = 0; i < 16; ++i) {
                int level = i * 4096;
                int bright = (level - x) & 0xffff;
                if(bright > 65535) {
                    bright = 65535;
                }
                tlc5948_control.brightness[i] = gamma(bright);
            }
            x = (x + 128) % 65536;
        }
    }
}