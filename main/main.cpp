//////////////////////////////////////////////////////////////////////

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_task.h"
#include "high_side.h"
#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    tlc5948_init();
    i2c_task_start();

    uint32_t x = 0;

    while(true) {
        uint32_t g = (x * x) >> 16;
        for(int i=0; i<16; ++i) {
            tlc5948_control.brightness[i] = g;
        }
        tlc5948_set_grayscale();
        esp_rom_delay_us(5000);
        x = (x + 256) % 65536;
    }
}