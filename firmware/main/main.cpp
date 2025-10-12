//////////////////////////////////////////////////////////////////////

#include "esp_log.h"

#include "display.h"
#include "i2c_task.h"

static char const *TAG = "main";

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
    display_init();

    for(int i = 0; i < 256; ++i) {
        a[i] = 0;
        b[i] = v();
    }

    int frames = 0;
    while(true) {
        uint16_t *backbuffer = display_update();
        for(int i = 0; i < 256; ++i) {
            int x = a[i] + b[i];
            if(x < 0) {
                x = 0;
                b[i] = v();
            } else if(x > 2040) {
                x = 2040;
                b[i] = -v();
            }
            a[i] = (int16_t)x;
            backbuffer[i] = gamma(a[i]);
        }
        frames += 1;
    }
}