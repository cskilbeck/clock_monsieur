#include <freertos/FreeRTOS.h>

#include "util.h"

//////////////////////////////////////////////////////////////////////

void delay_ms(int ms)
{
    vTaskDelay(ms * (1000 / configTICK_RATE_HZ));
}

void delay_secs(int seconds)
{
    vTaskDelay(seconds * configTICK_RATE_HZ);
}
