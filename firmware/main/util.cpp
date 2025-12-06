#include <freertos/FreeRTOS.h>

#include "util.h"

//////////////////////////////////////////////////////////////////////

void delay_ms(int ms)
{
    int64_t ticks = (int64_t)ms * configTICK_RATE_HZ / 1000LLU;
    vTaskDelay((TickType_t)ticks);
}

void delay_secs(int seconds)
{
    vTaskDelay((TickType_t)seconds * configTICK_RATE_HZ);
}
