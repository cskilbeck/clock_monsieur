#include <freertos/FreeRTOS.h>

#include "util.h"

//////////////////////////////////////////////////////////////////////

void delay_ms(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delay_secs(int seconds)
{
    delay_ms(seconds * 1000);
}
