#include <freertos/FreeRTOS.h>

#include "util.h"

bool gDumpStacks{ false };

namespace util
{
    void stack_stat()
    {
        if(gDumpStacks) {
            printf("%s: %u\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
        }
    }
}    // namespace util
