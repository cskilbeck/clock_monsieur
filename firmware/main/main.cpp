#include <freertos/FreeRTOS.h>

#include "main.h"
#include "util.h"
#include "display.h"
#include "wifi.h"
#include "lux.h"
#include "button.h"
#include "clock_time.h"
#include "settings.h"
#include "version.h"
#include "ota.h"
#include "state.h"
#include "console.h"
#include "buzzer.h"

LOG_CONTEXT("main");

EventGroupHandle_t system_events;

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    LOG_INFO("----- CLOCK MONSIEUR, FIRMWARE VERSION: %s -----", VERSION_STR);

    system_events = xEventGroupCreate();

    settings_init();
    display_init();
    lux_init();
    button_init();
    buzzer_init();
    wifi_init();
    ota_init();
    clock_init();
    state_init();
    console_init();

    while(true) {
        display_update();
        clock_update();
        button_update();
        lux_update();
        state_update();
        display->update_ambient();
    }
}
