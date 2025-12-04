#include <freertos/FreeRTOS.h>
#include <sys/time.h>
#include <nvs_flash.h>

#include "main.h"
#include "util.h"
#include "display.h"
#include "graphics.h"
#include "wifi.h"
#include "lux.h"
#include "button.h"
#include "clock_time.h"
#include "settings.h"
#include "version.h"
#include "ota.h"
#include "state.h"
#include "timezone.h"
#include "console.h"

LOG_CONTEXT("main");

EventGroupHandle_t system_events;

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    LOG_INFO("----- CLOCK MONSIEUR, FIRMWARE VERSION: %s -----", VERSION_STR);

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);

    system_events = xEventGroupCreate();

    settings.load();

    lux_init();
    display_init();
    button_init();
    wifi_init();
    ota_init();
    console_init();
    clock_init();
    state_init();

    state_set(boot_state);

    while(true) {
        display_update();
        clock_update();
        button_update();
        lux_update();
        state_update();
        display->update_ambient();
    }
}
