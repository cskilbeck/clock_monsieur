//////////////////////////////////////////////////////////////////////

#include <freertos/FreeRTOS.h>

#include "nvs_flash.h"

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
#include "console.h"

LOG_CONTEXT("main");

EventGroupHandle_t system_events;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"mem", "show stack and heap usage", "">
{
    void on_command(int argc, char **argv) override
    {
        char buffer[512];
        vTaskList(buffer);
        printf("%s\n", buffer);
        auto x = esp_get_minimum_free_heap_size();
        printf("HEAP: %lu\n", x);
    }
} cmd_mem;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"factory", "erase NVS partition", "">
{
    void on_command(int argc, char **argv) override
    {
        state_set(factory_reset_state);
    }
} cmd_factory;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"reset", "reset the ESP32", "">
{
    void on_command(int argc, char **argv) override
    {
        esp_restart();
    }
} cmd_reset;

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
    // state_set(ota_state);

    // got this far, probably fine...
    ota_mark_app_valid();

    while(true) {
        display_update();
        button_update();
        int ambient = lux_update();
        display->set_ambient(ambient);
        state_update();
        display->update_ambient();
    }
}
