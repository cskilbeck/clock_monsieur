//////////////////////////////////////////////////////////////////////

#include <memory>
#include <cmath>
#include "esp_rtc_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "main.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "graphics.h"
#include "state.h"
#include "menu.h"
#include "version.h"
#include "settings.h"
#include "button.h"
#include "util.h"
#include "clock_time.h"
#include "timezone.h"

LOG_CONTEXT("state");

//////////////////////////////////////////////////////////////////////

state_handler_t null_state;
boot_state_t boot_state;
wifi_check_state_t wifi_check_state;
factory_reset_state_t factory_reset_state;
clock_state_t clock_state;
menu_state_t menu_state;
timezone_select_state_t timezone_select_state;
ota_state_t ota_state;

//////////////////////////////////////////////////////////////////////

namespace
{
    state_handler_t *current_state = &null_state;
    QueueHandle_t state_queue;
    int frames;

    uint64_t state_start_timestamp;
    double state_elapsed_seconds;

};    // namespace

//////////////////////////////////////////////////////////////////////

void state_handler_t::init()
{
}
void state_handler_t::on_start()
{
}
void state_handler_t::on_update()
{
}
void state_handler_t::on_stop()
{
}

//////////////////////////////////////////////////////////////////////

void state_init()
{
    state_queue = xQueueCreate(2, sizeof(void *));
}

//////////////////////////////////////////////////////////////////////

void state_update()
{
    state_handler_t *new_state;
    BaseType_t got = xQueueReceive(state_queue, (void *)&new_state, (TickType_t)0);
    if(got == pdPASS) {
        current_state->on_stop();
        current_state = new_state;
        frames = 0;
        state_start_timestamp = esp_rtc_get_time_us();
        current_state->on_start();
    }
    frames += 1;
    state_elapsed_seconds = (esp_rtc_get_time_us() - state_start_timestamp) / 1000000.0;
    current_state->on_update();
}

//////////////////////////////////////////////////////////////////////

void state_set(state_handler_t &new_state)
{
    void *ptr_to_new_state = (void *)&new_state;
    xQueueSendToBack(state_queue, &ptr_to_new_state, (TickType_t)0);
}

//////////////////////////////////////////////////////////////////////

void boot_state_t::on_start()
{
    boot_msg = (char *)malloc(128);
    sprintf(boot_msg, "CLOCK MONSIEUR! V%s   ", VERSION_STR);
    factory_reset = true;
}

//////////////////////////////////////////////////////////////////////

void boot_state_t::on_update()
{
    const font_t &font = big_caps_font;
    factory_reset &= button_left.held && button_right.held;
    int content_width = font.measure_string(boot_msg);
    int max_x = content_width - screen_width;
    int x = screen_width - 1 - (frames >> 1);
    display->set_ambient(160);
    gfx.clear();
    font.draw_string(gfx, boot_msg, x, 0, 0.6);
    int total = content_width + screen_width;
    int relative = -(x - screen_width) * 60 / total;
    for(int i = 0; i < relative; ++i) {
        gfx.set_second_only(1.0f, i);
    }
    gfx.display();
    if(x < -content_width) {
        if(factory_reset) {
            state_set(factory_reset_state);
        } else {
            state_set(wifi_check_state);
        }
    }
}

//////////////////////////////////////////////////////////////////////

void boot_state_t::on_stop()
{
    free(boot_msg);
    xEventGroupSetBits(system_events, SYS_EVENT_BOOT_MSG_COMPLETE);
}

//////////////////////////////////////////////////////////////////////

void wifi_check_state_t::on_update()
{
    EventBits_t sys_events = xEventGroupGetBits(system_events);
    bool network_up = (sys_events & SYS_EVENT_NETWORK_CONNECTED) == SYS_EVENT_NETWORK_CONNECTED;
    if(network_up) {
        state_set(clock_state);
        return;
    }
    bool wifi_up = (sys_events & SYS_EVENT_WIFI_CONNECTED) == SYS_EVENT_WIFI_CONNECTED;
    bool provisioning = (sys_events & SYS_EVENT_PROVISIONING) != 0;
    bool connected = (sys_events & SYS_EVENT_BLE_CONNECTED) != 0;
    bool error = (sys_events & SYS_EVENT_PROVISIONING_ERROR) != 0;
    bool done = (sys_events & SYS_EVENT_PROVISIONING_DONE) != 0;
    gfx.clear();
    int sec = ((frames >> 2) % 12);
    for(int s = 0; s < 60; ++s) {
        float color = ((s + sec) % 12) < 6 ? 0.0f : 0.95f;
        gfx.set_second_only(color, 59 - s);
    }
    char const *msg = "Network?";
    if(wifi_up) {
        char const *msg = "WiFi connecting...";
    }
    if(provisioning) {
        msg = "Searching...";
    }
    if(connected) {
        msg = provisioning_pop();
    }
    if(error) {
        msg = "Error...";
    }
    if(done) {
        msg = "Success!";
    }
    font_t const &font = font_5x7_font;
    int w = font.measure_string(msg);
    int x = (frames >> 2) % (screen_width + w);
    font.draw_string(gfx, msg, screen_width - x, 0, 0.9f);
    gfx.display();
}

//////////////////////////////////////////////////////////////////////

void factory_reset_state_t::on_update()
{
    char const *msg = "Factory Reset!";
    int x = 30 - frames / 2;
    display->set_ambient(255);
    gfx.clear();
    int width = font_5x7_font.draw_string(gfx, msg, x, 0, 1);
    gfx.display();
    if(x < -width) {
        ESP_LOG_ERR(nvs_flash_erase());
        ESP_LOG_ERR(nvs_flash_init());
        esp_restart();
    }
}

//////////////////////////////////////////////////////////////////////

void clock_state_t::on_update()
{
    struct timeval tv_now;
    get_time(&tv_now);

    float clock_color = 1.0f;
    if((xEventGroupGetBits(system_events) & SYS_EVENT_SNTP_SYNCHRONIZED) == 0) {
        clock_color = fabsf((frames & 127) / 127.0f - 0.5f) + 0.5f;
    }

    gfx.draw_clock(tv_now.tv_sec, clock_color);

    float constexpr microseconds = 1000000.0f;
    float constexpr one_second = 1.0f;
    float second_snap{};
    switch(settings.clock_fade_mode) {
    case clock_fade_mode_t::off:
        gfx.display();
        return;    // <--- !
    case clock_fade_mode_t::high:
        second_snap = one_second / microseconds;
        break;
    case clock_fade_mode_t::medium:
        second_snap = (one_second / 0.5f) / microseconds;
        break;
    case clock_fade_mode_t::low:
        second_snap = (one_second / 0.2f) / microseconds;
        break;
    }
    gfx2.draw_clock(tv_now.tv_sec - 1, clock_color);
    gfx2.fade_to(gfx, min(1.0f, tv_now.tv_usec * second_snap));

    if(button_select.pressed) {
        state_set(menu_state);
    }
}

//////////////////////////////////////////////////////////////////////

void menu_state_t::on_start()
{
    menu_init();
}

//////////////////////////////////////////////////////////////////////

void menu_state_t::on_update()
{
    menu_update();
}

//////////////////////////////////////////////////////////////////////

void timezone_select_state_t::on_start()
{
    timezone_select_init();
}

//////////////////////////////////////////////////////////////////////

void timezone_select_state_t::on_update()
{
    timezone_select_update();
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void ota_state_t::on_update()
{
    gfx.clear();
    display->set_ambient(200);
    char const *msg = "Firmware Updating...";
    int width = font_5x7_font.measure_string(msg);
    int x = screen_width - ((frames >> 1) % (width + screen_width));
    font_5x7_font.draw_string(gfx, msg, x, 0, 1);
    gfx.display();
}
