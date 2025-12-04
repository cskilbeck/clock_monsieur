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
#if defined(DEBUG)
    int const slowness = 1;
#else
    int const slowness = 3;
#endif
    const font_t &font = big_caps_font;
    bool any_button_held = button_select.held || button_right.held || button_left.held || button_up.held || button_down.held;
    if(!any_button_held) {
        factory_reset = false;
    }
    int content_width = font.measure_string(boot_msg);
    int max_x = (content_width - screen_width) * slowness;
    int x = screen_width - 1 - frames / slowness;
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
    bool in_progress = (sys_events & SYS_EVENT_PROVISIONING_IN_PROGRESS) != 0;
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
    if(!wifi_up) {
        msg = "WiFi connecting...";
    }
    if(provisioning) {
        msg = "Use app: ESP BLE Provisioning";
    }
    char buffer[16];
    if(in_progress) {
        msg = "Enter WiFi details";
    } else if(connected) {
        snprintf(buffer, sizeof(buffer) - 1, "PIN:%s", provisioning_pop());
        msg = buffer;
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
// Show the message for 10 seconds
// If they hold UP for 3 or more seconds during that time, factory reset
// Else just go wifi_check_state (i.e. continue boot process)

void factory_reset_state_t::on_start()
{
    released_time = 0.0;
    held_time = 0.0;
}

void factory_reset_state_t::on_update()
{
    display->set_ambient(255);
    gfx.clear();
    if(button_up.held) {
        char buffer[2];
        float held = state_elapsed_seconds - held_time;
        if(held >= 3.0) {
            ESP_LOG_ERR(nvs_flash_erase());
            ESP_LOG_ERR(nvs_flash_init());
            esp_restart();
        }
        sprintf(buffer, "%d", (int)(4 - held));
        font_5x7_font.draw_string(gfx, buffer, 0, 0, 1);
    } else {
        held_time = state_elapsed_seconds;
        if(button_up.released) {
            released_time = state_elapsed_seconds;
        }
        char const *msg = "Hold UP to Factory Reset!";
        int width = font_5x7_font.measure_string(msg);
        int max_width = width + screen_width;
        double t = state_elapsed_seconds - released_time;
        int x = ((int)(t * screen_width)) % max_width;
        font_5x7_font.draw_string(gfx, msg, screen_width - x, 0, 1);
        if(t > 10) {
            state_set(wifi_check_state);
        }
    }
    gfx.display();
}

//////////////////////////////////////////////////////////////////////

void clock_state_t::on_update()
{
    float clock_color = 1.0f;
    if((xEventGroupGetBits(system_events) & SYS_EVENT_SNTP_SYNCHRONIZED) == 0) {
        clock_color = fabsf((frames & 127) / 127.0f - 0.5f) + 0.5f;
    }

    float constexpr microseconds = 1000000.0f;
    float constexpr one_second = 1.0f;
    float second_snap{ 0.0f };
    suseconds_t usecs = local_wall_time.tv_usec;
    float fade = 0.0f;
    switch(settings.clock_fade_mode) {
    case clock_fade_mode_t::off:
        break;
    case clock_fade_mode_t::high:
        second_snap = one_second / microseconds;
        fade = min(usecs * second_snap, 1.0f);
        break;
    case clock_fade_mode_t::medium:
        second_snap = (one_second / 0.5f) / microseconds;
        usecs = max(0l, usecs - 500000);
        fade = min(usecs * second_snap, 1.0f);
        break;
    case clock_fade_mode_t::low:
        second_snap = (one_second / 0.25f) / microseconds;
        usecs = max(0l, usecs - 750000);
        fade = min(usecs * second_snap, 1.0f);
        break;
    }

    gfx.draw_clock(local_wall_time.tv_sec, clock_color);
    if(settings.clock_fade_mode == clock_fade_mode_t::off) {
        gfx.display();
    } else {
        gfx2.draw_clock(local_wall_time.tv_sec + 1, clock_color);
        gfx.fade_to(gfx2, fade);
    }

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
