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
#include "buzzer.h"
#include "melodies.h"

LOG_CONTEXT("state");

//////////////////////////////////////////////////////////////////////

namespace
{
    alignas(std::max_align_t) static uint8_t state_buf[max_sizeof_v<state_handler_t,            //
                                                                    boot_state_t,               //
                                                                    wifi_check_state_t,         //
                                                                    factory_reset_state_t,      //
                                                                    clock_state_t,              //
                                                                    ota_state_t,                //
                                                                    menu_state_t,               //
                                                                    timezone_select_state_t,    //
                                                                    lux_state_t,                //
                                                                    led_edit_state_t,           //
                                                                    timer_state_t,              //
                                                                    alarm_state_t>              //
    ];
    state_handler_t *current_state = nullptr;
    QueueHandle_t state_queue;
    int frames;

    uint64_t state_start_timestamp;
    double state_elapsed_seconds;

    double snooze_end_time = 0;

}    // namespace

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
    current_state = new(state_buf) state_handler_t{};
    state_queue = xQueueCreate(2, sizeof(void (*)(void *)));
    state_set<boot_state_t>();
    // state_set<led_edit_state_t>();
}

//////////////////////////////////////////////////////////////////////

void state_update()
{
    void (*ctor)(void *);
    if(xQueueReceive(state_queue, &ctor, 0) == pdPASS) {
        current_state->on_stop();
        current_state->~state_handler_t();
        ctor(state_buf);
        current_state = reinterpret_cast<state_handler_t *>(state_buf);
        frames = 0;
        state_start_timestamp = esp_rtc_get_time_us();
        current_state->on_start();
    }
    frames += 1;
    state_elapsed_seconds = (esp_rtc_get_time_us() - state_start_timestamp) / 1000000.0;
    current_state->on_update();
}

//////////////////////////////////////////////////////////////////////

void state_enqueue(void (*ctor)(void *))
{
    xQueueSendToBack(state_queue, &ctor, 0);
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
    int const slowness = 2;
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
    display->set_ambient(255);
    gfx.clear();
    if(state_elapsed_seconds < 0.5f) {
        frames = 0;
    } else {
        font.draw_string(gfx, boot_msg, x, 0, 1.0f);
        int total = content_width + screen_width;
        int relative = -(x - screen_width) * 60 / total;
        for(int i = 0; i < 60; ++i) {
            gfx.set_second_only(relative / 60.0f, i);
        }
    }
    gfx.display();
    if(x < -content_width) {
        if(factory_reset) {
            state_set<factory_reset_state_t>();
        } else {
            state_set<wifi_check_state_t>();
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
        state_set<clock_state_t>();
        return;
    }
    bool wifi_up = (sys_events & SYS_EVENT_WIFI_CONNECTED) == SYS_EVENT_WIFI_CONNECTED;
    bool provisioning = (sys_events & SYS_EVENT_PROVISIONING) != 0;
    bool got_ssid = (sys_events & SYS_EVENT_PROVISIONING_GOT_SSID) != 0;
    bool in_progress = (sys_events & SYS_EVENT_PROVISIONING_IN_PROGRESS) != 0;
    bool connected = (sys_events & SYS_EVENT_BLE_CONNECTED) != 0;
    bool error = (sys_events & SYS_EVENT_PROVISIONING_ERROR) != 0;
    bool done = (sys_events & SYS_EVENT_PROVISIONING_DONE) != 0;
    gfx.clear();
    int sec = (int)(state_elapsed_seconds * 30.0);
    for(int s = 0; s < 60; ++s) {
        float color = s / 60.0f;
        gfx.set_second_only(color, sec % 60);
        sec += 1;
    }
    char const *msg = "Network?";
    if(!wifi_up) {
        msg = "WiFi connecting...";
    }
    if(provisioning) {
        msg = "Use app: ESP BLE Provisioning";
    }
    char buffer[16];
    if(got_ssid) {
        msg = "Connecting...";
    } else if(in_progress) {
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
// If they hold UP for 5 or more seconds during that time, factory reset
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
        int constexpr RESET_HOLD_TIME = 5;
        char buffer[2];
        float held = state_elapsed_seconds - held_time;
        if(held >= RESET_HOLD_TIME) {
            ESP_LOG_ERR(nvs_flash_erase());
            ESP_LOG_ERR(nvs_flash_init());
            esp_restart();
        }
        sprintf(buffer, "%d", (int)((RESET_HOLD_TIME + 1) - held));
        font_5x7_font.draw_string(gfx, buffer, 11, 0, 1);
        int seconds = (int)(held * 60 / RESET_HOLD_TIME);
        float scale = 1.0f / seconds;
        gfx.set_second_only(1.0f, 0);
        for(int i = 1; i < seconds; ++i) {
            gfx.set_second_only(i * scale, i);
        }
    } else {
        held_time = state_elapsed_seconds;
        if(button_up.released) {
            released_time = state_elapsed_seconds;
        }
        char const *msg = "Press UP to Factory Reset!";
        int width = font_5x7_font.measure_string(msg);
        int max_width = width + screen_width;
        double t = state_elapsed_seconds - released_time;
        int x = ((int)(t * 2 * screen_width)) % max_width;
        font_5x7_font.draw_string(gfx, msg, screen_width - x, 0, 1);
        if(t > 10) {
            state_set<wifi_check_state_t>();
        }
    }
    gfx.display();
}

//////////////////////////////////////////////////////////////////////

void clock_state_t::on_update()
{
    clock_draw();

    // snooze check
    if(snooze_end_time > 0) {
        double now = utc_wall_time.tv_sec + utc_wall_time.tv_usec / 1000000.0;
        if(now >= snooze_end_time) {
            snooze_end_time = 0;
            state_set<alarm_state_t>();
            return;
        }
    }

    // alarm check
    static time_t last_alarm_minute = -1;

    if(settings.alarm_enabled == alarm_enabled_t::On) {
        long local_seconds = utc_wall_time.tv_sec + timezone_offset_seconds;
        int local_hour = (int)((local_seconds / 3600) % 24);
        int local_minute = (int)((local_seconds % 3600) / 60);
        time_t current_epoch_minute = utc_wall_time.tv_sec / 60;

        if(local_hour == settings.alarm_hour && local_minute == settings.alarm_minute && current_epoch_minute != last_alarm_minute) {
            int local_dow = (int)(((local_seconds / 86400) + 4) % 7);    // 0=Sun .. 6=Sat
            bool should_fire = true;
            if(settings.alarm_mode == alarm_mode_t::Weekdays) {
                should_fire = (local_dow >= 1 && local_dow <= 5);
            }
            if(should_fire) {
                last_alarm_minute = current_epoch_minute;
                state_set<alarm_state_t>();
                return;
            }
        }
    }

    if(button_select.pressed) {
        state_set<menu_state_t>();
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

//////////////////////////////////////////////////////////////////////

void led_edit_state_t::on_start()
{
    led_number = 0;
}

//////////////////////////////////////////////////////////////////////

void led_edit_state_t::on_update()
{
    display->set_ambient(255);
    gfx.clear();
    if(button_left.pressed) {
        led_number += 1;
        LOG_INFO("%d", led_number);
    }
    if(button_right.pressed) {
        led_number -= 1;
        LOG_INFO("%d", led_number);
    }
    if(button_up.held) {
        for(int i = 0; i < 256; ++i) {
            gfx.set_led(1.0f, i);
        }
    } else {
        gfx.set_led(1.0f, led_number);
    }
    gfx.display();
}

//////////////////////////////////////////////////////////////////////

#if defined(DEBUG)

void led_test_state_t::on_start()
{
    brightness = 0.0f;
}

//////////////////////////////////////////////////////////////////////

void led_test_state_t::on_update()
{
    if(button_select.pressed) {
        state_set<menu_state_t>();
    }
    display->set_ambient(255);
    gfx.clear();
    if(button_up.held) {
        brightness += 0.01f;
    }
    if(button_down.held) {
        brightness -= 0.01f;
    }
    if(button_left.pressed) {
        brightness -= 0.01f;
    }
    if(button_right.pressed) {
        brightness += 0.01f;
    }
    brightness = std::clamp(brightness, 0.0f, 1.0f);
    for(int i = 0; i < 256; ++i) {
        gfx.set_led(brightness, i);
    }
    gfx.display();
}

#endif

//////////////////////////////////////////////////////////////////////

void timer_state_t::on_start()
{
    end_time = utc_wall_time.tv_sec + utc_wall_time.tv_usec / 1000000.0 + settings.timer_seconds;
    done = false;
}

//////////////////////////////////////////////////////////////////////

void timer_state_t::on_update()
{
    double now = utc_wall_time.tv_sec + utc_wall_time.tv_usec / 1000000.0;
    double remaining_f = end_time - now;
    int remaining = (remaining_f <= 0.0) ? 0 : (int)ceil(remaining_f);

    if(remaining <= 0) {
        if(!done) {
            buzzer_play_melody(melodies::CHARGE_X3, melodies::CHARGE_X3_COUNT, false);
            done = true;
        }

        // any button dismisses, or auto-return when melody finishes
        if(button_select.pressed || button_left.pressed || button_right.pressed || button_up.pressed || button_down.pressed) {
            buzzer_stop();
            state_set<clock_state_t>();
            return;
        }
        if(!buzzer_is_playing()) {
            state_set<clock_state_t>();
            return;
        }

        remaining = 0;
    } else {
        // cancel on select or left during countdown
        if(button_select.pressed || button_left.pressed) {
            state_set<clock_state_t>();
            return;
        }
    }

    int mins = remaining / 60;
    int secs = remaining % 60;

    gfx.clear();

    // seconds ring: bright for 0..secs-1, dim for secs..59
    for(int i = 0; i < 60; ++i) {
        gfx.set_second(i < secs ? 1.0f : 0.15f, i);
    }

    // M:SS display using clock font at same digit positions as draw_time
    char buf[16];
    sprintf(buf, "%2d%02d", mins, secs);
    font_t const &font = settings.clock_font == clock_font_t::Square ? square_font_font : font_5x7_font;
    font.draw_char_centered(gfx, buf[0], 2, 0, 1.0f);
    font.draw_char_centered(gfx, buf[1], 8, 0, 1.0f);
    font.draw_char_centered(gfx, buf[2], 16, 0, 1.0f);
    font.draw_char_centered(gfx, buf[3], 22, 0, 1.0f);

    // steady colon
    gfx.buffer[graphics_t::matrix_lookup[2][12]] = 1.0f;
    gfx.buffer[graphics_t::matrix_lookup[4][12]] = 1.0f;

    gfx.display();
}

//////////////////////////////////////////////////////////////////////

void alarm_state_t::on_start()
{
    int idx = (int)settings.alarm_melody;
    if(idx < 0 || idx >= melodies::count)
        idx = 0;
    auto const &m = melodies::table[idx];
    buzzer_play_melody(m.notes, m.count, true);
    end_time = utc_wall_time.tv_sec + utc_wall_time.tv_usec / 1000000.0 + 60.0;
    snooze_msg = false;
    snooze_msg_time = 0;

    // Once mode: disable on first fire (snooze re-fires still work this session)
    if(snooze_end_time == 0 && settings.alarm_mode == alarm_mode_t::Once) {
        settings.alarm_enabled = alarm_enabled_t::Off;
        settings.save();
    }
}

//////////////////////////////////////////////////////////////////////

void alarm_state_t::on_update()
{
    double now = utc_wall_time.tv_sec + utc_wall_time.tv_usec / 1000000.0;
    bool any_button = button_select.pressed || button_left.pressed || button_right.pressed || button_up.pressed || button_down.pressed;

    if(snooze_msg) {

        // snooze message phase: show "SNOOZE!" for 1 second
        if(any_button) {
            // second press during message: cancel this occurrence only
            snooze_end_time = 0;
            state_set<clock_state_t>();
            return;
        }

        if(now - snooze_msg_time >= 1.0) {
            // message done: set snooze timer and return to clock
            snooze_end_time = now + settings.alarm_snooze * 60.0;
            state_set<clock_state_t>();
            return;
        }

        gfx.clear();
        for(int i = 0; i < 60; ++i) {
            gfx.set_second(0.15f, i);
        }
        font_5x7_narrow_modern_font.draw_string(gfx, "SNOOZE!", 0, 0, 1.0f);
        gfx.display();

    } else {

        // firing phase: melody playing, show alarm time
        bool timed_out = now >= end_time;

        if(any_button) {
            // first press: enter snooze message
            buzzer_stop();
            snooze_msg = true;
            snooze_msg_time = now;
            return;
        }

        if(timed_out) {
            // 1 minute with no response: auto-snooze
            buzzer_stop();
            snooze_end_time = now + settings.alarm_snooze * 60.0;
            state_set<clock_state_t>();
            return;
        }

        gfx.clear();

        // dim ring
        for(int i = 0; i < 60; ++i) {
            gfx.set_second(0.15f, i);
        }

        // display alarm time HH:MM
        int hours = settings.alarm_hour;
        int minutes = settings.alarm_minute;
        char const *fmt;
        if(settings.clock_mode == clock_mode_t::clock_24_hour) {
            hours %= 24;
            fmt = "%02d%02d";
        } else {
            hours %= 12;
            if(hours == 0) {
                hours = 12;
            }
            fmt = "%2d%02d";
        }
        char buf[16];
        sprintf(buf, fmt, hours, minutes);
        font_t const &font = settings.clock_font == clock_font_t::Square ? square_font_font : font_5x7_font;
        font.draw_char_centered(gfx, buf[0], 2, 0, 1.0f);
        font.draw_char_centered(gfx, buf[1], 8, 0, 1.0f);
        font.draw_char_centered(gfx, buf[2], 16, 0, 1.0f);
        font.draw_char_centered(gfx, buf[3], 22, 0, 1.0f);

        // steady colon
        gfx.buffer[graphics_t::matrix_lookup[2][12]] = 1.0f;
        gfx.buffer[graphics_t::matrix_lookup[4][12]] = 1.0f;

        gfx.display();
    }
}
