//////////////////////////////////////////////////////////////////////

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "main.h"
#include "nvs_flash.h"
#include "graphics.h"
#include "state.h"
#include "version.h"
#include "settings.h"
#include "button.h"
#include "util.h"
#include "clock_time.h"

LOG_CONTEXT("state");

//////////////////////////////////////////////////////////////////////

state_handler_t null_state;
boot_state_t boot_state;
factory_reset_state_t factory_reset_state;
clock_state_t clock_state;
ota_state_t ota_state;

//////////////////////////////////////////////////////////////////////

namespace
{
    state_handler_t *current_state = &null_state;
    QueueHandle_t state_queue;
    int frames;

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
        current_state->on_start();
    }
    frames += 1;
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
    sprintf(boot_msg, "CLOCK MONSIEUR   WELCOME! FIRMWARE VERSION %s   ", VERSION_STR);
    // sprintf(boot_msg, "!!!European!!!");
    // sprintf(boot_msg, "the quick brown fox jumps over the lazy dog");
    // sprintf(boot_msg, "tttttttt");
    factory_reset = true;
    x = screen_width;
}

//////////////////////////////////////////////////////////////////////
// content_width is width of the content to show
// screen_width is width of the display screen
// content_pos is the origin X pos where the content was drawn (always either negative or 0)

void calculate_scrollbar(int content_width, int content_pos, int *scrollbar_width_out, int *scrollbar_pos_out)
{
    if(content_width <= screen_width) {
        *scrollbar_width_out = 0;
        *scrollbar_pos_out = 0;
        return;
    }
    float ratio = (float)screen_width / (float)content_width;
    float scrollbar_width = (float)(screen_width * ratio + 0.5f);
    float max_scroll = (float)content_width - (float)screen_width;
    float current_scroll = (float)-content_pos;
    float normalized_pos = current_scroll / max_scroll;
    float scrollbar_track_length = (float)screen_width - scrollbar_width;
    float scrollbar_pos_float = normalized_pos * scrollbar_track_length;
    *scrollbar_width_out = (int)scrollbar_width;
    *scrollbar_pos_out = (int)(scrollbar_pos_float + 0.5f);
}

void boot_state_t::on_update()
{
    const font_t &font = big_caps_font;
    factory_reset &= button_left.held && button_right.held;
    int content_width = font.measure_string(boot_msg);
    int max_x = content_width - screen_width;
    // if(button_left.held && ((frames & 1) == 0)) {
    //     x = min(0, x + 1);
    // }
    // if(button_right.held && ((frames & 1) == 0)) {
    //     x = max(-max_x, x - 1);
    // }
    if((frames & 1) == 0) {
        x -= 1;
    }
    int dx = x;
    display->set_ambient(160);
    gfx.clear();
    font.draw_string(gfx, boot_msg, dx, 0, 0.6);

    // int scrollbar_width;
    // int scrollbar_pos;
    // calculate_scrollbar(content_width, dx, &scrollbar_width, &scrollbar_pos);

    // if(scrollbar_width < 26) {
    //     int y = 6;
    //     for(int i = 0; i < scrollbar_width; ++i) {
    //         int sx = (int)(i + scrollbar_pos);
    //         if(gfx.get_pixel(sx, y) == 0) {
    //             gfx.set_pixel(0.4f, sx, y);
    //         }
    //     }
    // }

    gfx.display();
    if(x < -content_width) {
        state_set(clock_state);
    }
}

//////////////////////////////////////////////////////////////////////

void boot_state_t::on_stop()
{
    free(boot_msg);
    if(factory_reset) {
        state_set(factory_reset_state);
    }
    xEventGroupSetBits(system_events, BOOT_MSG_COMPLETE_BIT);
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

    gfx.draw_clock(tv_now.tv_sec);

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
    gfx2.draw_clock(tv_now.tv_sec - 1);
    gfx2.fade_to(gfx, min(1.0f, tv_now.tv_usec * second_snap));
}

//////////////////////////////////////////////////////////////////////

void ota_state_t::on_update()
{
    gfx.clear();
    font_5x7_font.draw_string(gfx, "FW!!", 0, 0, 1);
    gfx.display();
}
