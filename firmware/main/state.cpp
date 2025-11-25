//////////////////////////////////////////////////////////////////////

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "state.h"
#include "version.h"
#include "graphics.h"
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
    sprintf(boot_msg, "Clock Monsieur v %s", VERSION_STR);
    factory_reset = true;
}

//////////////////////////////////////////////////////////////////////

void boot_state_t::on_update()
{
    factory_reset &= button_left.held && button_right.held;
    int x = 30 - frames / 2;
    display->set_ambient(255);
    gfx.clear();
    int width = gfx.draw_string(boot_msg, x, 0, 1);
    gfx.display();
    if(x < -width) {
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
}

//////////////////////////////////////////////////////////////////////

void factory_reset_state_t::on_update()
{
    char const *msg = "Factory Reset!";
    int x = 30 - frames / 2;
    display->set_ambient(255);
    gfx.clear();
    int width = gfx.draw_string(msg, x, 0, 1);
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
    gfx.draw_string("FW!!", 0, 0, 1);
    gfx.display();
}
