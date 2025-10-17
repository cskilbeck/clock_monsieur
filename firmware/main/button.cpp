//////////////////////////////////////////////////////////////////////

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "button.h"
#include "gpio_defs.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////

LOG_CONTEXT("button");

//////////////////////////////////////////////////////////////////////

namespace
{
    button_t buttons[NUM_BUTTONS] = { { .gpio_num = BUTTON_0_GPIO } };

    TaskHandle_t button_task_handle;

    //////////////////////////////////////////////////////////////////////
    // No debouncing! We're running at 5mS so it'll be fine, whatever

    void periodic_timer_callback(void *arg)
    {
        for(int i = 0; i < NUM_BUTTONS; ++i) {
            button_t &button = buttons[i];
            int down = gpio_get_level((gpio_num_t)button.gpio_num) == 0;
            if(down != button.held) {
                if(down) {
                    button.pressed += 1;
                } else {
                    button.released += 1;
                }
            }
            button.held = down;
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////
// start button reader - runs on whatever task this is

esp_err_t button_init()
{
    // set the GPIOs to input with pullup
    gpio_config_t gpio_cfg{};
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    for(int i = 0; i < NUM_BUTTONS; ++i) {
        gpio_cfg.pin_bit_mask |= 1LLU << buttons[i].gpio_num;
    }
    ESP_CHECK(gpio_config(&gpio_cfg));

    // start 5mS timer
    esp_timer_handle_t periodic_timer;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "button_timer",
    };
    ESP_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_CHECK(esp_timer_start_periodic(periodic_timer, 5000));    // 5 mS
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// Get state of all buttons and reset pressed/released counters
// Call this from the same task that you called button_init from

esp_err_t button_get(button_t result[NUM_BUTTONS])
{
    for(int i = 0; i < NUM_BUTTONS; ++i) {
        result[i] = buttons[i];
    }
    for(int i = 0; i < NUM_BUTTONS; ++i) {
        buttons[i].pressed = 0;
        buttons[i].released = 0;
    }
    return ESP_OK;
}
