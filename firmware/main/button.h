#pragma once

#include <cstdint>
#include "esp_err.h"

enum button_id
{
    BUTTON_0,
    BUTTON_1,
    BUTTON_2,
    NUM_BUTTONS
};

struct button_t
{
    uint8_t gpio_num;    // which GPIO for the button
    uint8_t held;        // 0..1 - currently depressed (1) or not (0)
    uint8_t pressed;     // 0..255 - how many times it was pressed since you last asked
    uint8_t released;    // 0..255 - how many times it was released since you last asked
};

esp_err_t button_init();

// get current state of a button (resets pressed/released values)
esp_err_t button_get(button_t result[NUM_BUTTONS]);