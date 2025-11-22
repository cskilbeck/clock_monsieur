#pragma once

#include <cstdint>
#include "esp_err.h"

enum button_id
{
    BUTTON_0,
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    NUM_BUTTONS
};

enum
{
    BUTTON_UP = BUTTON_0,
    BUTTON_SELECT = BUTTON_1,
    BUTTON_DOWN = BUTTON_2,
    BUTTON_RIGHT = BUTTON_3,
    BUTTON_LEFT = BUTTON_4,
};

struct button_t
{
    uint8_t gpio_num;    // which GPIO for the button
    uint8_t held;        // 0..1 - currently depressed (1) or not (0)
    uint8_t pressed;     // 0..255 - how many times it was pressed since you last asked
    uint8_t released;    // 0..255 - how many times it was released since you last asked
};

esp_err_t button_init();
void button_update();

extern button_t buttons[NUM_BUTTONS];

extern button_t &button_up;
extern button_t &button_down;
extern button_t &button_left;
extern button_t &button_right;
extern button_t &button_select;
