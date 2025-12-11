//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#include <cstdint>
#include "display.h"
#include "font_5x7.h"
#include "font_5x7_narrow_modern.h"
#include "square_font.h"
#include "big_caps.h"

//////////////////////////////////////////////////////////////////////

static int constexpr screen_width = 26;
static int constexpr screen_height = 7;

uint16_t gamma_get(uint16_t x);
uint16_t gamma_get(float x);

//////////////////////////////////////////////////////////////////////

struct graphics_t
{
    float buffer[256];

    static uint8_t const matrix_lookup[screen_height][screen_width];

    void clear();
    void clear_matrix();
    void set_led(float color, uint8_t n);
    float get_pixel(int x, int y);
    void set_second(float color, uint8_t second);
    void set_tick(float color, uint8_t tick);
    void set_second_only(float color, uint8_t second);
    void draw_time(int hours, int minutes, float color);
    void draw_colon(int seconds, float color);
    void draw_seconds(int current_second);
    void draw_clock(long seconds, float clock_color);
    void display();
    void fade_to(graphics_t &other, float scale);

    void set_debug(int led, int value);

    //////////////////////////////////////////////////////////////////////

    __attribute__((always_inline)) inline void set_pixel_unclipped(float color, int x, int y)
    {
        buffer[matrix_lookup[y][x]] = color;
    }

    __attribute__((always_inline)) inline void set_pixel(float color, int x, int y)
    {
        if(y < 0 || y >= screen_height || x < 0 || x >= screen_width) {
            return;
        }
        set_pixel_unclipped(color, x, y);
    }
};

//////////////////////////////////////////////////////////////////////

extern graphics_t gfx;
extern graphics_t gfx2;
