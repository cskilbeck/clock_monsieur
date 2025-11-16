#pragma once

#include <cstdint>
#include "display.h"

uint16_t gamma_get(uint16_t x);
uint16_t gamma_getf(float x);

int measure_string(const char *str);

struct graphics_t
{
    graphics_t(display_t &d) : display(d)
    {
    }

    void cls(uint16_t color);
    void set_pixel(uint16_t color, uint8_t n);
    void set_second(uint16_t color, uint8_t second);
    int draw_char(int c, int x, int y, int color);
    int draw_char_centered(int c, int x, int y, int color);
    int draw_string(const char *str, int x, int y, int color);
    void draw_time(int hours, int minutes, int color, int colon_color);
    void seconds_tail(int current_second, int current_microseconds);

    void fade_matrix(graphics_t &a, graphics_t &b, int scale);    // 0 .. 2048

    display_t &display;
};
