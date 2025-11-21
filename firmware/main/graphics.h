#pragma once

#include <cstdint>
#include "display.h"

uint16_t gamma_get(uint16_t x);
uint16_t gamma_get(float x);

struct glyph_t
{
    uint8_t width : 4;
    uint8_t shift : 4;
    uint8_t data[7];
};

int measure_string(char const *str);

struct graphics_t
{
    float buffer[256];

    void clear();
    void set_pixel(float color, uint8_t n);
    void set_second(float color, uint8_t second);
    int draw_char(glyph_t const &glyph, int x, int y, float color);
    int draw_char_centered(int c, int x, int y, float color);
    int draw_string(char const *str, int x, int y, float color);
    void draw_time(int hours, int minutes, float color);
    void draw_colon(int seconds);
    void draw_seconds(int current_second);

    void draw_clock(long seconds);
    void display(display_t &display);
    void fade_to(display_t &display, graphics_t &other, float scale);
};
