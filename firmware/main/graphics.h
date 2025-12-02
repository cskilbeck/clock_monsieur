//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#include <cstdint>
#include "display.h"
#include "font_5x6.h"
#include "font_5x7.h"
#include "font_5x7_narrow.h"
#include "big_caps.h"

//////////////////////////////////////////////////////////////////////

uint16_t gamma_get(uint16_t x);
uint16_t gamma_get(float x);

//////////////////////////////////////////////////////////////////////

struct graphics_t;

struct font_t
{
    uint8_t font_width;
    uint8_t font_height;
    uint8_t const *bitmap;
    uint8_t const *widths;

    static uint8_t get_c(uint8_t c)
    {
        return c < 32 ? 95 : c - 32;
    }

    uint8_t const *get_bitmap(int c) const;
    uint8_t get_width(int c) const;

    int measure_string(char const *str) const;

    int draw_char(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_char_centered(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_string(graphics_t &gfx, char const *str, int x, int y, float color) const;

    // adds a scrollbar for the X pos/width
    void draw_long_string(graphics_t &gfx, char const *text, int x, int y, float color, float scrollbar_color) const;
};

//////////////////////////////////////////////////////////////////////

struct graphics_t
{
    float buffer[256];

    void clear();
    void clear_matrix();
    void set_led(float color, uint8_t n);
    void set_pixel(float color, int x, int y);
    float get_pixel(int x, int y);
    void set_second(float color, uint8_t second);
    void set_second_only(float color, uint8_t second);
    void draw_time(int hours, int minutes, float color);
    void draw_colon(int seconds, float color);
    void draw_seconds(int current_second);
    void draw_clock(long seconds, float clock_color);
    void display();
    void fade_to(graphics_t &other, float scale);

    void set_debug(int led, int value);
};

//////////////////////////////////////////////////////////////////////

extern graphics_t gfx;
extern graphics_t gfx2;

static int constexpr screen_width = 26;
static int constexpr screen_height = 7;