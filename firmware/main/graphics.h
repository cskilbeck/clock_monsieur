//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#include <cstdint>
#include "display.h"
#include "font_5x6.h"
#include "font_5x7.h"

//////////////////////////////////////////////////////////////////////

uint16_t gamma_get(uint16_t x);
uint16_t gamma_get(float x);

//////////////////////////////////////////////////////////////////////

struct graphics_t;

struct font_t
{
    uint8_t font_width;
    uint8_t font_height;
    const uint8_t *bitmap;
    const uint8_t *bounds;

    struct bounds_t
    {
        uint8_t bounds;

        uint8_t low() const
        {
            return bounds & 0xf;
        }
        uint8_t high() const
        {
            return bounds >> 4;
        }

        uint8_t width() const
        {
            return high() - low();
        }

        uint8_t safe_width() const
        {
            uint8_t w = width();
            return w == 0 ? 3 : w;
        }
    };

    bounds_t get_bounds(int c) const;
    int measure_string(char const *str) const;
    int draw_char(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_char_centered(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_string(graphics_t &gfx, char const *str, int x, int y, float color) const;
};

//////////////////////////////////////////////////////////////////////

struct graphics_t
{
    float buffer[256];

    void clear();
    void set_pixel(float color, uint8_t n);
    void set_second(float color, uint8_t second);
    void draw_time(int hours, int minutes, float color);
    void draw_colon(int seconds);
    void draw_seconds(int current_second);
    void draw_clock(long seconds);
    void display();
    void fade_to(graphics_t &other, float scale);
};

//////////////////////////////////////////////////////////////////////

extern graphics_t gfx;
extern graphics_t gfx2;
