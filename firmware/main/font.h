#pragma once

#include <cstdint>

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
    int char_width(int c) const;
    int measure_string(char const *str) const;
    int draw_char(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_char_centered(graphics_t &gfx, int c, int x, int y, float color) const;
    int draw_string(graphics_t &gfx, char const *str, int x, int y, float color) const;
    void draw_long_string(graphics_t &gfx, char const *text, int x, int y, float color, float scrollbar_color) const;
};
