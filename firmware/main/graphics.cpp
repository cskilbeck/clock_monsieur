//////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "stdio.h"
#include "display.h"
#include "graphics.h"
#include "settings.h"
#include "font_6x7.h"

//////////////////////////////////////////////////////////////////////

uint16_t const matrix_lookup[7][26] = {
    { 0xC7, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xC0, 0xCF, 0xCE, 0xCD, 0xCC, 0xCB,
      0xF7, 0xF6, 0xF5, 0xF4, 0xF3, 0xF2, 0xF1, 0xF0, 0xFF, 0xFE, 0xFD, 0xFC, 0xFB },

    { 0xD7, 0xD6, 0xD5, 0xD4, 0xD3, 0xD2, 0xD1, 0xD0, 0xDF, 0xDE, 0xDD, 0xDC, 0xDB,
      0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x1F, 0x1E, 0x1D, 0x1C, 0x1B },

    { 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xBF, 0xBE, 0xBD, 0xBC, 0xBB,
      0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B },

    { 0xA7, 0xA6, 0xA5, 0xA4, 0xA3, 0xA2, 0xA1, 0xA0, 0xAF, 0xAE, 0xAD, 0xAC, 0xAB,
      0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B },

    { 0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x8F, 0x8E, 0x8D, 0x8C, 0x8B,
      0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x3F, 0x3E, 0x3D, 0x3C, 0x3B },

    { 0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x9F, 0x9E, 0x9D, 0x9C, 0x9B,
      0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, 0x50, 0x5F, 0x5E, 0x5D, 0x5C, 0x5B },

    { 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71, 0x70, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B,
      0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x40, 0x4F, 0x4E, 0x4D, 0x4C, 0x4B },
};

uint16_t const hour_lookup[12] = {
    235, 226, 26, 10, 90, 106, 111, 101, 153, 169, 218, 234,
};

uint16_t const second_lookup[60] = {
    236, 231, 230, 229, 228, 227, 225, 248, 250, 249, 24,  25,  40,  42,  41,  8,   9,   56,  58,  57,
    88,  89,  72,  74,  104, 105, 107, 108, 109, 110, 96,  97,  98,  99,  100, 102, 103, 121, 122, 120,
    154, 152, 137, 138, 136, 170, 168, 185, 184, 217, 216, 201, 202, 200, 232, 233, 224, 239, 238, 237,
};

uint16_t const seconds_hours_lookup[120] = {
    236, 235, 231, 231, 230, 230, 229, 229, 228, 228, 227, 226, 225, 225, 248, 248, 250, 250, 249, 249, 24,  26,  25,  25,
    40,  40,  42,  42,  41,  41,  8,   10,  9,   9,   56,  56,  58,  58,  57,  57,  88,  90,  89,  89,  72,  72,  74,  74,
    104, 104, 105, 106, 107, 107, 108, 108, 109, 109, 110, 110, 96,  111, 97,  97,  98,  98,  99,  99,  100, 100, 102, 101,
    103, 103, 121, 121, 122, 122, 120, 120, 154, 153, 152, 152, 137, 137, 138, 138, 136, 136, 170, 169, 168, 168, 185, 185,
    184, 184, 217, 217, 216, 218, 201, 201, 202, 202, 200, 200, 232, 232, 233, 234, 224, 224, 239, 239, 238, 238, 237, 237,
};

graphics_t gfx;
graphics_t gfx2;

//////////////////////////////////////////////////////////////////////
// approximate pow(2.2) as x^2 - (x^2 - x^3) / 4
// and convert endianness for display buffer

uint16_t gamma_get(uint16_t x)
{
    int x2 = (x * x) >> 11;
    int x3 = (x * x2) >> 11;
    int x2_2 = x2 - ((x2 - x3) >> 2);
    return __builtin_bswap16(x2_2);
}

//////////////////////////////////////////////////////////////////////

uint16_t gamma_get(float x)
{
    float x2 = x * x;
    float x3 = x2 * x;
    float x2_2 = x2 - ((x2 - x3) * 0.25f);
    return __builtin_bswap16(x2_2 * 2047.5f);
}

//////////////////////////////////////////////////////////////////////

glyph_t const &get_glyph(int c)
{
    c -= 32;
    if(c < 0 || c > 95) {
        c = 95;
    }
    return font_6x7[c];
}

//////////////////////////////////////////////////////////////////////

void graphics_t::clear()
{
    memset(buffer, 0, sizeof(buffer));
}

//////////////////////////////////////////////////////////////////////

void graphics_t::set_pixel(float color, uint8_t pixel)
{
    buffer[pixel] = color;
}

//////////////////////////////////////////////////////////////////////

void graphics_t::set_second(float color, uint8_t second)
{
    if(second > 59) {
        second = 59;
    }
    uint16_t const *s = seconds_hours_lookup + second * 2;
    buffer[s[0]] = color;
    buffer[s[1]] = color;
}

//////////////////////////////////////////////////////////////////////

int graphics_t::draw_char(glyph_t const &glyph, int x, int y, float color)
{
    constexpr int glyph_height = 7;
    constexpr int glyph_width = 6;
    constexpr int screen_height = 7;
    constexpr int screen_width = 26;

    // fully clipped?
    if(y <= -glyph_height) {
        return glyph.width;
    }

    if(x <= -glyph.width) {
        return glyph.width;
    }

    uint8_t const *char_data = glyph.data;

    // y clip
    int y_end = y + glyph_height;
    if(y < 0) {
        int clip = -y;
        char_data += clip;
        y_end = glyph_height - clip;
        y = 0;
    }
    if(y_end > screen_height) {
        y_end = screen_height;
    }

    // x clip
    int shift = glyph.shift;
    int x_end = x + glyph.width;
    if(x < 0) {
        int clip = -x;
        shift += clip;
        x_end = glyph.width - clip;
        x = 0;
    }
    if(x_end > screen_width) {
        x_end = screen_width;
    }

    // draw it
    for(; y < y_end; ++y) {
        uint8_t row = *char_data++;
        row >>= shift;
        for(int sx = x; sx < x_end; ++sx) {
            if((row & 1) != 0) {
                buffer[matrix_lookup[y][sx]] = color;
            }
            row >>= 1;
        }
    }
    return glyph.width;
}

//////////////////////////////////////////////////////////////////////

int graphics_t::draw_char_centered(int c, int x, int y, float color)
{
    glyph_t const &glyph = get_glyph(c);
    return draw_char(glyph, x - glyph.width / 2, y, color);
}

//////////////////////////////////////////////////////////////////////

int graphics_t::draw_string(char const *str, int x, int y, float color)
{
    int width = 0;
    while(*str) {
        glyph_t const &c = get_glyph(*str);
        int w = draw_char(c, x, y, color) + 1;
        ++str;
        x += w;
        width += w;
    }
    return width;
}

//////////////////////////////////////////////////////////////////////

int measure_string(char const *str)
{
    int width = 0;
    while(int c = *str++) {
        c -= 32;
        if(c < 0 || c > 95) {
            c = 95;
        }
        width += font_6x7[c].width + 1;
    }
    return width;
}

//////////////////////////////////////////////////////////////////////

void graphics_t::draw_time(int hours, int minutes, float color)
{
    minutes %= 60;
    char const *fmt;
    if(settings.clock_mode == clock_mode_t::clock_24_hour) {
        hours %= 24;
        fmt = "%02d%02d";
    } else {
        hours %= 12;
        if(hours == 0) {
            hours = 12;
        }
        fmt = "%2d%02d";
    }
    char time_buffer[16];
    sprintf(time_buffer, fmt, hours, minutes);
    draw_char_centered(time_buffer[0], 2, 0, color);
    draw_char_centered(time_buffer[1], 8, 0, color);
    draw_char_centered(time_buffer[2], 16, 0, color);
    draw_char_centered(time_buffer[3], 22, 0, color);
}

//////////////////////////////////////////////////////////////////////

void graphics_t::draw_colon(int seconds)
{
    float colon_color{};
    switch(settings.colon_mode) {
    case colon_mode_t::off:
        colon_color = 0.0f;
        break;
    case colon_mode_t::on:
        colon_color = 1.0f;
        break;
    case colon_mode_t::dim:
        colon_color = 0.5f;
        break;
    case colon_mode_t::pulse:
        colon_color = (seconds & 1) == 0 ? 0 : 0.75f;
        break;
    }
    buffer[matrix_lookup[2][12]] = colon_color;
    buffer[matrix_lookup[4][12]] = colon_color;
}

//////////////////////////////////////////////////////////////////////

void graphics_t::draw_seconds(int current_second)
{
    auto draw_tail = [this, current_second](int TAIL_LENGTH) {
        float delta = 1.0f / TAIL_LENGTH;
        float intensity = 1.0f;
        float s = current_second;
        for(int i = 0; i < TAIL_LENGTH; ++i) {
            set_second(1.0f - i * delta, s);
            s -= 1;
            if(s < 0) {
                s += 60;
            }
        }
    };

    switch(settings.seconds_mode) {
    case seconds_mode_t::fixed:
        for(int i = 0; i <= current_second; ++i) {
            set_second(1.0f, i);
        }
        for(int i = current_second + 1; i < 60; ++i) {
            set_second(0.25f, i);
        }
        break;
    case seconds_mode_t::single:
        set_second(1.0f, current_second);
        break;
    case seconds_mode_t::tail_short:
        draw_tail(10);
        break;
    case seconds_mode_t::tail_medium:
        draw_tail(30);
        break;
    case seconds_mode_t::tail_long:
        draw_tail(59);
        break;
    }
}

//////////////////////////////////////////////////////////////////////

void graphics_t::fade_to(graphics_t &other, float scale)
{
    float o = 1.0f - scale;
    for(int i = 0; i < 256; ++i) {
        float src = buffer[i] * o;
        float dst = other.buffer[i] * scale;
        ::display->led[i] = gamma_get(src + dst);
    }
}

//////////////////////////////////////////////////////////////////////

void graphics_t::display()
{
    for(int i = 0; i < 256; ++i) {
        ::display->led[i] = gamma_get(buffer[i]);
    }
}

//////////////////////////////////////////////////////////////////////

void graphics_t::draw_clock(long seconds)
{
    int hours = seconds / 3600;
    int hour_seconds = seconds % 3600;
    int minutes = hour_seconds / 60;
    int secs = hour_seconds % 60;
    clear();
    draw_seconds(secs);
    draw_time(hours, minutes, 1);
    draw_colon(secs);
}