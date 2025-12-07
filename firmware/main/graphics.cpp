//////////////////////////////////////////////////////////////////////

#include <esp_err.h>

#include <cstdio>
#include <cstring>

#include "stdio.h"
#include "display.h"
#include "graphics.h"
#include "settings.h"
#include "button.h"
#include "util.h"
#include "settings.h"

LOG_CONTEXT("graphics");

//////////////////////////////////////////////////////////////////////

namespace
{
    DRAM_ATTR uint8_t const matrix_lookup[7][26] = {
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

    DRAM_ATTR uint8_t const hour_lookup[12] = {
        235, 226, 26, 10, 90, 106, 111, 101, 153, 169, 218, 234,
    };

    DRAM_ATTR uint8_t const second_lookup[60] = {
        236, 231, 230, 229, 228, 227, 225, 248, 250, 249, 24,  25,  40,  42,  41,  8,   9,   56,  58,  57,
        88,  89,  72,  74,  104, 105, 107, 108, 109, 110, 96,  97,  98,  99,  100, 102, 103, 121, 122, 120,
        154, 152, 137, 138, 136, 170, 168, 185, 184, 217, 216, 201, 202, 200, 232, 233, 224, 239, 238, 237,
    };

    DRAM_ATTR uint8_t const seconds_hours_lookup[120] = {
        236, 235, 231, 231, 230, 230, 229, 229, 228, 228, 227, 226, 225, 225, 248, 248, 250, 250, 249, 249, 24,  26,  25,  25,
        40,  40,  42,  42,  41,  41,  8,   10,  9,   9,   56,  56,  58,  58,  57,  57,  88,  90,  89,  89,  72,  72,  74,  74,
        104, 104, 105, 106, 107, 107, 108, 108, 109, 109, 110, 110, 96,  111, 97,  97,  98,  98,  99,  99,  100, 100, 102, 101,
        103, 103, 121, 121, 122, 122, 120, 120, 154, 153, 152, 152, 137, 137, 138, 138, 136, 136, 170, 169, 168, 168, 185, 185,
        184, 184, 217, 217, 216, 218, 201, 201, 202, 202, 200, 200, 232, 232, 233, 234, 224, 224, 239, 239, 238, 238, 237, 237,
    };

    uint8_t constexpr debug_led_0 = 0x49;
    uint8_t constexpr debug_led_1 = 0xBA;

    static DRAM_ATTR bool debug_led[2] = { 0, 0 };

    //////////////////////////////////////////////////////////////////////
    // content_width is width of the content to show
    // screen_width is width of the display screen
    // content_pos is the origin X pos where the content was drawn (always either negative or 0)

    void calculate_scrollbar(int content_width, int content_pos, int *scrollbar_width_out, int *scrollbar_pos_out)
    {
        if(content_width <= screen_width) {
            *scrollbar_width_out = 0;
            *scrollbar_pos_out = 0;
            return;
        }
        float ratio = (float)screen_width / (float)content_width;
        float scrollbar_width = (float)(screen_width * ratio + 0.9999f);
        float max_scroll = (float)content_width - (float)screen_width;
        float current_scroll = (float)-content_pos;
        float normalized_pos = current_scroll / max_scroll;
        float scrollbar_track_length = (float)screen_width - scrollbar_width;
        float scrollbar_pos_float = normalized_pos * scrollbar_track_length;
        *scrollbar_width_out = (int)scrollbar_width;
        *scrollbar_pos_out = (int)(scrollbar_pos_float + 0.9999f);
    }


};    // namespace

// gfx is the primary graphics buffer
graphics_t gfx;

// gfx2 is for drawing the 'other second' clock display into
// so we can fade between it and gfx
graphics_t gfx2;

//////////////////////////////////////////////////////////////////////
// approximate pow(2.2) as x^2 - (x^2 - x^3) / 4
// and convert endianness for display buffer

IRAM_ATTR uint16_t gamma_get(uint16_t x)
{
    int x2 = (x * x) >> 11;
    int x3 = (x * x2) >> 11;
    int x2_2 = x2 - ((x2 - x3) >> 2);
    return __builtin_bswap16(x2_2);
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR uint16_t gamma_get(float x)
{
    float x2 = x * x;
    float x3 = x2 * x;
    float x2_2 = x2 - ((x2 - x3) * 0.25f);
    return __builtin_bswap16(x2_2 * 2047.5f);
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR uint8_t font_t::get_width(int c) const
{
    return widths[get_c(c)];
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR uint8_t const *font_t::get_bitmap(int c) const
{
    return bitmap + get_c(c) * font_height;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::measure_string(char const *str) const
{
    int width = 0;
    while(int c = *str++) {
        int w = get_width(c);
        width += w == 0 ? (font_width / 2 - 1) : w;
        width += 1;    // +1 for single pixel gap between chars
    }
    return width - 1;    // trim the trailing single pixel gap
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::draw_char(graphics_t &gfx, int c, int x, int y, float color) const
{
    uint8_t char_width = get_width(c);
    if(char_width == 0) {
        return (font_width / 2) - 1;
    }

    // fully clipped?
    if(y <= -font_height) {
        return char_width;
    }

    int x_end = x + char_width;
    if(x_end <= 0) {
        return char_width;
    }

    uint8_t const *char_data = get_bitmap(c);

    // y clip
    int y_end = y + font_height;
    if(y < 0) {
        int clip = -y;
        char_data += clip;
        y_end = font_height - clip;
        y = 0;
    }
    if(y_end > screen_height) {
        y_end = screen_height;
    }

    // x clip
    int shift = 0;
    if(x < 0) {
        shift -= x;
        x = 0;
    }
    if(x_end >= screen_width) {
        x_end = screen_width - 1;
    }

    // draw it
    for(; y < y_end; ++y) {
        uint8_t row = *char_data++;
        row <<= shift;
        for(int sx = x; sx <= x_end && row != 0; ++sx) {
            if((row & 0x80) != 0) {
                gfx.buffer[matrix_lookup[y][sx]] = color;
            }
            row <<= 1;
        }
    }
    return char_width;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::draw_char_centered(graphics_t &gfx, int c, int x, int y, float color) const
{
    return draw_char(gfx, c, x - get_width(c) / 2, y, color);
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::draw_string(graphics_t &gfx, char const *str, int x, int y, float color) const
{
    int left = x;
    while(int c = *str++) {
        x += draw_char(gfx, c, x, y, color) + 1;
    }
    return x - left - 1;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void font_t::draw_long_string(graphics_t &gfx, char const *text, int x, int y, float color, float scrollbar_color) const
{
    int width = draw_string(gfx, text, x, y, color);
    if(width > screen_width) {
        int scrollbar_width;
        int scrollbar_pos;
        calculate_scrollbar(width, x, &scrollbar_width, &scrollbar_pos);
        int scrollbar_end = scrollbar_pos + scrollbar_width;
        for(int sx = scrollbar_pos; sx < scrollbar_end; ++sx) {
            float p = min(1.0f, gfx.get_pixel(sx, 0) + scrollbar_color);
            gfx.set_pixel(p, sx, 0);
        }
    }
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::clear()
{
    memset(buffer, 0, sizeof(buffer));
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::clear_matrix()
{
    for(auto &y : matrix_lookup) {
        for(auto x : y) {
            buffer[x] = 0.0f;
        }
    }
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::set_led(float color, uint8_t pixel)
{
    buffer[pixel] = color;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::set_pixel(float color, int x, int y)
{
    if(y < 0 || y >= screen_height || x < 0 || x >= screen_width) {
        return;
    }
    buffer[matrix_lookup[y][x]] = color;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR float graphics_t::get_pixel(int x, int y)
{
    if(y < 0 || y >= screen_height || x < 0 || x >= screen_width) {
        return 0.0f;
    }
    return buffer[matrix_lookup[y][x]];
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::set_second(float color, uint8_t second)
{
    if(second > 59) {
        second = 59;
    }
    uint8_t const *s = seconds_hours_lookup + second * 2;
    buffer[s[0]] = color;
    buffer[s[1]] = color;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::set_second_only(float color, uint8_t second)
{
    if(second > 59) {
        second = 59;
    }
    buffer[second_lookup[second]] = color;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::draw_time(int hours, int minutes, float color)
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
    font_t const &font = settings.clock_font == clock_font_t::modern ? square_font : font_5x7_font;
    font.draw_char_centered(*this, time_buffer[0], 2, 0, color);
    font.draw_char_centered(*this, time_buffer[1], 8, 0, color);
    font.draw_char_centered(*this, time_buffer[2], 16, 0, color);
    font.draw_char_centered(*this, time_buffer[3], 22, 0, color);
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::draw_colon(int seconds, float color)
{
    float colon_color{};
    switch(settings.colon_mode) {
    case colon_mode_t::off:
        colon_color = 0.0f;
        break;
    case colon_mode_t::on:
        colon_color = color;
        break;
    case colon_mode_t::dim:
        colon_color = color * 0.5f;
        break;
    case colon_mode_t::pulse:
        colon_color = (seconds & 1) == 0 ? 0 : color * 0.75f;
        break;
    }
    buffer[matrix_lookup[2][12]] = colon_color;
    buffer[matrix_lookup[4][12]] = colon_color;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::draw_seconds(int current_second)
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
            set_second(0.2f, i);
        }
        break;
    case seconds_mode_t::single:
        for(int i = 0; i < 60; ++i) {
            set_second(0.2f, i);
        }
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

void graphics_t::set_debug(int led, int value)
{
    debug_led[led & 1] = value;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::fade_to(graphics_t &other, float scale)
{
    float o = 1.0f - scale;
    for(int i = 0; i < 256; ++i) {
        float src = buffer[i] * o;
        float dst = other.buffer[i] * scale;
        ::display->led[i] = gamma_get(src + dst);
    }
    ::display->led[debug_led_0] = debug_led[0];
    ::display->led[debug_led_1] = debug_led[1];
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::display()
{
    for(int i = 0; i < 256; ++i) {
        ::display->led[i] = gamma_get(buffer[i]);
    }
    ::display->led[debug_led_0] = debug_led[0];
    ::display->led[debug_led_1] = debug_led[1];
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR void graphics_t::draw_clock(long seconds, float clock_color)
{
    int hours = seconds / 3600;
    int hour_seconds = seconds % 3600;
    int minutes = hour_seconds / 60;
    int secs = hour_seconds % 60;
    clear();
    draw_seconds(secs);
    draw_time(hours, minutes, clock_color);
    draw_colon(secs, clock_color);
}