//////////////////////////////////////////////////////////////////////

#include <esp_err.h>

#include <cstdio>
#include <cstring>

#include "stdio.h"
#include "display.h"
#include "font.h"
#include "graphics.h"
#include "settings.h"
#include "button.h"
#include "util.h"
#include "settings.h"

LOG_CONTEXT("font");

//////////////////////////////////////////////////////////////////////

namespace
{
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
        float ratio = (float)screen_width / content_width;
        float scrollbar_width = screen_width * ratio + 0.9999f;
        float max_scroll = (float)content_width - screen_width;
        float normalized_pos = -content_pos / max_scroll;
        float scrollbar_track_length = screen_width - scrollbar_width;
        float scrollbar_pos_float = normalized_pos * scrollbar_track_length;
        *scrollbar_width_out = (int)scrollbar_width;
        *scrollbar_pos_out = (int)(scrollbar_pos_float + 0.9999f);
    }
}    // namespace

//////////////////////////////////////////////////////////////////////

IRAM_ATTR uint8_t const *font_t::get_bitmap(int c) const
{
    return bitmap + get_c(c) * font_height;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::char_width(int c) const
{
    int w = widths[get_c(c)];
    return w == 0 ? font_width / 2 - 1 : w;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::measure_string(char const *str) const
{
    int total_width = 0;
    while(int c = *str++) {
        total_width += char_width(c) + 1;
    }
    return total_width - 1;    // trim the trailing single pixel gap
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::draw_char(graphics_t &gfx, int c, int x, int y, float color) const
{
    uint8_t c_width = char_width(c);

    // fully clipped?
    if(y <= -font_height) {
        return c_width;
    }

    int x_end = x + c_width;
    if(x_end <= 0) {
        return c_width;
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
                gfx.set_pixel_unclipped(color, sx, y);
            }
            row <<= 1;
        }
    }
    return c_width;
}

//////////////////////////////////////////////////////////////////////

IRAM_ATTR int font_t::draw_char_centered(graphics_t &gfx, int c, int x, int y, float color) const
{
    return draw_char(gfx, c, x - char_width(c) / 2, y, color);
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
    int s_width = draw_string(gfx, text, x, y, color);
    if(s_width > screen_width) {
        int scrollbar_width;
        int scrollbar_pos;
        calculate_scrollbar(s_width, x, &scrollbar_width, &scrollbar_pos);
        int scrollbar_end = scrollbar_pos + scrollbar_width;
        for(int sx = scrollbar_pos; sx < scrollbar_end; ++sx) {
            float p = min(1.0f, gfx.get_pixel(sx, 0) + scrollbar_color);
            gfx.set_pixel(p, sx, 0);
        }
    }
}
