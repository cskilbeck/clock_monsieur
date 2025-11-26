//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#include <cstdint>

//////////////////////////////////////////////////////////////////////

enum class settings_field_t : uint8_t
{
    clock_mode = 0,
    auto_brightness = 1,
    brightness = 2,
    seconds_mode = 3,
    colon_mode = 4,
    clock_fade_mode = 5,
};

//////////////////////////////////////////////////////////////////////
// Clock 12/24 hour display

enum class clock_mode_t : uint8_t
{
    clock_12_hour = 0,
    clock_24_hour = 1
};

//////////////////////////////////////////////////////////////////////
// Use ambient light sensor to adjust brightness automatically

enum class auto_brightness_t : uint8_t
{
    off = 0,
    on = 1
};

//////////////////////////////////////////////////////////////////////
// How to display the seconds

enum class seconds_mode_t : uint8_t
{
    tail_long = 0,
    tail_medium = 1,
    tail_short = 2,
    fixed = 3,
    single = 4
};

//////////////////////////////////////////////////////////////////////
// How to display the time colon separating hours:minutes

enum class colon_mode_t : uint8_t
{
    off = 0,
    on = 1,
    dim = 2,
    pulse = 3
};

//////////////////////////////////////////////////////////////////////
// How quickly to transition from one second to the next

enum class clock_fade_mode_t : uint8_t
{
    off = 0,
    low = 1,
    medium = 2,
    high = 3
};

//////////////////////////////////////////////////////////////////////

struct settings_t
{
    clock_mode_t clock_mode{ clock_mode_t::clock_12_hour };
    auto_brightness_t auto_brightness{ auto_brightness_t::on };
    uint8_t brightness{ 128 };
    seconds_mode_t seconds_mode{ seconds_mode_t::tail_long };
    colon_mode_t colon_mode{ colon_mode_t::on };
    clock_fade_mode_t clock_fade_mode{ clock_fade_mode_t::medium };

    esp_err_t load();
    esp_err_t save();
};

extern settings_t settings;
