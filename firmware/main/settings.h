//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#include <cstdint>

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
// Brightness of display

enum class brightness_mode_t : uint8_t
{
    low = 0,
    medium = 1,
    high = 2
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

enum class second_snap_mode_t : uint8_t
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
    brightness_mode_t brightness_mode{ brightness_mode_t::high };
    seconds_mode_t seconds_mode{ seconds_mode_t::tail_long };
    colon_mode_t colon_mode{ colon_mode_t::on };
    second_snap_mode_t second_snap_mode{ second_snap_mode_t::medium };
};

extern settings_t settings;
