#pragma once

#include <cstdint>

enum class clock_mode_t : uint8_t
{
    clock_mode_12_hour = 0,
    clock_mode_24_hour = 1
};

enum class brightness_mode_t : uint8_t
{
    brightness_mode_low = 0,
    brightness_mode_medium = 1,
    brightness_mode_high = 2
};

enum class seconds_mode_t : uint8_t
{
    seconds_mode_tail_long = 0,
    seconds_mode_tail_medium = 1,
    seconds_mode_tail_short = 2,
    seconds_mode_fixed = 3,
    seconds_mode_single = 4
};

struct settings_t
{
    clock_mode_t clock_mode{ clock_mode_t::clock_mode_12_hour };
    brightness_mode_t seconds_brightness_mode{ brightness_mode_t::brightness_mode_medium };
    brightness_mode_t clock_brightness_mode{ brightness_mode_t::brightness_mode_medium };
    seconds_mode_t seconds_mode{ seconds_mode_t::seconds_mode_tail_long };
};

extern settings_t settings;
