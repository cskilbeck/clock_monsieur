#pragma once

#include <cstdint>

enum class clock_mode_t : uint8_t
{
    clock_12_hour = 0,
    clock_24_hour = 1
};

enum class brightness_mode_t : uint8_t
{
    low = 0,
    medium = 1,
    high = 2
};

enum class seconds_mode_t : uint8_t
{
    tail_long = 0,
    tail_medium = 1,
    tail_short = 2,
    fixed = 3,
    single = 4
};

enum class colon_mode_t : uint8_t
{
    off = 0,
    on = 1,
    dim = 2,
    pulse = 3
};

struct settings_t
{
    clock_mode_t clock_mode{ clock_mode_t::clock_12_hour };
    brightness_mode_t brightness_mode{ brightness_mode_t::high };
    seconds_mode_t seconds_mode{ seconds_mode_t::tail_long };
    colon_mode_t colon_mode{ colon_mode_t::on };
};

extern settings_t settings;
