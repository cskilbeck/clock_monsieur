//////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

#include "esp_err.h"

#include "timezone_data.h"

// Thus far, settings are all either uint8_t (possibly via an enum class) or a timezone_location_t

struct timezone_location_t
{
    char name[LONGEST_TZ_LOCATION_NAME + 1] = { 0 };
};

inline bool operator==(timezone_location_t a, timezone_location_t b)
{
    return strcmp(a.name, b.name) == 0;
}

// !!!! returns a static buffer !!!!
template <typename T> inline char const *setting_to_string(T const &a)
{
    static char buffer[8];
    sprintf(buffer, "%u", (uint8_t)a);
    return buffer;
}

template <> inline char const *setting_to_string(timezone_location_t const &a)
{
    return a.name;
}

//////////////////////////////////////////////////////////////////////
// ONLY ADD to this list, NEVER REMOVE anything

// #define X(ID, NAME, TYPE, DEFAULT_VALUE)

#define SETTINGS_FIELDS                                                 \
    X(0, clock_mode, clock_mode_t, clock_mode_t::clock_12_hour)         \
    X(1, auto_brightness, auto_brightness_t, auto_brightness_t::on)     \
    X(2, brightness, uint8_t, 128)                                      \
    X(3, seconds_mode, seconds_mode_t, seconds_mode_t::tail_long)       \
    X(4, colon_mode, colon_mode_t, colon_mode_t::on)                    \
    X(5, clock_fade_mode, clock_fade_mode_t, clock_fade_mode_t::medium) \
    X(6, clock_font, clock_font_t, clock_font_t::modern)                \
    X(7, timezone_mode, timezone_mode_t, timezone_mode_t::automatic)    \
    X(8, location, timezone_location_t, 0)

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

enum class clock_font_t : uint8_t
{
    normal = 0,
    modern = 1
};

//////////////////////////////////////////////////////////////////////

enum class timezone_mode_t : uint8_t
{
    automatic = 0,
    selected = 1
};

//////////////////////////////////////////////////////////////////////

enum class settings_field_id_t : uint8_t
{
#undef X
#define X(ID, NAME, TYPE, VALUE) NAME = ID,

    SETTINGS_FIELDS
};

//////////////////////////////////////////////////////////////////////

struct settings_t
{
#undef X
#define X(ID, NAME, TYPE, VALUE) TYPE NAME{ VALUE };

    SETTINGS_FIELDS

    esp_err_t load();
    esp_err_t save();
};

extern settings_t settings;
