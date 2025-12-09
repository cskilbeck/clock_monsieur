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
    X(1, auto_brightness, auto_brightness_t, auto_brightness_t::Auto)   \
    X(2, brightness, uint8_t, 128)                                      \
    X(3, seconds_mode, seconds_mode_t, seconds_mode_t::Long)            \
    X(4, colon_mode, colon_mode_t, colon_mode_t::On)                    \
    X(5, clock_fade_mode, clock_fade_mode_t, clock_fade_mode_t::Medium) \
    X(6, clock_font, clock_font_t, clock_font_t::Square)                \
    X(7, timezone_mode, timezone_mode_t, timezone_mode_t::Auto)         \
    X(8, location, timezone_location_t, 0)                              \
    X(9, ticks, tick_mode_t, tick_mode_t::Track)

//////////////////////////////////////////////////////////////////////
// Clock 12/24 hour display

enum class clock_mode_t : uint8_t
{
    clock_12_hour = 0,
    clock_24_hour = 1,
    max = 1
};

//////////////////////////////////////////////////////////////////////
// Use ambient light sensor to adjust brightness automatically

enum class auto_brightness_t : uint8_t
{
    Fixed = 0,
    Auto = 1,
    max = 1
};

//////////////////////////////////////////////////////////////////////
// How to display the seconds

enum class seconds_mode_t : uint8_t
{
    Long = 0,
    Medium = 1,
    Short = 2,
    Fixed = 3,
    Single = 4,
    max = 4
};

//////////////////////////////////////////////////////////////////////
// How to display the hour ticks

enum class tick_mode_t : uint8_t
{
    Off = 0,
    On = 1,
    Dim = 2,
    Track = 3,
    max = 3
};

//////////////////////////////////////////////////////////////////////
// How to display the time colon separating hours:minutes

enum class colon_mode_t : uint8_t
{
    Off = 0,
    On = 1,
    Dim = 2,
    Pulse = 3,
    max = 3
};

//////////////////////////////////////////////////////////////////////
// How quickly to transition from one second to the next

enum class clock_fade_mode_t : uint8_t
{
    Off = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    max = 3
};

//////////////////////////////////////////////////////////////////////

enum class clock_font_t : uint8_t
{
    Normal = 0,
    Square = 1,
    max = 1
};

//////////////////////////////////////////////////////////////////////

enum class timezone_mode_t : uint8_t
{
    Auto = 0,
    Select = 1,
    max = 1
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
