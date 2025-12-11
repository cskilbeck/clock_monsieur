//////////////////////////////////////////////////////////////////////

#pragma once

#include <cstring>

#include "timezone_data.h"

#include "esp_err.h"

//////////////////////////////////////////////////////////////////////
// ONLY ADD to this list, NEVER REMOVE anything

// #define X(ID, NAME, TYPE, DEFAULT_VALUE)

#define SETTINGS_FIELDS                                                 \
    X(0, clock_mode, clock_mode_t, clock_mode_t::clock_12_hour)         \
    X(1, auto_brightness, auto_brightness_t, auto_brightness_t::Auto)   \
    X(3, seconds_mode, seconds_mode_t, seconds_mode_t::Long)            \
    X(4, colon_mode, colon_mode_t, colon_mode_t::On)                    \
    X(5, clock_fade_mode, clock_fade_mode_t, clock_fade_mode_t::Medium) \
    X(6, clock_font, clock_font_t, clock_font_t::Square)                \
    X(7, timezone_mode, timezone_mode_t, timezone_mode_t::Auto)         \
    X(8, location, timezone_location_t, 0)                              \
    X(9, ticks, tick_mode_t, tick_mode_t::Track)                        \
    X(10, brightness, brightness_level_t, brightness_level_t::Max)

//////////////////////////////////////////////////////////////////////
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

template <typename T> [[maybe_unused]] static char const *enum_to_string(T value)
{
    return "?";
}

#define ENUM_NAMES(type)                                                \
    template <> [[maybe_unused]] char const *enum_to_string(type value) \
    {                                                                   \
        switch(value) {

#define ENAME(type, value) \
    case type::value:      \
        return #value

#define ENAME_TEXT(type, value, text) \
    case type::value:                 \
        return text

#define ENUM_NAMES_END \
    default:           \
        return "?";    \
        }              \
        return "?";    \
        }

//////////////////////////////////////////////////////////////////////
// Clock 12/24 hour display

enum class clock_mode_t : uint8_t
{
    clock_12_hour,
    clock_24_hour,
    ENUM_MAX
};

ENUM_NAMES(clock_mode_t)
ENAME_TEXT(clock_mode_t, clock_12_hour, "12 hour");
ENAME_TEXT(clock_mode_t, clock_24_hour, "24 hour");
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// Use ambient light sensor to adjust brightness automatically

enum class auto_brightness_t : uint8_t
{
    Fixed,
    Auto,
    ENUM_MAX
};

ENUM_NAMES(auto_brightness_t)
ENAME(auto_brightness_t, Fixed);
ENAME(auto_brightness_t, Auto);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// Brightness level

enum class brightness_level_t : uint8_t
{
    Min,
    Low,
    Medium,
    High,
    Max,
    ENUM_MAX
};

ENUM_NAMES(brightness_level_t)
ENAME(brightness_level_t, Min);
ENAME(brightness_level_t, Low);
ENAME(brightness_level_t, Medium);
ENAME(brightness_level_t, High);
ENAME(brightness_level_t, Max);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// How to display the seconds

enum class seconds_mode_t : uint8_t
{
    Long,
    Medium,
    Short,
    Fixed,
    Single,
    ENUM_MAX
};

ENUM_NAMES(seconds_mode_t)
ENAME(seconds_mode_t, Long);
ENAME(seconds_mode_t, Medium);
ENAME(seconds_mode_t, Short);
ENAME(seconds_mode_t, Fixed);
ENAME(seconds_mode_t, Single);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// How to display the hour ticks

enum class tick_mode_t : uint8_t
{
    Off,
    On,
    Dim,
    Track,
    ENUM_MAX
};

ENUM_NAMES(tick_mode_t)
ENAME(tick_mode_t, Off);
ENAME(tick_mode_t, On);
ENAME(tick_mode_t, Dim);
ENAME(tick_mode_t, Track);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// How to display the time colon separating hours:minutes

enum class colon_mode_t : uint8_t
{
    Off,
    On,
    Dim,
    Pulse,
    ENUM_MAX
};

ENUM_NAMES(colon_mode_t)
ENAME(colon_mode_t, Off);
ENAME(colon_mode_t, On);
ENAME(colon_mode_t, Dim);
ENAME(colon_mode_t, Pulse);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////
// How quickly to transition from one second to the next

enum class clock_fade_mode_t : uint8_t
{
    Off,
    Low,
    Medium,
    High,
    ENUM_MAX
};

ENUM_NAMES(clock_fade_mode_t)
ENAME(clock_fade_mode_t, Off);
ENAME(clock_fade_mode_t, Low);
ENAME(clock_fade_mode_t, Medium);
ENAME(clock_fade_mode_t, High);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////

enum class clock_font_t : uint8_t
{
    Normal,
    Square,
    ENUM_MAX
};

ENUM_NAMES(clock_font_t)
ENAME(clock_font_t, Normal);
ENAME(clock_font_t, Square);
ENUM_NAMES_END

//////////////////////////////////////////////////////////////////////

enum class timezone_mode_t : uint8_t
{
    Auto,
    Select,
    ENUM_MAX
};

ENUM_NAMES(timezone_mode_t)
ENAME(timezone_mode_t, Auto);
ENAME(timezone_mode_t, Select);
ENUM_NAMES_END

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

    uint8_t get_brightness() const;

    esp_err_t load();
    esp_err_t save();
};

//////////////////////////////////////////////////////////////////////

extern settings_t settings;

//////////////////////////////////////////////////////////////////////

void settings_init();
