//////////////////////////////////////////////////////////////////////

#pragma once

#include "util.h"
#include "timezone_data.h"

#include "esp_err.h"

//////////////////////////////////////////////////////////////////////

ENUM(alarm_enabled_t, uint8_t, Off, On)

//////////////////////////////////////////////////////////////////////

ENUM(alarm_mode_t, uint8_t, EveryDay, Weekdays, Once)

//////////////////////////////////////////////////////////////////////

ENUM(alarm_melody_t, uint8_t, Gentle, Standard, Urgent, Cuckoo, Charge, Birthday)

//////////////////////////////////////////////////////////////////////
// ONLY ADD to this list, NEVER REMOVE anything

// #define X(ID, NAME, TYPE, DEFAULT_VALUE)

#define SETTINGS_FIELDS                                               \
    X(0, clock_mode, clock_mode_t, clock_mode_t::clock_12_hour)       \
    X(1, auto_brightness, auto_brightness_t, auto_brightness_t::Auto) \
    X(3, seconds_mode, seconds_mode_t, seconds_mode_t::Long)          \
    X(4, colon_mode, colon_mode_t, colon_mode_t::On)                  \
    X(5, clock_fade_mode, clock_fade_mode_t, clock_fade_mode_t::High) \
    X(6, clock_font, clock_font_t, clock_font_t::Normal)              \
    X(7, timezone_mode, timezone_mode_t, timezone_mode_t::Auto)       \
    X(8, location, timezone_location_t, 0)                            \
    X(9, ticks, tick_mode_t, tick_mode_t::Track)                      \
    X(10, brightness, brightness_level_t, brightness_level_t::Medium) \
    X(11, timer_seconds, uint16_t, 60)                                \
    X(12, alarm_hour, uint8_t, 7)                                     \
    X(13, alarm_minute, uint8_t, 0)                                   \
    X(14, alarm_mode, alarm_mode_t, alarm_mode_t::EveryDay)           \
    X(15, alarm_melody, alarm_melody_t, alarm_melody_t::Charge)       \
    X(16, alarm_snooze, uint8_t, 5)                                   \
    X(17, alarm_enabled, alarm_enabled_t, alarm_enabled_t::Off)

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

template <> inline char const *setting_to_string(uint16_t const &a)
{
    static char buffer[8];
    sprintf(buffer, "%u", a);
    return buffer;
}

template <> inline char const *setting_to_string(timezone_location_t const &a)
{
    return a.name;
}

//////////////////////////////////////////////////////////////////////
// Clock 12/24 hour display
// Can't use the wacky macros because 12/24 are not valid identifiers

enum class clock_mode_t : uint8_t
{
    clock_12_hour,
    clock_24_hour
};

template <> [[maybe_unused]] constexpr char const *enum_to_string(clock_mode_t value)
{
    switch(value) {
    case clock_mode_t::clock_12_hour:
        return "12";
    case clock_mode_t::clock_24_hour:
        return "24";
    default:
        return nullptr;
    }
}

//////////////////////////////////////////////////////////////////////
// Use ambient light sensor to adjust brightness automatically

ENUM(auto_brightness_t, uint8_t, Fixed, Auto)

//////////////////////////////////////////////////////////////////////
// Brightness level

ENUM(brightness_level_t, uint8_t, Min, Low, Medium, High, Max)

//////////////////////////////////////////////////////////////////////
// How to display the seconds

ENUM(seconds_mode_t, uint8_t, Long, Medium, Short, Fixed, Single)

//////////////////////////////////////////////////////////////////////
// How to display the hour ticks

ENUM(tick_mode_t, uint8_t, Off, On, Dim, Track)

//////////////////////////////////////////////////////////////////////
// How to display the time colon separating hours:minutes

ENUM(colon_mode_t, uint8_t, Off, On, Dim, Pulse)

//////////////////////////////////////////////////////////////////////
// How quickly to transition from one second to the next

ENUM(clock_fade_mode_t, uint8_t, Off, Low, Medium, High)

//////////////////////////////////////////////////////////////////////

ENUM(clock_font_t, uint8_t, Normal, Square)

//////////////////////////////////////////////////////////////////////

ENUM(timezone_mode_t, uint8_t, Auto, Select)

//////////////////////////////////////////////////////////////////////

enum class settings_field_id_t : uint8_t
{
#undef X
#define X(ID, NAME, TYPE, VALUE) NAME = ID,

    SETTINGS_FIELDS
};

//////////////////////////////////////////////////////////////////////

struct brightness_range_t
{
    uint8_t min_brightness;
    uint8_t max_brightness;
};

//////////////////////////////////////////////////////////////////////

struct settings_t
{
#undef X
#define X(ID, NAME, TYPE, VALUE) TYPE NAME{ VALUE };

    SETTINGS_FIELDS

    brightness_range_t get_brightness_range() const;

    esp_err_t load();
    esp_err_t save();
};

//////////////////////////////////////////////////////////////////////

extern settings_t settings;

//////////////////////////////////////////////////////////////////////

void settings_init();
