//////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstdio>

#include "nvs_handle.hpp"

#include "settings.h"
#include "console.h"
#include "util.h"

LOG_CONTEXT("settings");

#define SETTINGS_NAMESPACE "settings"
#define SETTINGS_KEY "data"

settings_t const default_settings;

settings_t loaded_settings;
settings_t settings;

using nvs::NVSHandle;

//////////////////////////////////////////////////////////////////////

namespace
{
    template <typename T> esp_err_t save_setting(NVSHandle *nvs_handle, char const *key, char const *name, T const &value)
    {
        return nvs_handle->set_item(key, value);
    }

    template <> esp_err_t save_setting(NVSHandle *nvs_handle, char const *key, char const *name, timezone_location_t const &value)
    {
        return nvs_handle->set_string(key, value.name);
    }

    template <typename T> esp_err_t load_setting(NVSHandle *nvs_handle, char const *key, char const *name, T &value)
    {
        return nvs_handle->get_item(key, value);
    }

    template <> esp_err_t load_setting(NVSHandle *nvs_handle, char const *key, char const *name, timezone_location_t &value)
    {
        return nvs_handle->get_string(key, value.name, sizeof(value.name));
    }
}    // namespace

//////////////////////////////////////////////////////////////////////

esp_err_t settings_t::save()
{
    LOG_INFO("Saving settings");

    esp_err_t err;
    std::unique_ptr<NVSHandle> nvs_handle = nvs::open_nvs_handle(SETTINGS_NAMESPACE, NVS_READWRITE, &err);
    if(err != ESP_OK) {
        LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    char key_name[5];
    int saved = 0;

#undef X
#define X(ID, NAME, TYPE, VALUE)                                                            \
    if(settings.NAME != loaded_settings.NAME) {                                             \
        sprintf(key_name, "%d", ID);                                                        \
        LOG_INFO("  Save [%s] %s (%s)", key_name, #NAME, setting_to_string(settings.NAME)); \
        save_setting(nvs_handle.get(), key_name, #NAME, settings.NAME);                     \
        loaded_settings.NAME = settings.NAME;                                               \
        saved += 1;                                                                         \
    }

    SETTINGS_FIELDS

    err = nvs_handle->commit();
    if(err != ESP_OK) {
        LOG_ERROR("Failed to commit settings to NVS: %s", esp_err_to_name(err));
    } else {
        LOG_INFO("%d settings saved", saved);
    }
    return err;
}

//////////////////////////////////////////////////////////////////////

esp_err_t settings_t::load()
{
    LOG_INFO("Loading settings");

    esp_err_t err;
    auto nvs_handle = nvs::open_nvs_handle(SETTINGS_NAMESPACE, NVS_READONLY, &err);
    if(err != ESP_OK) {
        LOG_ERROR("Can't find saved settings: (%s)", esp_err_to_name(err));
        return err;
    }

    char key_name[5];

#undef X
#define X(ID, NAME, TYPE, VALUE)                                                                         \
    {                                                                                                    \
        sprintf(key_name, "%d", ID);                                                                     \
        if(load_setting(nvs_handle.get(), key_name, #NAME, loaded_settings.NAME) == ESP_OK) {            \
            LOG_INFO("  Loaded [%s] %s = %s", key_name, #NAME, setting_to_string(loaded_settings.NAME)); \
        }                                                                                                \
    }

    SETTINGS_FIELDS

    settings = loaded_settings;

    LOG_INFO("Settings loaded OK");

    return err;
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"brightness", "show/set brightness", "auto|fixed 0..255">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<auto_brightness_t> names[] = {
            "auto", auto_brightness_t::Auto,     //
            "fixed", auto_brightness_t::Fixed    //
        };
        auto_brightness_t auto_brightness = auto_brightness_t::Auto;
        int brightness = -1;
        if(argc == 1) {
            printf("%s %d\n", get_enum_name(names, settings.auto_brightness), settings.brightness);
            return;
        } else if(argc == 3 && find_enum(argv[1], names, auto_brightness)) {
            brightness = atoi(argv[2]);
            if(brightness > 255) {
                brightness = 255;
            }
        }
        if(brightness >= 0) {
            settings.brightness = (uint8_t)brightness;
            settings.auto_brightness = auto_brightness;
        } else {
            usage();
        }
    }
} cmd_brightness;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"seconds", "show/set seconds display mode", "long|medium|short|fixed|single">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<seconds_mode_t> names[] = {
            "long",   seconds_mode_t::Long,      //
            "medium", seconds_mode_t::Medium,    //
            "short",  seconds_mode_t::Short,     //
            "fixed",  seconds_mode_t::Fixed,     //
            "single", seconds_mode_t::Single,
        };
        if(argc == 1) {
            printf("%s\n", get_enum_name(names, settings.seconds_mode));
        } else if(!(argc == 2 && find_enum(argv[1], names, settings.seconds_mode))) {
            usage();
        }
    }
} cmd_seconds;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"colon", "show/set colon display mode", "off|on|dim|pulse">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<colon_mode_t> names[] = {
            "off",   colon_mode_t::Off,      //
            "on",    colon_mode_t::On,       //
            "dim",   colon_mode_t::Dim,      //
            "pulse", colon_mode_t::Pulse,    //
        };
        if(argc == 1) {
            printf("%s\n", get_enum_name(names, settings.colon_mode));
        } else if(!(argc == 2 && find_enum(argv[1], names, settings.colon_mode))) {
            usage();
        }
    }
} cmd_colon;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"fade", "show/set fade between seconds", "off|low|medium|high">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<clock_fade_mode_t> names[] = {
            "off",    clock_fade_mode_t::Off,       //
            "low",    clock_fade_mode_t::Low,       //
            "medium", clock_fade_mode_t::Medium,    //
            "high",   clock_fade_mode_t::High,      //
        };
        if(argc == 1) {
            printf("%s\n", get_enum_name(names, settings.clock_fade_mode));
        } else if(!(argc == 2 && find_enum(argv[1], names, settings.clock_fade_mode))) {
            usage();
        }
    }

} cmd_fade;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"clock", "show/set 12/24 hour clock mode", "12|24">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<clock_mode_t> names[] = {
            "12", clock_mode_t::clock_12_hour,    //
            "24", clock_mode_t::clock_24_hour,    //
        };
        if(argc == 1) {
            printf("%s\n", get_enum_name(names, settings.clock_mode));
        } else if(!(argc == 2 && find_enum(argv[1], names, settings.clock_mode))) {
            usage();
        }
    }
} cmd_clock;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"save", "save settings", "">
{
    void on_command(int argc, char **argv) override
    {
        settings.save();
    }
} cmd_save;
