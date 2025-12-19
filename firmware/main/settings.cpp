//////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstdio>

#include "nvs_handle.hpp"
#include <nvs_flash.h>

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

uint8_t settings_t::get_brightness() const
{
    switch(brightness) {
    case brightness_level_t::Min:
        return 32;
    case brightness_level_t::Low:
        return 64;
    case brightness_level_t::Medium:
        return 128;
    case brightness_level_t::High:
        return 176;
    case brightness_level_t::Max:
        return 255;
    default:
        return 128;
    }
}

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

struct : console_command_t<"brightness", "show/set brightness", enum_names<auto_brightness_t, brightness_level_t>()>
{
    void on_command(int argc, char **argv) override
    {
        auto_brightness_t auto_brightness = auto_brightness_t::Auto;
        brightness_level_t brightness_level = brightness_level_t::High;
        if(argc == 1) {
            printf("%s %s\n", enum_to_string(settings.auto_brightness), enum_to_string(settings.brightness));
        } else if(argc == 3 && find_enum(argv[1], auto_brightness) && find_enum(argv[2], brightness_level)) {
            settings.brightness = brightness_level;
            settings.auto_brightness = auto_brightness;
        } else {
            usage();
        }
    }
} cmd_brightness;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"seconds", "show/set seconds display mode", enum_names<seconds_mode_t>()>
{
    void on_command(int argc, char **argv) override
    {
        if(argc == 1) {
            printf("%s\n", enum_to_string(settings.seconds_mode));
        } else if(!(argc == 2 && find_enum(argv[1], settings.seconds_mode))) {
            usage();
        }
    }
} cmd_seconds;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"colon", "show/set colon display mode", enum_names<colon_mode_t>()>
{
    void on_command(int argc, char **argv) override
    {
        if(argc == 1) {
            printf("%s\n", enum_to_string(settings.colon_mode));
        } else if(!(argc == 2 && find_enum(argv[1], settings.colon_mode))) {
            usage();
        }
    }
} cmd_colon;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"fade", "show/set fade between seconds", enum_names<clock_fade_mode_t>()>
{
    void on_command(int argc, char **argv) override
    {
        if(argc == 1) {
            printf("%s\n", enum_to_string(settings.clock_fade_mode));
        } else if(!(argc == 2 && find_enum(argv[1], settings.clock_fade_mode))) {
            usage();
        }
    }

} cmd_fade;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"clock", "show/set 12/24 hour clock mode", enum_names<clock_mode_t>()>
{
    void on_command(int argc, char **argv) override
    {
        if(argc == 1) {
            printf("%s\n", enum_to_string(settings.clock_mode));
        } else if(!(argc == 2 && find_enum(argv[1], settings.clock_mode))) {
            usage();
        }
    }
} cmd_clock;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"font", "show/set clock font", enum_names<clock_font_t>()>
{
    void on_command(int argc, char **argv) override
    {
        if(argc == 1) {
            printf("%s\n", enum_to_string(settings.clock_font));
        } else if(!(argc == 2 && find_enum(argv[1], settings.clock_font))) {
            usage();
        }
    }
} cmd_font;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"save", "save settings", "">
{
    void on_command(int argc, char **argv) override
    {
        settings.save();
    }
} cmd_save;

//////////////////////////////////////////////////////////////////////

void settings_init()
{
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG_ERR(ret);
    settings.load();
}