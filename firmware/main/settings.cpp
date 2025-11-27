//////////////////////////////////////////////////////////////////////

#include <cstdio>

#include "nvs_flash.h"
#include "nvs.h"

#include "settings.h"
#include "console.h"
#include "util.h"

LOG_CONTEXT("settings");

#define SETTINGS_NAMESPACE "settings"
#define SETTINGS_KEY "data"

settings_t settings;

//////////////////////////////////////////////////////////////////////

esp_err_t settings_t::save()
{
    LOG_INFO("Saving settings");

    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    DEFER(nvs_close(nvs_handle));

    err = nvs_set_blob(nvs_handle, SETTINGS_KEY, &settings, sizeof(settings));
    if(err != ESP_OK) {
        LOG_ERROR("Failed to write settings: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(nvs_handle);
    if(err != ESP_OK) {
        LOG_ERROR("Failed to commit settings to NVS: %s", esp_err_to_name(err));
    }
    LOG_INFO("Settings saved OK");
    return err;
}

//////////////////////////////////////////////////////////////////////

esp_err_t settings_t::load()
{
    LOG_INFO("Loading settings");

    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if(err != ESP_OK) {
        LOG_ERROR("Can't find saved settings: (%s)", esp_err_to_name(err));
        return err;
    }
    DEFER(nvs_close(nvs_handle));

    settings_t temp_settings;
    size_t size = sizeof(temp_settings);
    err = nvs_get_blob(nvs_handle, SETTINGS_KEY, &temp_settings, &size);
    if(err != ESP_OK) {
        LOG_ERROR("Failed to load settings: %s", esp_err_to_name(err));
        return err;
    }
    *this = temp_settings;
    LOG_INFO("Settings loaded OK");
    return err;
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"brightness", "show/set brightness", "auto|fixed 0..255">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<auto_brightness_t> names[] = {
            "auto", auto_brightness_t::on,     //
            "fixed", auto_brightness_t::off    //
        };
        auto_brightness_t auto_brightness = auto_brightness_t::on;
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
            "long",   seconds_mode_t::tail_long,      //
            "medium", seconds_mode_t::tail_medium,    //
            "short",  seconds_mode_t::tail_short,     //
            "fixed",  seconds_mode_t::fixed,          //
            "single", seconds_mode_t::single,
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
            "off",   colon_mode_t::off,      //
            "on",    colon_mode_t::on,       //
            "dim",   colon_mode_t::dim,      //
            "pulse", colon_mode_t::pulse,    //
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
            "off",    clock_fade_mode_t::off,       //
            "low",    clock_fade_mode_t::low,       //
            "medium", clock_fade_mode_t::medium,    //
            "high",   clock_fade_mode_t::high,      //
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
