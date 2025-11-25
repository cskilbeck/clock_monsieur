//////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>
#include <ctype.h>

#include <freertos/FreeRTOS.h>
#include "nvs_flash.h"

#include "util.h"
#include "settings.h"
#include "console.h"

LOG_CONTEXT("console");

//////////////////////////////////////////////////////////////////////

console_command_base_t *&console_command_list()
{
    static console_command_base_t *cmd_list{ nullptr };
    return cmd_list;
}

namespace
{
    //////////////////////////////////////////////////////////////////////
    // https://stackoverflow.com/questions/9659697/parse-string-into-array-based-on-spaces-or-double-quotes-strings/9660047#9660047

    template <size_t MAX_ARGS> int parse_args(char *str, char *(&argv)[MAX_ARGS])
    {
        enum states
        {
            SCAN,
            IN_WORD,
            IN_STRING
        } state = SCAN;

        char *arg = nullptr;

        int argc = 0;

        for(char *p = str; *p != 0; ++p) {
            int c = *p;
            switch(state) {
            case SCAN:
                if(c == '"') {
                    state = IN_STRING;
                    arg = p + 1;
                } else if(!isspace(c)) {
                    state = IN_WORD;
                    arg = p;
                }
                break;

            case IN_STRING:
                if(c == '"') {
                    *p = 0;
                    argv[argc++] = arg;
                    if(argc == MAX_ARGS) {
                        return argc;
                    }
                    state = SCAN;
                }
                break;

            case IN_WORD:
                if(isspace(c)) {
                    *p = 0;
                    argv[argc++] = arg;
                    if(argc == MAX_ARGS) {
                        return argc;
                    }
                    state = SCAN;
                }
                break;
            }
        }
        if(state != SCAN && arg != nullptr) {
            argv[argc++] = arg;
        }
        return argc;
    }

    //////////////////////////////////////////////////////////////////////

    void handle_command(char *cmd)
    {
        char *argv[16];
        int argc = parse_args(cmd, argv);
        printf(">");
        for(int i = 0; i < argc; ++i) {
            printf(" %s", argv[i]);
        }
        printf("\n");
        if(argc != 0) {
            for(console_command_base_t *c = console_command_list(); c != nullptr; c = c->next) {
                if(strcmp(c->name(), argv[0]) == 0) {
                    c->on_command(argc, argv);
                    return;
                }
            }
            printf("Unknown command: %s\n", argv[0]);
        }
    }

    //////////////////////////////////////////////////////////////////////

    void console_task(void *pvParameter)
    {
        delay_ms(200);
        LOG_INFO("CONSOLE IS READY");

        char cmd[64];
        int cmdlen = 0;

        char buffer[16];
        printf("> ");
        while(true) {
            if(fgets(buffer, sizeof(buffer), stdin) != NULL) {
                int l = strlen(buffer);
                bool enter = buffer[l - 1] == '\n';
                if(enter) {
                    l -= 1;
                }
                int cmd_remain = sizeof(cmd) - cmdlen - 1;
                if(l >= cmd_remain) {
                    l = cmd_remain;
                }
                char *fragment = cmd + cmdlen;
                memcpy(fragment, buffer, l);
                cmdlen += l;
                cmd[cmdlen] = 0;
                if(enter) {
                    handle_command(cmd);
                    cmdlen = 0;
                }
            }
            delay_ms(100);
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

console_command_base_t::console_command_base_t()
{
    next = console_command_list();
    console_command_list() = this;
}

//////////////////////////////////////////////////////////////////////

void console_init()
{
    xTaskCreatePinnedToCore(console_task, "console", 3072, NULL, 1, NULL, 0);
}

//////////////////////////////////////////////////////////////////////

template <typename S> struct enum_name
{
    char const *name;
    S value;
};

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> bool find_enum(char const *what, enum_name<T> const (&values)[N], T &result)
{
    for(auto const &f : values) {
        if(strcmp(f.name, what) == 0) {
            result = f.value;
            return true;
        }
    }
    printf("Bad value %s\n", what);
    return false;
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"help", "show commands">
{
    void on_command(int argc, char **argv) override
    {
        for(console_command_base_t *c = console_command_list(); c != nullptr; c = c->next) {
            printf("%-20s %s\n", c->name(), c->help());
        }
    }
} cmd_help;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"mem", "show stack and heap usage">
{
    void on_command(int argc, char **argv) override
    {
        char buffer[512];
        vTaskList(buffer);
        printf("%s\n", buffer);
        auto x = esp_get_minimum_free_heap_size();
        printf("HEAP: %lu\n", x);
    }
} cmd_mem;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"factory", "erase NVS partition">
{
    void on_command(int argc, char **argv) override
    {
        ESP_LOG_ERR(nvs_flash_erase());
        ESP_LOG_ERR(nvs_flash_init());
    }
} cmd_factory;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"reset", "reset the ESP32">
{
    void on_command(int argc, char **argv) override
    {
        esp_restart();
    }
} cmd_reset;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"brightness", "set brightness: auto|manual low|medium|high">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<auto_brightness_t> b[] = {
            "auto", auto_brightness_t::on,      //
            "manual", auto_brightness_t::off    //
        };
        enum_name<brightness_mode_t> c[] = {
            "low",    brightness_mode_t::low,       //
            "medium", brightness_mode_t::medium,    //
            "high",   brightness_mode_t::high       //
        };
        auto_brightness_t brightness = auto_brightness_t::on;
        brightness_mode_t mode = brightness_mode_t::medium;
        if(argc == 3 && find_enum(argv[1], b, brightness) && find_enum(argv[2], c, mode)) {
            settings.auto_brightness = brightness;
            settings.brightness_mode = mode;
        } else {
            printf("Usage: brightness [auto|manual] [low|medium|high]\n");
        }
    }
} cmd_brightness;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"seconds", "set seconds: long|medium|short|fixed|single">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<seconds_mode_t> m[] = {
            "long",   seconds_mode_t::tail_long,      //
            "medium", seconds_mode_t::tail_medium,    //
            "short",  seconds_mode_t::tail_short,     //
            "fixed",  seconds_mode_t::fixed,          //
            "single", seconds_mode_t::single,
        };
        if(!(argc == 2 && find_enum(argv[1], m, settings.seconds_mode))) {
            printf("Usage: seconds [long|medium|short|fixed|single]\n");
        }
    }
} cmd_seconds;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"colon", "set colon: off|on|dim|pulse">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<colon_mode_t> m[] = {
            "off",   colon_mode_t::off,      //
            "on",    colon_mode_t::on,       //
            "dim",   colon_mode_t::dim,      //
            "pulse", colon_mode_t::pulse,    //
        };
        if(!(argc == 2 && find_enum(argv[1], m, settings.colon_mode))) {
            printf("Usage: colon [off|on|dim|pulse]\n");
        }
    }
} cmd_colon;

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"fade", "set fade: off|low|medium|high">
{
    void on_command(int argc, char **argv) override
    {
        enum_name<clock_fade_mode_t> names[] = {
            "off",    clock_fade_mode_t::off,       //
            "low",    clock_fade_mode_t::low,       //
            "medium", clock_fade_mode_t::medium,    //
            "high",   clock_fade_mode_t::high,      //
        };
        if(!(argc == 2 && find_enum(argv[1], names, settings.clock_fade_mode))) {
            printf("Usage: fade [off|low|medium|high]\n");
        }
    }

} cmd_fade;
