//////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cstring>
#include <ctype.h>

#include <freertos/FreeRTOS.h>
#include "nvs_flash.h"

#include "util.h"
#include "console.h"

LOG_CONTEXT("console");

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

    console_command_t *console_command_list{ nullptr };

    //////////////////////////////////////////////////////////////////////

    void handle_command(char *cmd)
    {
        char *argv[16];
        int argc = parse_args(cmd, argv);
        printf(">%s\n", cmd);
        if(argc != 0) {
            for(console_command_t *c = console_command_list; c != nullptr; c = c->next) {
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

console_command_t::console_command_t()
{
    next = console_command_list;
    console_command_list = this;
}

//////////////////////////////////////////////////////////////////////

struct help_command : console_command_t
{
    help_command() : console_command_t()
    {
    }
    char const *name() const override
    {
        return "help";
    }
    char const *help() const override
    {
        return "show commands";
    }
    void on_command(int argc, char **argv) override
    {
        for(console_command_t *c = console_command_list; c != nullptr; c = c->next) {
            printf("%-20s %s\n", c->name(), c->help());
        }
    }
} cmd_help;

//////////////////////////////////////////////////////////////////////

struct mem_command : console_command_t
{
    mem_command() : console_command_t()
    {
    }
    char const *name() const override
    {
        return "mem";
    }
    char const *help() const override
    {
        return "show stack and heap usage";
    }
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

struct factory_command : console_command_t
{
    factory_command() : console_command_t()
    {
    }
    char const *name() const override
    {
        return "factory";
    }
    char const *help() const override
    {
        return "erase NVS partition";
    }
    void on_command(int argc, char **argv) override
    {
        ESP_LOG_ERR(nvs_flash_erase());
        ESP_LOG_ERR(nvs_flash_init());
    }
} cmd_factory;

//////////////////////////////////////////////////////////////////////

struct reset_command : console_command_t
{
    reset_command() : console_command_t()
    {
    }
    char const *name() const override
    {
        return "reset";
    }
    char const *help() const override
    {
        return "reset the ESP32";
    }
    void on_command(int argc, char **argv) override
    {
        esp_restart();
    }
} cmd_reset;

//////////////////////////////////////////////////////////////////////

void console_init()
{
    xTaskCreatePinnedToCore(console_task, "console", 3072, NULL, 1, NULL, 0);
}
