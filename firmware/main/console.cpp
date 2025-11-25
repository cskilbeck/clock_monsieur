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

console_command_base_t *console_command_base_t::command_list{};

namespace
{
    //////////////////////////////////////////////////////////////////////
    // https://stackoverflow.com/questions/9659697/parse-string-into-array-based-on-spaces-or-double-quotes-strings/9660047#9660047

    template <size_t MAX_ARGS> int parse_args(char *str, char *(&argv)[MAX_ARGS])
    {
        enum state_t
        {
            space,
            quotes,
            word,
        } state = space;

        char *arg = nullptr;
        char quote{};
        int argc = 0;

        auto is_quote = [](int c) { return c == '"' || c == '\''; };

        auto push = [&arg, &argc, &argv, &state](char *p) {
            *p = 0;
            argv[argc++] = arg;
            state = space;
        };

        for(char *p = str; *p != 0; ++p) {
            int c = *p;
            switch(state) {
            case space:
                if(is_quote(c)) {
                    state = quotes;
                    quote = c;
                    arg = p + 1;
                } else if(!isspace(c)) {
                    state = word;
                    arg = p;
                }
                break;

            case quotes:
                if(c == quote) {
                    push(p);
                }
                break;

            case word:
                if(isspace(c)) {
                    push(p);
                }
                break;
            }
            if(argc == MAX_ARGS) {
                return argc;
            }
        }
        if(state != space && arg != nullptr) {
            argv[argc++] = arg;
        }
        return argc;
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
                    char *argv[16];
                    int argc = parse_args(cmd, argv);
                    printf(">");
                    for(int i = 0; i < argc; ++i) {
                        printf(" %s", argv[i]);
                    }
                    printf("\n");
                    if(argc != 0) {
                        bool found{ false };
                        for(console_command_base_t *c = console_command_base_t::command_list; c != nullptr; c = c->next) {
                            if(strcmp(c->name(), argv[0]) == 0) {
                                c->on_command(argc, argv);
                                found = true;
                                break;
                            }
                        }
                        if(!found) {
                            printf("Unknown command: %s\n", argv[0]);
                        }
                    }
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
    next = command_list;
    command_list = this;
}

//////////////////////////////////////////////////////////////////////

void console_init()
{
    xTaskCreatePinnedToCore(console_task, "console", 3072, NULL, 1, NULL, 0);
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"help", "show commands", "">
{
    void on_command(int argc, char **argv) override
    {
        for(console_command_base_t *c = command_list; c != nullptr; c = c->next) {
            printf("%-20s %s\n", c->name(), c->help());
        }
    }
} cmd_help;
