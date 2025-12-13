//////////////////////////////////////////////////////////////////////

#pragma once

#include <cstring>
#include "settings.h"

//////////////////////////////////////////////////////////////////////

struct console_command_base_t
{
    console_command_base_t()
    {
        next = command_list;
        command_list = this;
    }

    virtual char const *name() const = 0;
    virtual char const *help() const = 0;
    virtual char const *args() const = 0;
    virtual void on_command(int argc, char **argv)
    {
    }

    void usage()
    {
        printf("%s\nUsage: %s %s\n", help(), name(), args());
    }
    console_command_base_t *next;

    static console_command_base_t *command_list;
};

//////////////////////////////////////////////////////////////////////

template <string_literal NAME, string_literal HELP, string_literal ARGS> struct console_command_t : console_command_base_t
{
    console_command_t() : console_command_base_t()
    {
    }

    virtual char const *name() const
    {
        return NAME;
    }

    virtual char const *help() const
    {
        return HELP;
    }

    virtual char const *args() const
    {
        return ARGS;
    }

    virtual void on_command(int argc, char **argv) = 0;
};

//////////////////////////////////////////////////////////////////////

template <typename T> bool find_enum(char const *what, T &result)
{
    for(int i = 0; i < count_enum_values<T>(); ++i) {
        if(strcasecmp(what, enum_to_string((T)i)) == 0) {
            result = (T)i;
            return true;
        }
    }
    printf("Bad value %s\n", what);
    return false;
}

//////////////////////////////////////////////////////////////////////

void console_init();
