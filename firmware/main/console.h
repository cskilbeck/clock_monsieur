//////////////////////////////////////////////////////////////////////

#pragma once

#include <cstring>

//////////////////////////////////////////////////////////////////////

struct console_command_base_t
{
    console_command_base_t();

    virtual char const *name() const = 0;
    virtual char const *help() const = 0;
    virtual char const *args() const = 0;
    virtual void on_command(int argc, char **argv){};

    console_command_base_t *next;

    static console_command_base_t *command_list;
};

//////////////////////////////////////////////////////////////////////

template <std::size_t N> struct string_literal
{
    char value[N];
    constexpr string_literal(const char (&str)[N])
    {
        for(int i = 0; i < N; ++i) {
            value[i] = str[i];
        }
    }
    constexpr operator char const *() const
    {
        return value;
    }
};

template <string_literal NAME, string_literal HELP, string_literal ARGS> struct console_command_t : console_command_base_t
{
    console_command_t() : console_command_base_t()
    {
    }
    char const *name() const override
    {
        return NAME;
    }
    char const *help() const override
    {
        return HELP;
    }
    char const *args() const override
    {
        return ARGS;
    }

    void usage()
    {
        printf("%s\nUsage: %s %s\n", help(), name(), args());
    }
};

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

void console_init();
