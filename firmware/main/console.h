//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

struct console_command_base_t
{
    console_command_base_t();

    virtual char const *name() const = 0;
    virtual char const *help() const = 0;
    virtual void on_command(int argc, char **argv){};

    console_command_base_t *next;
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

template <string_literal NAME, string_literal HELP> struct console_command_t : console_command_base_t
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
};

console_command_base_t *&console_command_list();

//////////////////////////////////////////////////////////////////////

void console_init();
