#pragma once

//////////////////////////////////////////////////////////////////////

struct console_command_t
{
    console_command_t();

    virtual char const *name() const = 0;
    virtual char const *help() const = 0;
    virtual void on_command(int argc, char **argv) = 0;

    console_command_t *next;
};

//////////////////////////////////////////////////////////////////////

void console_init();
