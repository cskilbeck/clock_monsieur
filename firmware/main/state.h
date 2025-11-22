#pragma once

//////////////////////////////////////////////////////////////////////

struct state_handler_t
{
    virtual void init();
    virtual void on_start();
    virtual void on_update();
    virtual void on_stop();
};

//////////////////////////////////////////////////////////////////////

void state_init();
void state_set(state_handler_t &new_state);
void state_update();

//////////////////////////////////////////////////////////////////////

struct boot_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;
    void on_stop() override;
    char *boot_msg;
    int msg_width;
};

//////////////////////////////////////////////////////////////////////

struct clock_state_t : state_handler_t
{
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

struct ota_state_t : state_handler_t
{
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

extern boot_state_t boot_state;
extern clock_state_t clock_state;
extern ota_state_t ota_state;
