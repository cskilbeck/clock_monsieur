#pragma once

#include <algorithm>
#include <cstddef>
#include <new>

#include "graphics.h"

//////////////////////////////////////////////////////////////////////

struct state_handler_t
{
    virtual ~state_handler_t() = default;
    virtual void init();
    virtual void on_start();
    virtual void on_update();
    virtual void on_stop();
};

//////////////////////////////////////////////////////////////////////

void state_init();
void state_enqueue(void (*ctor)(void *));
void state_update();

template <typename T>
void state_set()
{
    state_enqueue([](void *buf) { new (buf) T{}; });
}

//////////////////////////////////////////////////////////////////////

struct boot_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;
    void on_stop() override;
    char *boot_msg;
    bool factory_reset;
};

//////////////////////////////////////////////////////////////////////

struct wifi_check_state_t : state_handler_t
{
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

struct factory_reset_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;

    float held_time;
    float released_time;
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

struct menu_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

struct timer_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;

    double end_time;
    bool done;
};

//////////////////////////////////////////////////////////////////////

struct alarm_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;

    double end_time;
    bool snooze_msg;
    double snooze_msg_time;
};

//////////////////////////////////////////////////////////////////////

struct timezone_select_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

struct lux_state_t : state_handler_t
{
    void on_update() override;
};

//////////////////////////////////////////////////////////////////////

struct led_edit_state_t : state_handler_t
{
    void on_start() override;
    void on_update() override;

    uint8_t led_number = 0;
};

//////////////////////////////////////////////////////////////////////

template <typename... Ts> inline constexpr size_t max_sizeof_v = std::max({ sizeof(Ts)... });
