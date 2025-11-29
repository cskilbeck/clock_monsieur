//////////////////////////////////////////////////////////////////////

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "util.h"
#include "main.h"
#include "graphics.h"
#include "graphics.h"
#include "button.h"
#include "menu.h"
#include "state.h"
#include "settings.h"

LOG_CONTEXT("menu");

//////////////////////////////////////////////////////////////////////

struct item_t
{
    using select_function = std::function<void()>;

    char const *name;
    item_t *parent = nullptr;
    item_t *prev = nullptr;
    item_t *next = nullptr;
    item_t *children = nullptr;
    select_function on_select;

    static int name_x;
    static item_t *current_item;

    item_t(char const *name, item_t *_parent, select_function const &select_fn = select_function{})
        : name(name), parent(_parent), prev(nullptr), next(nullptr), children(nullptr), on_select(select_fn)
    {
        if(_parent) {
            parent = _parent;
            item_t *child = _parent->children;
            if(child == nullptr) {
                _parent->children = this;
            } else {
                while(child->next != nullptr) {
                    child = child->next;
                }
                child->next = this;
                prev = child;
            }
        }
    }

    void go(item_t *where)
    {
        name_x = 0;
        current_item = where;
    }

    virtual void on_update()
    {
        int width = font_5x6_font.measure_string(name);
        int min_name_x = (screen_width - width) * 2;
        gfx.clear();
        font_5x6_font.draw_long_string(gfx, name, name_x / 2, 0, 1.0f, 0.5f);
        gfx.display();

        if(button_up.pressed && prev != nullptr) {
            // UP previous menu item
            go(prev);

        } else if(button_down.pressed && next != nullptr) {
            // DOWN next menu item
            go(next);

        } else if(button_left.held && name_x < 0) {
            // LEFT scroll left
            name_x += 1;

        } else if(button_right.held && name_x > min_name_x) {
            // RIGHT scroll right
            name_x -= 1;

        } else if(button_left.pressed && name_x == 0) {
            if(parent != nullptr) {
                // LEFT PRESS (if at 0) go up to parent
                go(parent);
            } else {
                // or back to clock if no parent
                state_set(clock_state);
            }

        } else if(button_select.pressed) {
            // SELECT go into child menu or call on_select()
            if(on_select) {
                LOG_INFO("%s selected!", name);
                on_select();
            }
            if(children != nullptr) {
                go(children);
            } else {
                go(parent);
            }
        }
    }
};

int item_t::name_x = 0;

item_t *item_t::current_item{ nullptr };

item_t root_menu = item_t("Save", nullptr, []() {
    settings.save();
    state_set(clock_state);
});

item_t settings_menu = item_t("Settings", &root_menu);
item_t timezone_menu = item_t("Timezone", &root_menu);

// Now we need to get the timezones in there which is a hassle

item_t mode_menu = item_t("Mode", &settings_menu);
item_t brightness_menu = item_t("Brightness", &settings_menu);
item_t seconds_menu = item_t("Seconds", &settings_menu);
item_t colon_menu = item_t("Colon", &settings_menu);
item_t fade_menu = item_t("Fade", &settings_menu);

item_t mode_12_menu = item_t("12 hr", &mode_menu, []() { settings.clock_mode = clock_mode_t::clock_12_hour; });
item_t mode_24_menu = item_t("24 hr", &mode_menu, []() { settings.clock_mode = clock_mode_t::clock_24_hour; });

item_t brightness_auto_menu = item_t("Auto", &brightness_menu);
item_t brightness_auto_on_menu = item_t("On", &brightness_auto_menu, []() { settings.auto_brightness = auto_brightness_t::on; });
item_t brightness_auto_off_menu = item_t("Off", &brightness_auto_menu, []() { settings.auto_brightness = auto_brightness_t::off; });

item_t brightness_low_menu = item_t("Low", &brightness_menu, []() { settings.brightness = 96; });
item_t brightness_medium_menu = item_t("Medium", &brightness_menu, []() { settings.brightness = 150; });
item_t brightness_high_menu = item_t("High", &brightness_menu, []() { settings.brightness = 255; });

item_t seconds_single_menu = item_t("Single", &seconds_menu, []() { settings.seconds_mode = seconds_mode_t::single; });
item_t seconds_fixed_menu = item_t("Fixed", &seconds_menu, []() { settings.seconds_mode = seconds_mode_t::fixed; });
item_t seconds_long_menu = item_t("Long", &seconds_menu, []() { settings.seconds_mode = seconds_mode_t::tail_long; });
item_t seconds_medium_menu = item_t("Medium", &seconds_menu, []() { settings.seconds_mode = seconds_mode_t::tail_medium; });
item_t seconds_short_menu = item_t("Short", &seconds_menu, []() { settings.seconds_mode = seconds_mode_t::tail_short; });

item_t colon_off_menu = item_t("Off", &colon_menu, []() { settings.colon_mode = colon_mode_t::off; });
item_t colon_on_menu = item_t("On", &colon_menu, []() { settings.colon_mode = colon_mode_t::on; });
item_t colon_dim_menu = item_t("Dim", &colon_menu, []() { settings.colon_mode = colon_mode_t::dim; });
item_t colon_pulse_menu = item_t("Pulse", &colon_menu, []() { settings.colon_mode = colon_mode_t::pulse; });

item_t fade_low_menu = item_t("Low", &fade_menu, []() { settings.clock_fade_mode = clock_fade_mode_t::low; });
item_t fade_medium_menu = item_t("Medium", &fade_menu, []() { settings.clock_fade_mode = clock_fade_mode_t::medium; });
item_t fade_high_menu = item_t("High", &fade_menu, []() { settings.clock_fade_mode = clock_fade_mode_t::high; });
item_t fade_off_menu = item_t("Off", &fade_menu, []() { settings.clock_fade_mode = clock_fade_mode_t::off; });

//////////////////////////////////////////////////////////////////////

void menu_init()
{
    item_t::current_item = &settings_menu;
}

//////////////////////////////////////////////////////////////////////

void menu_update()
{
    item_t::current_item->on_update();
}
