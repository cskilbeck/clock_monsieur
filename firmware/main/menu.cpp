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

namespace
{
    struct item_t;

    int name_x = 0;
    item_t *current_item{ nullptr };

    void go(item_t *where);

    struct item_t
    {
        using select_function = std::function<void()>;

        char const *name;
        item_t *parent = nullptr;
        item_t *prev = nullptr;
        item_t *next = nullptr;
        item_t *children = nullptr;
        select_function on_select;

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

        virtual void on_update()
        {
            int width = font_5x7_narrow_font.measure_string(name);
            int min_name_x = (screen_width - width) * 2;
            gfx.clear();
            font_5x7_narrow_font.draw_long_string(gfx, name, name_x / 2, 0, 1.0f, 0.5f);
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
                item_t *old_item = current_item;
                if(on_select) {
                    LOG_INFO("%s selected!", name);
                    on_select();
                    if(old_item != current_item) {
                        LOG_INFO("went to %s!", current_item->name);
                    }
                    if(parent != nullptr && old_item == current_item) {
                        go(parent);
                        return;
                    }
                }
                // only go into children if on_select didn't call go()
                if(children != nullptr && old_item == current_item) {
                    go(children);
                }
            }
        }
    };

    void go(item_t *where)
    {
        LOG_INFO("GO to %s", where->name);
        name_x = 0;
        current_item = where;
    }

    item_t root_menu{ "Save", nullptr, [] {
                         settings.save();
                         state_set(clock_state);
                     } };

    ///// Settings

    item_t settings_menu{ "Settings", &root_menu };

    /////     Mode

    extern item_t mode_menu;

    item_t mode_12_menu{ "12 hr", &mode_menu, [] { settings.clock_mode = clock_mode_t::clock_12_hour; } };
    item_t mode_24_menu{ "24 hr", &mode_menu, [] { settings.clock_mode = clock_mode_t::clock_24_hour; } };

    item_t mode_menu{ "Mode", &settings_menu, [] {
                         if(settings.clock_mode == clock_mode_t::clock_24_hour) {
                             go(&mode_24_menu);
                         } else {
                             go(&mode_12_menu);
                         }
                     } };

    /////     Brightness

    item_t brightness_menu{ "Brightness", &settings_menu };

    item_t brightness_auto_menu{ "Auto", &brightness_menu };
    item_t brightness_auto_on_menu{ "On", &brightness_auto_menu, [] { settings.auto_brightness = auto_brightness_t::on; } };
    item_t brightness_auto_off_menu{ "Off", &brightness_auto_menu, [] { settings.auto_brightness = auto_brightness_t::off; } };

    item_t brightness_low_menu{ "Low", &brightness_menu, [] { settings.brightness = 96; } };
    item_t brightness_medium_menu{ "Medium", &brightness_menu, [] { settings.brightness = 150; } };
    item_t brightness_high_menu{ "High", &brightness_menu, [] { settings.brightness = 255; } };

    /////     Seconds

    extern item_t seconds_menu;

    item_t seconds_single_menu{ "Single", &seconds_menu, [] { settings.seconds_mode = seconds_mode_t::single; } };
    item_t seconds_fixed_menu{ "Fixed", &seconds_menu, [] { settings.seconds_mode = seconds_mode_t::fixed; } };
    item_t seconds_long_menu{ "Long", &seconds_menu, [] { settings.seconds_mode = seconds_mode_t::tail_long; } };
    item_t seconds_medium_menu{ "Medium", &seconds_menu, [] { settings.seconds_mode = seconds_mode_t::tail_medium; } };
    item_t seconds_short_menu{ "Short", &seconds_menu, [] { settings.seconds_mode = seconds_mode_t::tail_short; } };

    item_t seconds_menu{ "Seconds", &settings_menu, [] {
                            switch(settings.seconds_mode) {
                            case seconds_mode_t::tail_long:
                                go(&seconds_long_menu);
                                break;
                            case seconds_mode_t::tail_medium:
                                go(&seconds_medium_menu);
                                break;
                            case seconds_mode_t::tail_short:
                                go(&seconds_short_menu);
                                break;
                            case seconds_mode_t::fixed:
                                go(&seconds_fixed_menu);
                                break;
                            case seconds_mode_t::single:
                                go(&seconds_single_menu);
                                break;
                            }
                        } };

    /////     Colon

    extern item_t colon_menu;

    item_t colon_off_menu{ "Off", &colon_menu, [] { settings.colon_mode = colon_mode_t::off; } };
    item_t colon_on_menu{ "On", &colon_menu, [] { settings.colon_mode = colon_mode_t::on; } };
    item_t colon_dim_menu{ "Dim", &colon_menu, [] { settings.colon_mode = colon_mode_t::dim; } };
    item_t colon_pulse_menu{ "Pulse", &colon_menu, [] { settings.colon_mode = colon_mode_t::pulse; } };

    item_t colon_menu{ "Colon", &settings_menu, [] {
                          switch(settings.colon_mode) {
                          case colon_mode_t::off:
                              go(&colon_off_menu);
                              break;
                          case colon_mode_t::on:
                              go(&colon_on_menu);
                              break;
                          case colon_mode_t::dim:
                              go(&colon_dim_menu);
                              break;
                          case colon_mode_t::pulse:
                              go(&colon_pulse_menu);
                              break;
                          }
                      } };

    /////     Font

    extern item_t font_menu;

    item_t font_normal_menu{ "Normal", &font_menu, [] { settings.clock_font = clock_font_t::normal; } };
    item_t font_modern_menu{ "Modern", &font_menu, [] { settings.clock_font = clock_font_t::modern; } };

    item_t font_menu{ "Font", &settings_menu, [] {
                         if(settings.clock_font == clock_font_t::normal) {
                             go(&font_normal_menu);
                         } else {
                             go(&font_modern_menu);
                         }
                     } };

    /////     Fade

    extern item_t fade_menu;

    item_t fade_low_menu{ "Low", &fade_menu, [] { settings.clock_fade_mode = clock_fade_mode_t::low; } };
    item_t fade_medium_menu{ "Medium", &fade_menu, [] { settings.clock_fade_mode = clock_fade_mode_t::medium; } };
    item_t fade_high_menu{ "High", &fade_menu, [] { settings.clock_fade_mode = clock_fade_mode_t::high; } };
    item_t fade_off_menu{ "Off", &fade_menu, [] { settings.clock_fade_mode = clock_fade_mode_t::off; } };

    item_t fade_menu{ "Fade", &settings_menu, [] {
                         switch(settings.clock_fade_mode) {
                         case clock_fade_mode_t::off:
                             go(&fade_off_menu);
                             break;
                         case clock_fade_mode_t::low:
                             go(&fade_low_menu);
                             break;
                         case clock_fade_mode_t::medium:
                             go(&fade_medium_menu);
                             break;
                         case clock_fade_mode_t::high:
                             go(&fade_high_menu);
                             break;
                         }
                     } };

    ///// Timezone

    item_t timezone_menu{ "Timezone", &root_menu };
    item_t timezone_auto_menu{ "Auto", &timezone_menu, [] { LOG_INFO("AUTO!"); } };
    item_t timezone_select_menu{ "Select", &timezone_menu, [] { state_set(timezone_select_state); } };

    ///// System

    item_t system_menu{ "System", &root_menu };
    item_t factory_reset_menu{ "Factory reset", &system_menu };
    item_t factory_reset_are_you_sure_menu{ "You sure?", &factory_reset_menu };
    item_t factory_reset_no{ "No", &factory_reset_are_you_sure_menu, [] { go(&settings_menu); } };
    item_t factory_reset_yes{ "Yes", &factory_reset_are_you_sure_menu, [] { state_set(factory_reset_state); } };

}    // namespace

//////////////////////////////////////////////////////////////////////

void menu_init()
{
    current_item = &settings_menu;
}

//////////////////////////////////////////////////////////////////////

void menu_update()
{
    current_item->on_update();
}
