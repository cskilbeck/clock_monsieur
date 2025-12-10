//////////////////////////////////////////////////////////////////////

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "util.h"
#include "main.h"
#include "clock_time.h"
#include "graphics.h"
#include "graphics.h"
#include "button.h"
#include "menu.h"
#include "wifi.h"
#include "state.h"
#include "settings.h"
#include "version.h"

LOG_CONTEXT("menu");

//////////////////////////////////////////////////////////////////////

namespace
{
    struct item_t;

    timeval last_menu_activity_timestamp{ 0 };

    //////////////////////////////////////////////////////////////////////

    void menu_exit();

    //////////////////////////////////////////////////////////////////////

    struct item_t
    {
        using select_function = std::function<void()>;

        // current menu item
        static item_t *current_item;

        // previous menu item (or null)
        static item_t *previous_item;

        // transition from previous to current item in pixels
        static int transition_x;
        static int transition_y;

        static int transition_xvel;
        static int transition_yvel;

        // manual panning x
        static int name_x;

        static void go(item_t *where, int xvel = -2, int yvel = 0);

        char const *name;
        item_t *parent = nullptr;
        item_t *prev = nullptr;
        item_t *next = nullptr;
        item_t *children = nullptr;
        select_function on_select;

        item_t *old_item = nullptr;

        virtual char const *text() const
        {
            return name;
        }

        //////////////////////////////////////////////////////////////////////

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

        //////////////////////////////////////////////////////////////////////

        virtual void on_update()
        {
            int constexpr pan_speed = 2;
            int constexpr transition_speed = 2;

            gfx.clear();

            int offset_x = 0;
            int offset_y = 0;

            int width = font_5x7_narrow_modern_font.measure_string(text());
            int min_name_x = (screen_width - width) * pan_speed;

            int x_gap = 3;
            int y_gap = 1;

            if(previous_item != nullptr) {
                transition_x += transition_xvel;
                transition_y += transition_yvel;
                int x = transition_x / transition_speed;
                int y = transition_y / transition_speed;
                int prev_width = font_5x7_narrow_modern_font.draw_string(gfx, previous_item->text(), x, y, 1.0f);
                offset_x = (transition_xvel == 0) ? 0 : (transition_xvel < 0) ? x + prev_width + x_gap : x - (width + x_gap);
                offset_y = (transition_yvel == 0) ? 0 : (transition_yvel < 0) ? y + screen_height + y_gap : y - (screen_height + y_gap);
                if((transition_xvel > 0 && offset_x >= 0) ||    //
                   (transition_xvel < 0 && offset_x <= 0) ||    //
                   (transition_yvel > 0 && offset_y >= 0) ||    //
                   (transition_yvel < 0 && offset_y <= 0)) {
                    previous_item = nullptr;
                } else {
                    font_5x7_narrow_modern_font.draw_string(gfx, text(), offset_x, offset_y, 1.0f);
                }
            }

            if(previous_item == nullptr) {
                float scrollbar_color = 0.5f;
                font_5x7_narrow_modern_font.draw_long_string(gfx, text(), name_x / pan_speed, 0, 1.0f, scrollbar_color);

                if(button_up.pressed) {
                    if(prev != nullptr) {
                        // UP previous menu item
                        go(prev, 0, 1);
                    } else {
                        item_t *n = next;
                        while(n != nullptr && n->next != nullptr) {
                            n = n->next;
                        }
                        if(n != nullptr) {
                            go(n, 0, 1);
                        }
                    }

                } else if(button_down.pressed) {
                    // DOWN next menu item
                    if(next != nullptr) {
                        go(next, 0, -1);
                    } else {
                        go(parent->children, 0, -1);
                    }

                } else if(button_left.held && name_x < 0) {
                    // LEFT scroll left
                    name_x += 1;

                } else if(button_right.held && name_x > min_name_x) {
                    // RIGHT scroll right
                    name_x -= 1;

                } else if(button_left.pressed && name_x == 0) {
                    // LEFT PRESS (if at 0) go up to parent
                    if(parent != nullptr) {
                        if(parent->parent != nullptr) {
                            go(parent, 2, 0);
                        } else {
                            // or back to clock if gone back to the root node (which has no name)
                            menu_exit();
                        }
                    }

                } else if(button_select.pressed) {
                    // SELECT go into child menu or call on_select()
                    item_t *old_item = current_item;
                    if(on_select) {
                        LOG_DEBUG("%s selected", text());
                        on_select();
                        if(old_item != current_item) {
                            LOG_DEBUG("went to %s!", current_item->text());
                        }
                        // Only go to parent if this entry is not a single child
                        if(parent != nullptr && old_item == current_item && next != nullptr && prev != nullptr) {
                            go(parent, 2, 0);
                            return;
                        }
                    }
                    // only go into children if on_select didn't call go()
                    if(children != nullptr && old_item == current_item) {
                        go(children, -2, 0);
                    }
                }
            }
            gfx.display();
        }
    };

    //////////////////////////////////////////////////////////////////////

    void item_t::go(item_t *where, int xvel, int yvel)
    {
        if(where == item_t::current_item) {
            return;
        }
        LOG_DEBUG("GO to %s", where->name);

        transition_x = item_t::name_x;
        transition_y = 0;
        transition_xvel = xvel;
        transition_yvel = yvel;
        previous_item = item_t::current_item;
        name_x = 0;
        current_item = where;
        last_menu_activity_timestamp = utc_wall_time;
    }

    //////////////////////////////////////////////////////////////////////

#if defined(DEBUG)
    void show_menu(item_t const *m, int indent = 0)
    {
        LOG_INFO("%*s%s", indent, "", m->text());
        for(item_t const *c = m->children; c != nullptr; c = c->next) {
            show_menu(c, indent + 2);
        }
    }
#endif

    //////////////////////////////////////////////////////////////////////

    void menu_exit()
    {
        settings.save();
        state_set(clock_state);
    }


    //////////////////////////////////////////////////////////////////////

    item_t *item_t::previous_item = nullptr;
    int item_t::transition_x;
    int item_t::transition_y;
    int item_t::transition_xvel;
    int item_t::transition_yvel;

    item_t *item_t::current_item{ nullptr };
    int item_t::name_x{ 0 };

    item_t root_menu{ "root", nullptr };

    //////////////////////////////////////////////////////////////////////

    template <typename T> void cycle_enum(T &x)
    {
        int y = static_cast<int>(x);
        if(y == static_cast<int>(T::max)) {
            y = 0;
        } else {
            y += 1;
        }
        x = static_cast<T>(y);
    }

    //////////////////////////////////////////////////////////////////////

#define ENAME(type, value) \
    case type::value:      \
        return #value

    ///// Settings

    item_t settings_menu{ "Settings", &root_menu };

    /////     Mode

    item_t mode_menu{ "Mode", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.clock_mode) {
            case clock_mode_t::clock_12_hour:
                return "12 Hour";
            case clock_mode_t::clock_24_hour:
                return "24 Hour";
            }
            return "?";
        }
    } clock_mode_setting_menu{ "?", &mode_menu, [] { cycle_enum(settings.clock_mode); } };

    /////     Brightness

    item_t brightness_menu{ "Brightness", &settings_menu };

    struct brightness_auto_menu_t : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.auto_brightness) {
                ENAME(auto_brightness_t, Fixed);
                ENAME(auto_brightness_t, Auto);
            }
            return "?";
        }
    } brightness_auto_menu{ "?", &brightness_menu, [] { cycle_enum(settings.auto_brightness); } };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.brightness) {
            case 32:
                return "Low";
            default:
                return "Medium";
            case 128:
                return "High";
            case 255:
                return "Max";
            }
        }
    } brightness_level_menu{ "?", &brightness_menu, [] {
                                switch(settings.brightness) {
                                case 32:
                                    settings.brightness = 64;
                                    break;
                                default:
                                    settings.brightness = 128;
                                    break;
                                case 128:
                                    settings.brightness = 255;
                                    break;
                                case 255:
                                    settings.brightness = 32;
                                    break;
                                }
                            } };

    /////     Seconds

    item_t seconds_menu{ "Seconds", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.seconds_mode) {
                ENAME(seconds_mode_t, Long);
                ENAME(seconds_mode_t, Medium);
                ENAME(seconds_mode_t, Short);
                ENAME(seconds_mode_t, Fixed);
                ENAME(seconds_mode_t, Single);
            }
            return "?";
        }
    } seconds_type_menu{ "", &seconds_menu, [] { cycle_enum(settings.seconds_mode); } };

    /////     Colon

    item_t colon_menu{ "Colon", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.colon_mode) {
                ENAME(colon_mode_t, Off);
                ENAME(colon_mode_t, On);
                ENAME(colon_mode_t, Dim);
                ENAME(colon_mode_t, Pulse);
            }
            return "?";
        }
    } colon_setting_menu{ "?", &colon_menu, [] { cycle_enum(settings.colon_mode); } };

    /////     Font

    item_t font_menu{ "Font", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.clock_font) {
                ENAME(clock_font_t, Normal);
                ENAME(clock_font_t, Square);
            }
            return "?";
        }
    } font_setting_menu{ "?", &font_menu, [] { cycle_enum(settings.clock_font); } };

    /////     Fade

    item_t fade_menu{ "Fade", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.clock_fade_mode) {
                ENAME(clock_fade_mode_t, Off);
                ENAME(clock_fade_mode_t, Low);
                ENAME(clock_fade_mode_t, Medium);
                ENAME(clock_fade_mode_t, High);
            }
            return "?";
        }
    } fade_setting_menu{ "?", &fade_menu, [] { cycle_enum(settings.clock_fade_mode); } };

    /////     Ticks

    item_t ticks_menu{ "Ticks", &settings_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            switch(settings.ticks) {
                ENAME(tick_mode_t, Off);
                ENAME(tick_mode_t, On);
                ENAME(tick_mode_t, Dim);
                ENAME(tick_mode_t, Track);
            }
            return "?";
        }
    } ticks_setting_menu{ "?", &ticks_menu, [] { cycle_enum(settings.ticks); } };

    ///// Timezone

    item_t timezone_menu{ "Timezone", &root_menu };
    item_t timezone_auto_menu{ "Auto", &timezone_menu, [] {
                                  LOG_INFO("Auto timezone");
                                  settings.timezone_mode = timezone_mode_t::Auto;
                                  xEventGroupSetBits(system_events, SYS_EVENT_NEED_LOCATION);
                                  menu_exit();
                              } };

    struct timezone_select_menu_t : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            if(settings.timezone_mode == timezone_mode_t::Select && settings.location.name[0] != 0) {
                return settings.location.name;
            }
            return "Select";
        }
    };

    timezone_select_menu_t timezone_select_menu{ "ZONE", &timezone_menu, [] { state_set(timezone_select_state); } };

    ///// System

    item_t system_menu{ "System", &root_menu };
    item_t version_menu{ "Version", &system_menu };
    item_t show_version_menu{ "V" VERSION_STR, &version_menu };

    item_t ip_address_menu{ "IP Address", &system_menu };
    item_t mac_address_menu{ "MAC Address", &system_menu };

    struct ip_addr_item_t : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_ip_address();
        }
    } show_ip_menu{ "192.168.0.1", &ip_address_menu };

    struct mac_addr_item_t : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_mac_address();
        }
    } show_mac_address_menu{ "?", &mac_address_menu };

    item_t factory_reset_menu{ "Factory reset", &system_menu };
    item_t factory_reset_are_you_sure_menu{ "Really?", &factory_reset_menu };
    item_t factory_reset_no{ "No", &factory_reset_are_you_sure_menu, [] { item_t::go(&settings_menu); } };
    item_t factory_reset_yes{ "Yes", &factory_reset_are_you_sure_menu, [] { state_set(factory_reset_state); } };

}    // namespace

//////////////////////////////////////////////////////////////////////

void menu_init()
{
#if defined(DEBUG)
    // show_menu(&root_menu);
#endif
    last_menu_activity_timestamp = utc_wall_time;
    item_t::previous_item = nullptr;
    item_t::current_item = &settings_menu;
}

//////////////////////////////////////////////////////////////////////

void menu_update()
{
    // menu goes away automatically after 10 minutes of inactivity
    if((utc_wall_time.tv_sec - last_menu_activity_timestamp.tv_sec) > 10 * 60) {
        menu_exit();
    } else {
        item_t::current_item->on_update();
    }
}
