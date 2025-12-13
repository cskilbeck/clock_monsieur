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
    time_t last_menu_activity_timestamp{ 0 };

    //////////////////////////////////////////////////////////////////////

    void menu_exit();

    //////////////////////////////////////////////////////////////////////

    struct item_t
    {
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

        item_t *parent = nullptr;
        item_t *prev = nullptr;
        item_t *next = nullptr;
        item_t *children = nullptr;

        //////////////////////////////////////////////////////////////////////

        item_t(item_t *_parent) : parent(_parent), prev(nullptr), next(nullptr), children(nullptr)
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

        static void go(item_t *where, int xvel, int yvel)
        {
            if(where == item_t::current_item) {
                return;
            }
            LOG_DEBUG("GO to %s", where->text());

            transition_x = item_t::name_x;
            transition_y = 0;
            transition_xvel = xvel;
            transition_yvel = yvel;
            previous_item = item_t::current_item;
            name_x = 0;
            current_item = where;
            last_menu_activity_timestamp = utc_wall_time.tv_sec;
        }

        //////////////////////////////////////////////////////////////////////

        virtual char const *text() const
        {
            return "?";
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_select()
        {
            if(children != nullptr) {
                go(children, -2, 0);
            } else {
                go(parent, 2, 0);
            }
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_up()
        {
            // UP previous menu item
            if(prev != nullptr) {
                go(prev, 0, 1);
            } else {
                // or wrap to last sibling
                item_t *n = next;
                while(n != nullptr && n->next != nullptr) {
                    n = n->next;
                }
                if(n != nullptr) {
                    go(n, 0, 1);
                }
            }
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_down()
        {
            // DOWN next menu item
            if(next != nullptr) {
                go(next, 0, -1);
            } else {
                // or wrap to 1st sibling
                go(parent->children, 0, -1);
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

            // if scrolling to a new item
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

            // finished scrolling to new item?
            if(previous_item == nullptr) {

                float scrollbar_color = 0.5f;
                font_5x7_narrow_modern_font.draw_long_string(gfx, text(), name_x / pan_speed, 0, 1.0f, scrollbar_color);

                if(button_up.pressed) {
                    on_up();

                } else if(button_down.pressed) {
                    on_down();

                } else if(button_left.held && name_x < 0) {
                    // LEFT scroll left
                    name_x += 1;

                } else if(button_right.held && name_x > min_name_x) {
                    // RIGHT scroll right
                    name_x -= 1;

                } else if(button_left.pressed && name_x == 0) {
                    // LEFT PRESS (if at 0) go up to parent
                    if(parent != nullptr && parent->parent != nullptr) {
                        go(parent, 2, 0);
                    } else {
                        // or back to clock if gone back to the root node (which has no parent)
                        menu_exit();
                    }

                } else if(button_select.pressed) {
                    // SELECT go into child menu or call on_select()
                    on_select();
                }
            }
            gfx.display();
        }
    };

    //////////////////////////////////////////////////////////////////////

    item_t *item_t::previous_item = nullptr;
    int item_t::transition_x;
    int item_t::transition_y;
    int item_t::transition_xvel;
    int item_t::transition_yvel;

    item_t *item_t::current_item{ nullptr };
    int item_t::name_x{ 0 };

    item_t root_menu{ nullptr };

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

    template <typename T, T &value> struct enum_item_t : item_t
    {
        using item_t::item_t;

        void cycle_enum(int direction)
        {
            value = (T)(((int)value + direction) % count_enum_values<T>());
        }
        void on_up() override
        {
            cycle_enum(-1);
        }
        void on_down() override
        {
            cycle_enum(1);
        }
        void on_select() override
        {
            go(parent, 2, 0);
        }
        char const *text() const override
        {
            return enum_to_string(value);
        }
    };

#define MENU_OPTION(type, name, parent) \
    struct : enum_item_t<type, name>    \
    {                                   \
        using enum_item_t::enum_item_t; \
    } type##option(parent)

#define MENU_HEADER(name, txt, parent) \
    struct : item_t                    \
    {                                  \
        using item_t::item_t;          \
        char const *text() const       \
        {                              \
            return txt;                \
        }                              \
    } name(parent)

    //////////////////////////////////////////////////////////////////////

    MENU_HEADER(mode_menu, "Mode", &root_menu);
    MENU_OPTION(clock_mode_t, settings.clock_mode, &mode_menu);

    MENU_HEADER(dimmer_menu, "Dimmer", &root_menu);
    MENU_OPTION(auto_brightness_t, settings.auto_brightness, &dimmer_menu);

    MENU_HEADER(brightness_menu, "Brightness", &root_menu);
    MENU_OPTION(brightness_level_t, settings.brightness, &brightness_menu);

    MENU_HEADER(seconds_menu, "Seconds", &root_menu);
    MENU_OPTION(seconds_mode_t, settings.seconds_mode, &seconds_menu);

    MENU_HEADER(colon_menu, "Colon", &root_menu);
    MENU_OPTION(colon_mode_t, settings.colon_mode, &colon_menu);

    MENU_HEADER(font_menu, "Font", &root_menu);
    MENU_OPTION(clock_font_t, settings.clock_font, &font_menu);

    MENU_HEADER(fade_menu, "Fade", &root_menu);
    MENU_OPTION(clock_fade_mode_t, settings.clock_fade_mode, &fade_menu);

    MENU_HEADER(ticks_menu, "Ticks", &root_menu);
    MENU_OPTION(tick_mode_t, settings.ticks, &ticks_menu);

    MENU_HEADER(timezone_menu, "Timezone", &root_menu);
    struct : item_t
    {
        using item_t::item_t;
        void on_select() override
        {
            settings.timezone_mode = timezone_mode_t::Auto;
            xEventGroupSetBits(system_events, SYS_EVENT_NEED_LOCATION);
            item_t::on_select();
        }
        char const *text() const
        {
            return "Auto";
        }
    } timezone_auto_menu{ &timezone_menu };

    struct : item_t
    {
        using item_t::item_t;
        void on_select() override
        {
            state_set(timezone_select_state);
        }
        char const *text() const
        {
            return "Select";
        }
    } timezone_select_menu{ &timezone_menu };

    ///// System

    MENU_HEADER(system_menu, "System", &root_menu);

    MENU_HEADER(version_menu, "Version", &system_menu);
    MENU_HEADER(ip_address_menu, "IP Address", &system_menu);
    MENU_HEADER(mac_address_menu, "MAC Address", &system_menu);

    MENU_HEADER(factory_reset_menu, "Factory reset", &system_menu);
    MENU_HEADER(factory_reset_really_menu, "Really?", &factory_reset_menu);
    MENU_HEADER(factory_reset_no, "No", &factory_reset_really_menu);

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const
        {
            return "V" VERSION_STR;
        }
    } show_version_menu{ &version_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_ip_address();
        }
    } show_ip_menu{ &ip_address_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_mac_address();
        }
    } show_mac_address_menu{ &mac_address_menu };

    struct : item_t
    {
        using item_t::item_t;
        void on_select() override
        {
            state_set(factory_reset_state);
        }
        char const *text() const
        {
            return "Yes";
        }
    } factory_reset_yes{ &factory_reset_really_menu };

}    // namespace

//////////////////////////////////////////////////////////////////////

void menu_init()
{
#if defined(DEBUG)
    // show_menu(&root_menu);
#endif
    last_menu_activity_timestamp = utc_wall_time.tv_sec;
    item_t::previous_item = nullptr;
    if(item_t::current_item == nullptr) {
        item_t::current_item = &mode_menu;
    }
}

//////////////////////////////////////////////////////////////////////

void menu_update()
{
    // menu goes away automatically after 3 minutes of inactivity
    if((utc_wall_time.tv_sec - last_menu_activity_timestamp) > 3 * 60) {
        menu_exit();
    } else {
        item_t::current_item->on_update();
    }
}
