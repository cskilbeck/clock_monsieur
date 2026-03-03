//////////////////////////////////////////////////////////////////////

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "util.h"
#include "linked_list.h"
#include "main.h"
#include "clock_time.h"
#include "graphics.h"
#include "button.h"
#include "menu.h"
#include "wifi.h"
#include "state.h"
#include "settings.h"
#include "version.h"
#include "buzzer.h"
#include "melodies.h"

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
        using list_node = chs::list_node<item_t>;

        list_node node;

        using list = chs::linked_list<item_t, &item_t::node>;

        list children;

        item_t *parent = nullptr;

        // current menu item
        static list::iterator current_item;

        // previous menu item (or null)
        static char const *old_text;

        // transition from previous to current item in pixels
        static int transition_x;
        static int transition_y;

        static int transition_xvel;
        static int transition_yvel;

        // manual panning x
        static int name_x;

        //////////////////////////////////////////////////////////////////////

        item_t(item_t *_parent) : node(), children(), parent(_parent)
        {
            parent = _parent;
            if(_parent) {
                _parent->children.push_back(this);
            }
        }

        //////////////////////////////////////////////////////////////////////

        static void go(item_t::list::iterator where, int xvel, int yvel)
        {
            if(where == item_t::current_item) {
                return;
            }
            LOG_DEBUG("GO to %s", where->text());

            transition_x = item_t::name_x;
            transition_y = 0;
            transition_xvel = xvel;
            transition_yvel = yvel;
            old_text = item_t::current_item->text();
            name_x = 0;
            current_item = where;
            last_menu_activity_timestamp = utc_wall_time.tv_sec;
        }

        //////////////////////////////////////////////////////////////////////

        virtual char const *text() const
        {
            return "root";
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_select()
        {
            if(children.empty()) {
                go(parent, 2, 0);
            } else {
                go(children.head(), -2, 0);
            }
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_up()
        {
            // UP previous menu item
            auto &siblings = parent->children;
            auto prev = --list::iterator(this);
            if(prev == siblings.end()) {
                --prev;
            }
            go(prev, 0, 1);
        }

        //////////////////////////////////////////////////////////////////////

        virtual void on_down()
        {
            // DOWN next menu item
            auto &siblings = parent->children;
            auto next = ++list::iterator(this);
            if(next == siblings.end()) {
                ++next;
            }
            go(next, 0, -1);
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
            if(old_text != nullptr) {
                transition_x += transition_xvel;
                transition_y += transition_yvel;
                int x = transition_x / transition_speed;
                int y = transition_y / transition_speed;
                int prev_width = font_5x7_narrow_modern_font.draw_string(gfx, old_text, x, y, 1.0f);
                offset_x = (transition_xvel == 0) ? 0 : (transition_xvel < 0) ? x + prev_width + x_gap : x - (width + x_gap);
                offset_y = (transition_yvel == 0) ? 0 : (transition_yvel < 0) ? y + screen_height + y_gap : y - (screen_height + y_gap);
                if((transition_xvel > 0 && offset_x >= 0) ||    //
                   (transition_xvel < 0 && offset_x <= 0) ||    //
                   (transition_yvel > 0 && offset_y >= 0) ||    //
                   (transition_yvel < 0 && offset_y <= 0)) {
                    old_text = nullptr;
                } else {
                    font_5x7_narrow_modern_font.draw_string(gfx, text(), offset_x, offset_y, 1.0f);
                }
            }

            // finished scrolling to new item?
            if(old_text == nullptr) {

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

    char const *item_t::old_text = nullptr;
    int item_t::transition_x;
    int item_t::transition_y;
    int item_t::transition_xvel;
    int item_t::transition_yvel;

    item_t::list::iterator item_t::current_item((item_t *)nullptr);
    int item_t::name_x{ 0 };

    item_t root_menu{ nullptr };

    //////////////////////////////////////////////////////////////////////

#if defined(DEBUG)
    void show_menu(item_t const *m, int indent = 0)
    {
        LOG_INFO("%*s%s", indent, "", m->text());
        for(item_t const &c : m->children) {
            show_menu(&c, indent + 2);
        }
    }
#endif

    //////////////////////////////////////////////////////////////////////

    void menu_exit()
    {
        settings.save();
        item_t::current_item = item_t::list::iterator((item_t *)nullptr);
        state_set<clock_state_t>();
    }

    //////////////////////////////////////////////////////////////////////
    // Accelerating hold-repeat: call each frame with held state of up/down.
    // Returns true when the action should fire.

    bool hold_repeat(button_t const &btn, int &hold_frames)
    {
        if(btn.pressed) {
            hold_frames = 0;
            return true;
        }
        if(!btn.held) {
            return false;
        }
        hold_frames++;
        if(hold_frames <= 50) {
            return false;
        }
        int rate = 15 - (hold_frames - 50) / 20;
        if(rate < 1) {
            rate = 1;
        }
        return (hold_frames % rate) == 0;
    }

    //////////////////////////////////////////////////////////////////////

    template <typename T, T &value> struct enum_item_t : item_t
    {
        using item_t::item_t;

        void cycle_enum(int direction)
        {
            transition_x = item_t::name_x;
            transition_y = 0;
            transition_xvel = 0;
            transition_yvel = -direction;
            old_text = text();
            name_x = 0;
            last_menu_activity_timestamp = utc_wall_time.tv_sec;
            int v = (int)value + direction;
            size_t num_values = count_enum_values<T>();
            if(v < 0) {
                v = num_values - 1;
            } else if(v >= num_values) {
                v = 0;
            }
            value = (T)v;
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
            cycle_enum(1);
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

    MENU_HEADER(alarm_menu, "Alarm", &root_menu);

    struct : enum_item_t<alarm_enabled_t, settings.alarm_enabled>
    {
        using enum_item_t::enum_item_t;
        void on_up() override { item_t::on_up(); }
        void on_down() override { item_t::on_down(); }
    } alarm_enabled_toption(&alarm_menu);

    MENU_HEADER(alarm_set_menu, "Set", &alarm_menu);

    struct : item_t
    {
        using item_t::item_t;

        int field = 0;    // 0=hour, 1=minute
        int hold_frames = 0;

        char const *text() const override
        {
            return "Time";
        }

        void on_update() override
        {
            uint8_t &hour = settings.alarm_hour;
            uint8_t &minute = settings.alarm_minute;

            // left/right: switch field or go back
            if(button_left.pressed) {
                if(field == 1) {
                    field = 0;
                } else {
                    go(parent, 2, 0);
                    return;
                }
            } else if(button_right.pressed) {
                if(field == 0) {
                    field = 1;
                }
            }

            // select: back to alarm header
            if(button_select.pressed) {
                go(parent, 2, 0);
                return;
            }

            // up/down: adjust selected field
            if(hold_repeat(button_up, hold_frames)) {
                if(field == 0) {
                    hour = (hour + 1) % 24;
                } else {
                    minute = (minute + 1) % 60;
                }
            } else if(hold_repeat(button_down, hold_frames)) {
                if(field == 0) {
                    hour = (hour + 23) % 24;
                } else {
                    minute = (minute + 59) % 60;
                }
            }

            // draw HH:MM with narrow font (fits A/P suffix in 12h mode)
            int display_hour = hour;
            bool is_pm = false;
            bool is_24h = settings.clock_mode == clock_mode_t::clock_24_hour;
            char const *fmt;
            if(is_24h) {
                display_hour %= 24;
                fmt = "%02d%02d";
            } else {
                is_pm = (hour % 24) >= 12;
                display_hour = hour % 12;
                if(display_hour == 0) {
                    display_hour = 12;
                }
                fmt = "%2d%02d";
            }
            char buf[8];
            sprintf(buf, fmt, display_hour, (int)minute);

            gfx.clear();
            font_t const &font = font_5x7_narrow_modern_font;

            // blink selected field (use frame counter)
            static int blink_frame = 0;
            blink_frame++;
            bool visible = (blink_frame % 40) < 28;

            float hour_alpha = (field == 0 && !visible) ? 0.0f : 1.0f;
            float minute_alpha = (field == 1 && !visible) ? 0.0f : 1.0f;

            // narrow font: digits centered at 2, 6, colon at 9, digits at 13, 17, A/P at 20
            font.draw_char_centered(gfx, buf[0], 2, 0, hour_alpha);
            font.draw_char_centered(gfx, buf[1], 6, 0, hour_alpha);
            font.draw_char_centered(gfx, buf[2], 13, 0, minute_alpha);
            font.draw_char_centered(gfx, buf[3], 17, 0, minute_alpha);

            // steady colon at column 9
            gfx.buffer[graphics_t::matrix_lookup[2][9]] = 1.0f;
            gfx.buffer[graphics_t::matrix_lookup[4][9]] = 1.0f;

            // AM/PM indicator in 12h mode
            if(!is_24h) {
                font.draw_char(gfx, is_pm ? 'P' : 'A', 20, 0, 1.0f);
            }

            gfx.display();
        }
    } alarm_time_menu{ &alarm_set_menu };

    MENU_HEADER(alarm_mode_menu, "Repeat", &alarm_menu);
    MENU_OPTION(alarm_mode_t, settings.alarm_mode, &alarm_mode_menu);

    MENU_HEADER(alarm_melody_menu, "Melody", &alarm_menu);

    struct : enum_item_t<alarm_melody_t, settings.alarm_melody>
    {
        using enum_item_t::enum_item_t;

        void cycle_enum_with_preview(int direction)
        {
            cycle_enum(direction);
            int idx = (int)settings.alarm_melody;
            if(idx >= 0 && idx < melodies::count) {
                auto const &m = melodies::table[idx];
                buzzer_play_melody(m.notes, m.count, false);
            }
        }

        void on_up() override
        {
            cycle_enum_with_preview(-1);
        }

        void on_down() override
        {
            cycle_enum_with_preview(1);
        }

        void on_select() override
        {
            cycle_enum_with_preview(1);
        }

        void on_update() override
        {
            if(old_text == nullptr && button_left.pressed && name_x == 0) {
                buzzer_stop();
            }
            item_t::on_update();
        }
    } alarm_melody_option{ &alarm_melody_menu };

    MENU_HEADER(alarm_snooze_menu, "Snooze", &alarm_menu);

    struct : item_t
    {
        using item_t::item_t;

        int hold_frames = 0;

        char const *text() const override
        {
            static char buf[8];
            sprintf(buf, "%d min", settings.alarm_snooze);
            return buf;
        }

        void on_select() override
        {
            go(parent, 2, 0);
        }

        void on_update() override
        {
            if(hold_repeat(button_up, hold_frames) && settings.alarm_snooze < 30) {
                settings.alarm_snooze++;
            } else if(hold_repeat(button_down, hold_frames) && settings.alarm_snooze > 1) {
                settings.alarm_snooze--;
            }
            item_t::on_update();
        }
    } alarm_snooze_option{ &alarm_snooze_menu };

    //////////////////////////////////////////////////////////////////////

    MENU_HEADER(timer_menu, "Timer", &root_menu);

    struct : item_t
    {
        using item_t::item_t;

        int hold_frames = 0;

        char const *text() const override
        {
            return "Set";
        }

        void on_update() override
        {
            uint16_t &secs = settings.timer_seconds;

            // up/down: adjust duration
            if(hold_repeat(button_up, hold_frames)) {
                if(secs < 3599) {
                    secs++;
                }
            } else if(hold_repeat(button_down, hold_frames)) {
                if(secs > 1) {
                    secs--;
                }
            }

            // select: start countdown
            if(button_select.pressed) {
                settings.save();
                current_item = list::iterator((item_t *)nullptr);
                state_set<timer_state_t>();
                return;
            }

            // left: back to Timer header
            if(button_left.pressed) {
                go(parent, 2, 0);
                return;
            }

            // draw M:SS
            int mins = secs / 60;
            int s = secs % 60;
            char buf[8];
            sprintf(buf, "%2d%02d", mins, s);

            gfx.clear();
            font_t const &font = settings.clock_font == clock_font_t::Square ? square_font_font : font_5x7_font;
            font.draw_char_centered(gfx, buf[0], 2, 0, 1.0f);
            font.draw_char_centered(gfx, buf[1], 8, 0, 1.0f);
            font.draw_char_centered(gfx, buf[2], 16, 0, 1.0f);
            font.draw_char_centered(gfx, buf[3], 22, 0, 1.0f);

            // steady colon
            gfx.buffer[graphics_t::matrix_lookup[2][12]] = 1.0f;
            gfx.buffer[graphics_t::matrix_lookup[4][12]] = 1.0f;

            gfx.display();
        }
    } timer_setup_menu{ &timer_menu };

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
        char const *text() const
        {
            return "Auto";
        }
        void on_select() override
        {
            settings.timezone_mode = timezone_mode_t::Auto;
            xEventGroupSetBits(system_events, SYS_EVENT_NEED_LOCATION);
            item_t::on_select();
        }
    } timezone_auto_menu{ &timezone_menu };

    struct : item_t
    {
        using item_t::item_t;
        char const *text() const
        {
            return "Select";
        }
        void on_select() override
        {
            current_item = list::iterator((item_t *)nullptr);
            state_set<timezone_select_state_t>();
        }
    } timezone_select_menu{ &timezone_menu };

    ///// System

    MENU_HEADER(system_menu, "System", &root_menu);

    MENU_HEADER(version_menu, "Version", &system_menu);
    struct : item_t
    {
        using item_t::item_t;
        char const *text() const
        {
            return "V" VERSION_STR;
        }
    } show_version_menu{ &version_menu };

    MENU_HEADER(ip_address_menu, "IP Address", &system_menu);
    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_ip_address();
        }
    } show_ip_menu{ &ip_address_menu };

    MENU_HEADER(mac_address_menu, "MAC Address", &system_menu);
    struct : item_t
    {
        using item_t::item_t;
        char const *text() const override
        {
            return wifi_mac_address();
        }
    } show_mac_address_menu{ &mac_address_menu };

    MENU_HEADER(factory_reset_menu, "Factory reset", &system_menu);
    MENU_HEADER(factory_reset_really_menu, "Really?", &factory_reset_menu);
    MENU_HEADER(factory_reset_no, "No", &factory_reset_really_menu);

    struct : item_t
    {
        using item_t::item_t;
        void on_select() override
        {
            current_item = list::iterator((item_t *)nullptr);
            state_set<factory_reset_state_t>();
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
    item_t::old_text = nullptr;
    if(item_t::current_item.get() == nullptr) {
        item_t::current_item = item_t::list::iterator(&alarm_menu);
    }
}

//////////////////////////////////////////////////////////////////////

void menu_update()
{
    // menu goes away automatically after 30 seconds of inactivity
    if((utc_wall_time.tv_sec - last_menu_activity_timestamp) > 30) {
        menu_exit();
    } else {
        item_t::current_item->on_update();
    }
}
