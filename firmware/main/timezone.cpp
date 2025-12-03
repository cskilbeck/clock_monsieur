//////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdbool>
#include <ctime>
#include <cstring>

#include "util.h"
#include "button.h"
#include "state.h"
#include "timezone_data.h"

LOG_CONTEXT("timezone");

//////////////////////////////////////////////////////////////////////

namespace
{
    //////////////////////////////////////////////////////////////////////

    struct zone_offset : zone_offset_t
    {
        int offset_seconds() const
        {
            return offset_seconds_10 * 10;
        }

        int64_t epoch_start() const
        {
            uint16_t low = epoch_start_low;
            uint16_t high = epoch_start_high;
            return ((uint32_t)high << 16 | low) + MIN_EPOCH;
        }
    };

    //////////////////////////////////////////////////////////////////////

    struct tz_node : tz_node_t
    {
        char const *name() const
        {
            return &TZ_NAME_STRING[name_offset];
        }

        tz_details_t const *details() const
        {
            if(num_children != 0) {
                return nullptr;
            }
            return &TZ_DETAILS[children_index];
        }
    };

    zone_offset const *zone_offsets = reinterpret_cast<zone_offset const *>(ZONE_OFFSETS);
    tz_node const *tz_nodes = reinterpret_cast<tz_node const *>(TZ_NODES);

    //////////////////////////////////////////////////////////////////////
    // Find the most recent timezone which started before now (i.e. the current one)

    zone_offset const *find_timezone(time_t const now, zone_offset const *begin, zone_offset const *end)
    {
        if(begin == end) {
            return end;
        }
        end -= 1;
        int64_t end_seconds = end->epoch_start();
        if(end_seconds <= now) {
            return end;
        }
        zone_offset const *low = begin;
        zone_offset const *high = end;
        zone_offset const *result = nullptr;
        while(low < high) {
            zone_offset const *mid = low + (high - low) / 2;
            if(mid->epoch_start() < now) {
                result = mid;
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }
        return result;
    }

    //////////////////////////////////////////////////////////////////////
    // Find the tz_node for a given timezone location (e.g. Europe/London, America/Argentina/Buenos_Aires)

    tz_node const *find_location(const char *path)
    {
        if(path == nullptr) {
            return nullptr;
        }
        tz_node const *current_node = &tz_nodes[0];
        char const *segment_start = path;
        while(*segment_start != '\0') {
            char const *segment_end = strchr(segment_start, '/');
            size_t segment_len = segment_end != nullptr ? segment_end - segment_start : strlen(segment_start);
            if(current_node->num_children == 0) {
                return nullptr;
            }
            bool match_found = false;
            uint16_t child_start_idx = current_node->children_index;
            uint16_t child_count = current_node->num_children;
            for(uint16_t i = 0; i < child_count; ++i) {
                tz_node const *child = &tz_nodes[child_start_idx + i];
                char const *child_name = child->name();
                if(strncmp(segment_start, child_name, segment_len) == 0 && child_name[segment_len] == '\0') {
                    current_node = child;
                    match_found = true;
                    break;
                }
            }
            if(!match_found) {
                return nullptr;    // Path segment not found in current children
            }
            if(segment_end != nullptr) {
                segment_start = segment_end + 1;
                if(*segment_start == '\0') {
                    break;
                }
            } else {
                break;
            }
        }
        return current_node;
    }

    //////////////////////////////////////////////////////////////////////

    struct location_t
    {
        uint16_t index;     // top node
        uint16_t offset;    // where in the list of nodes
        uint16_t parent;    // parent node
    };

    //////////////////////////////////////////////////////////////////////

    int current_index = 0;
    simplestack_t<location_t, 3> parent_index{};
    int node_parent = 0;
    int node_count = 0;
    int node_index = 0;
    int name_x = 0;

    zone_offset const *zone_offset_start{};
    zone_offset const *zone_offset_end{};

    //////////////////////////////////////////////////////////////////////

    esp_err_t set_timezone(tz_node const *node)
    {
        if(node == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }
        tz_details_t const *details = node->details();
        if(details == nullptr) {
            return ESP_ERR_NOT_SUPPORTED;    // non-leaf path
        }
        zone_offset_start = &zone_offsets[details->offset_start_index];
        zone_offset_end = &zone_offsets[details->offset_start_index + details->offset_count];
        return ESP_OK;
    }

}    // namespace

int timezone_offset_seconds = 0;

//////////////////////////////////////////////////////////////////////

void timezone_select_init()
{
    tz_node const &root_node = tz_nodes[0];
    node_parent = 0;
    current_index = root_node.children_index;
    node_count = root_node.num_children;
    node_index = 0;
    parent_index.clear();
}

//////////////////////////////////////////////////////////////////////

void timezone_select_update()
{
    tz_node const *current = &tz_nodes[current_index + node_index];
    char const *name = current->name();
    int width = font_5x7_narrow_font.measure_string(name);
    int min_name_x = (screen_width - width) * 2;
    gfx.clear();
    font_5x7_narrow_font.draw_long_string(gfx, name, name_x / 2, 0, 1.0f, 0.5f);
    gfx.display();

    // control is lagged by 1 frame, whevs
    if(button_select.pressed) {
        if(current->num_children != 0) {
            parent_index.push({ (uint16_t)current_index, (uint16_t)node_index, (uint16_t)node_parent });
            tz_node const *current = &tz_nodes[current_index + node_index];
            current_index = current->children_index;
            node_index = 0;
            name_x = 0;
            node_count = current->num_children;
        } else {
            LOG_INFO("%s selected!", current->name());
            set_timezone(current);
            state_set(clock_state);
        }
    }
    if(button_up.pressed && node_index != 0) {
        node_index -= 1;
    }
    if(button_down.pressed && node_index < node_count - 1) {
        node_index += 1;
    }
    if(button_right.held && name_x > min_name_x) {
        name_x -= 1;
    }
    if(button_left.held && name_x < 0) {
        name_x += 1;
    } else if(button_left.pressed && name_x == 0) {
        if(parent_index.empty()) {
            state_set(clock_state);
        } else {
            location_t parent = parent_index.pop();
            current_index = parent.index;
            node_index = parent.offset;
            node_parent = parent.parent;
            tz_node const *current = &tz_nodes[current_index];
            node_count = tz_nodes[node_parent].num_children;
            name_x = 0;
        }
    }
}

//////////////////////////////////////////////////////////////////////

esp_err_t timezone_set(char const *location)
{
    LOG_INFO("Looking for timezone location %s", location);
    tz_node const *node = find_location(location);
    if(node == nullptr) {
        LOG_ERROR("Can't find timezone location %s", location);
        return ESP_ERR_NOT_FOUND;
    }
    LOG_INFO("Found timezone location %s", node->name());
    return set_timezone(node);
}

//////////////////////////////////////////////////////////////////////

esp_err_t timezone_update(timeval &current_time, int &offset_seconds)
{
    if(zone_offset_start == nullptr) {
        offset_seconds = 0;
        return ESP_ERR_INVALID_STATE;
    }
    zone_offset const *found = find_timezone(current_time.tv_sec, zone_offset_start, zone_offset_end);
    if(!found) {
        return ESP_ERR_NOT_FOUND;
    }
    offset_seconds = found->offset_seconds();
    return ESP_OK;
}