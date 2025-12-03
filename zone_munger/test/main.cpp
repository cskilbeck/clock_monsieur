#include <iostream>
#include <format>
#include <cstring>
#include <cstdio>

#include "../../firmware/main/timezone_data.h"

std::string format_time(int64_t time)
{
    time_t const x = time;
    tm t;
#if defined(_WIN64)
    gmtime_s(&t, &x);
#else
    gmtime_r(&t, &x);
#endif
    return std::format("{:04d}/{:02d}/{:02d} {:02d}:{:02d}", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
}

int get_offset_seconds(zone_offset_t const &zone_offset)
{
    return zone_offset.offset_seconds_10 * 10;
}

int64_t get_epoch_seconds(zone_offset_t const &zone_offset)
{
    uint16_t low = zone_offset.epoch_start_low;
    uint16_t high = zone_offset.epoch_start_high;
    return ((int32_t)high << 16 | low) + MIN_EPOCH;
}

int64_t get_epoch_seconds(zone_offset_t const *zone_offset)
{
    return get_epoch_seconds(*zone_offset);
}

char const *get_node_name(tz_node_t const &node)
{
    return &TZ_NAME_STRING[node.name_offset];
}

char const *get_node_name(tz_node_t const *node)
{
    return get_node_name(*node);
}

tz_details_t const *get_node_details(const tz_node_t *node)
{
    if(node->num_children != 0) {
        return nullptr;
    }
    return &TZ_DETAILS[node->children_index];
}

// Find the most recent timezone which started before now (i.e. the current one)

zone_offset_t const *find_timezone(time_t const now, zone_offset_t const *begin, zone_offset_t const *end)
{
    if(begin == end) {
        return end;
    }
    end -= 1;
    int64_t end_seconds = get_epoch_seconds(end);
    if(end_seconds <= now) {
        return end;
    }
    auto id = [](zone_offset_t const *m) -> int {
        return (int)(m - ZONE_OFFSETS);
    };
    printf("Range: %d - %d\n", id(begin), id(end));
    zone_offset_t const *low = begin;
    zone_offset_t const *high = end;
    zone_offset_t const *result = nullptr;
    while(low < high) {
        zone_offset_t const *mid = low + ((high - low) + 1) / 2;
        int64_t mid_seconds = get_epoch_seconds(mid);
        printf("Low = %d, High = %d, Mid = %d, Is %s < now?\n", id(low), id(high), id(mid), format_time(mid_seconds).c_str());
        if(mid_seconds < now) {
            result = mid;
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }
    return result;
}

// Find the tz_node_t for a given timezone location (e.g. Europe/London, America/Argentina/Buenos_Aires)

const tz_node_t *find_location(const char *path)
{
    if(path == nullptr) {
        return nullptr;
    }
    tz_node_t const *current_node = &TZ_NODES[0];
    char const *segment_start = path;
    while(*segment_start != '\0') {
        char const *segment_end = std::strchr(segment_start, '/');
        size_t segment_len = segment_end != nullptr ? segment_end - segment_start : std::strlen(segment_start);
        if(current_node->num_children == 0) {
            return nullptr;
        }
        bool match_found = false;
        uint16_t child_start_idx = current_node->children_index;
        uint16_t child_count = current_node->num_children;
        for(uint16_t i = 0; i < child_count; ++i) {
            tz_node_t const *child = &TZ_NODES[child_start_idx + i];
            char const *child_name = get_node_name(child);
            if(std::strncmp(segment_start, child_name, segment_len) == 0 && child_name[segment_len] == '\0') {
                current_node = child;
                match_found = true;
                break;
            }
        }
        if(!match_found) {
            return nullptr; // Path segment not found in current children
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

void print_node(tz_node_t const *node)
{
    if(node == nullptr) {
        printf("NULL Node!?\n");
    } else if(node->num_children == 0) {
        printf("%s:\n", get_node_name(node));
        tz_details_t const &details = TZ_DETAILS[node->children_index];
        for(int i = 0; i < details.offset_count; ++i) {
            int offset = i + details.offset_start_index;
            zone_offset_t const &zone_offset = ZONE_OFFSETS[offset];
            int64_t seconds = get_epoch_seconds(zone_offset);
            printf("%lld (%s),%d (offset %d)\n", seconds, format_time(seconds).c_str(), get_offset_seconds(zone_offset), offset);
        }
    } else {
        printf("%s\n", get_node_name(node));
    }
}

void show_node(int const node_index, int const indent = 0)
{
    tz_node_t const &node = TZ_NODES[node_index];
    if(node.num_children == 0) {
        printf("%*s%s:\n", indent, "", get_node_name(node));
        tz_details_t const &details = TZ_DETAILS[node.children_index];
        for(int i = 0; i < details.offset_count; ++i) {
            int offset = i + details.offset_start_index;
            zone_offset_t const &zone_offset = ZONE_OFFSETS[offset];
            int64_t seconds = get_epoch_seconds(zone_offset);
            std::string s = format_time(seconds);
            printf("%*s%lld (%s),%d (offset %d)\n", indent + 4, "", seconds, s.c_str(), get_offset_seconds(zone_offset), offset);
        }
    } else {
        printf("%*s%s\n", indent, "", get_node_name(node));
        for(int i = 0; i < node.num_children; ++i) {
            show_node(i + node.children_index, indent + 4);
        }
    }
}

int main()
{
    setbuf(stdout, nullptr);

    time_t now;
    time(&now);

    printf("Current UTC time: %s (%lld)\n", format_time(now).c_str(), now);

    char const *locations[] = {
        "Europe/London",
        // "Africa/Cairo",
        // "Asia/Tokyo",
        // "Australia/Sydney",
        // "America/Argentina/Buenos_Aires",
        // "Europe",
        // "Europe/",
        // "Europe//",
        // "Africa/FOO",
        // "Foo/Nah",
    };

    for(char const *location : locations) {
        tz_node_t const *node = find_location(location);
        if(node == nullptr || node->num_children != 0) {
            printf("Location %s not found\n", location);
            continue;
        }
        tz_details_t const *details = get_node_details(node);
        if(details != nullptr) {
            for(int i=0; i<details->offset_count; ++i) {
                int o = details->offset_start_index + i;
                zone_offset_t const &zone = ZONE_OFFSETS[o];
                printf("Zone [%4d]: %s (%d)\n", o, format_time(get_epoch_seconds(zone)).c_str(), get_offset_seconds(zone));
            }
            zone_offset_t const *start = &ZONE_OFFSETS[details->offset_start_index];
            zone_offset_t const *end = &ZONE_OFFSETS[details->offset_start_index + details->offset_count];
            zone_offset_t const *found = find_timezone(now, start, end);
            if(!found) {
                printf("ERROR: Can't get current time offset for %s\n", location);
            } else {
                printf("End: %s\n", format_time(get_epoch_seconds(found)).c_str());
                printf("Currently %s GMT offset is %d seconds\n", location, found->offset_seconds_10 * 10);
            }
        }
    }

    return 0;
}
