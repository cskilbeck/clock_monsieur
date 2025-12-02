#include <iostream>
#include <format>


#include "../timezone_data.h"

constexpr int64_t MIN_EPOCH = 1735689600; //int(datetime(2025, 1, 1).timestamp())  # Jan 1 2025

char const *name(tz_node_t const &node)
{
    return TZ_NAME_STRING + node.name_offset;
}

std::string format_time(int64_t time)
{
    tm *t = gmtime((time_t *)&time);
    return std::format("{:04d}/{:02d}/{:02d}", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

}

int64_t epoch_seconds(zone_offset_t const &zone_offset)
{
    uint16_t low = zone_offset.epoch_start_low;
    uint16_t high = zone_offset.epoch_start_high;
    return (((int32_t)high << 16) | low) + MIN_EPOCH;
}

int offset_seconds(zone_offset_t const &zone_offset)
{
    return zone_offset.offset_seconds_10 * 10;
}

void show_node(int const node_index, int const indent = 0)
{
    tz_node_t const &node = TZ_NODES[node_index];
    if(node.num_children == 0) {
        printf("%*s%s:\n", indent, "", name(node));
        tz_details_t const &details = TZ_DETAILS[node.children_index];
        for(int i=0; i<details.offset_count; ++i) {
            int offset = i + details.offset_start_index;
            zone_offset_t const &zone_offset = ZONE_OFFSETS[offset];
            int64_t seconds = epoch_seconds(zone_offset);
            std::string s = format_time(seconds);
            printf("%*s%lld (%s),%d (offset %d)\n", indent+4, "", seconds, s.c_str(), offset_seconds(zone_offset), offset);
        }
    } else {
        printf("%*s%s\n", indent, "", name(node));
        for(int i=0; i<node.num_children; ++i) {
            show_node(i + node.children_index, indent + 4);
        }
    }
}

int main()
{
    for(int i=0; i<10; ++i) {
        show_node(i);
    }
    fflush(stdout);
    return 0;
}
