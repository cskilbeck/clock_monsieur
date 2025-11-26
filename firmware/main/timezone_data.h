#pragma once

#include <cstdint>
#include <cstddef>

// Constants for the is_leaf field
#define TZ_NODE 0
#define TZ_LEAF 1

// Define the structure for a timezone node
struct tz_node_t
{
    // Constructor for BRANCH nodes (takes ints)
    constexpr tz_node_t(uint16_t name, int idx, int num) : name_offset(name), is_leaf(TZ_NODE), pad(0), child_index(idx), child_count(num)
    {
    }

    // Constructor for LEAF nodes (takes floats)
    constexpr tz_node_t(uint16_t name, float lat, float lon) : name_offset(name), is_leaf(TZ_LEAF), pad(0), latitude(lat), longitude(lon)
    {
    }

    uint16_t name_offset;
    uint16_t is_leaf : 1;
    uint16_t pad : 15;
    union
    {
        struct
        {
            int child_index;
            int child_count;
        };
        struct
        {
            float latitude;
            float longitude;
        };
    };
};

extern const char TZ_NAME_STRING[];
extern const tz_node_t TZ_NODES[432];
