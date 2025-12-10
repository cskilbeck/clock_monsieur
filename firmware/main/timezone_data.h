#pragma once

#include <cstdint>
#include <cstddef>

constexpr int64_t MIN_EPOCH = 1735689600; // Jan 1st 2025
constexpr size_t LONGEST_TZ_LOCATION_NAME = 30;

// timezone offset data
struct zone_offset_t
{
    uint16_t epoch_start_high;  // high 16 bits of epoch start time in seconds
    uint16_t epoch_start_low;   // low 16 bits of epoch start time in seconds
    int16_t offset_seconds_10;  // GMT offset in seconds * 10
};

// timezone details (coordinates and offset info)
struct tz_details_t
{
    uint16_t offset_start_index;  // Index into ZONE_OFFSETS array
    uint16_t offset_count;        // Number of offset entries for this location
};

// timezone node
struct tz_node_t
{
    uint16_t name_offset;     // Offset into TZ_NAME_STRING
    uint16_t num_children;    // # of child nodes (>0 = treenode, 0 =leafnode (zone)) 
    uint16_t children_index;  // For non-leaf: index into TZ_NODES, for leaf: index into TZ_DETAILS
};

extern const char TZ_NAME_STRING[3730];
extern const zone_offset_t ZONE_OFFSETS[13370];
extern const tz_details_t TZ_DETAILS[418];
extern const tz_node_t TZ_NODES[433];
