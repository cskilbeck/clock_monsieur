#pragma once

#include <ctime>
#include "timezone_data.h"

char *create_posix_tz_string(const char *std_abbr, long std_offset_sec, const char *dst_abbr, long dst_offset_sec, time_t dst_start_epoch,
                             time_t dst_end_epoch);

// start at the root of the tree
void timezone_init();

// go back up one level
void timezone_back();

// select current entry
void timezone_select();

// previous entry
void timezone_up();

// next entry
void timezone_down();

tz_node_t const &timezone_current();

// get the first node of the parent
tz_node_t const &timezone_first();
