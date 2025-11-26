#pragma once

#include "timezone_data.h"

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
