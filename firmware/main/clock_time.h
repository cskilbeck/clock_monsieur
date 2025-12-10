#pragma once

//////////////////////////////////////////////////////////////////////

void clock_init();
void clock_update();

//////////////////////////////////////////////////////////////////////

extern timeval utc_wall_time;               // current wall time UTC
extern int timezone_offset_seconds;         // timezone offset for right now
extern int timezone_next_offset_seconds;    // timezone offset for 1 second in the future
