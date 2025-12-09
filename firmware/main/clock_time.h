#pragma once

//////////////////////////////////////////////////////////////////////

void clock_init();
void clock_update();

void clock_get_time(timeval *local_time = nullptr, timeval *utc_time = nullptr);
