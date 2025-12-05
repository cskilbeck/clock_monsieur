#pragma once

//////////////////////////////////////////////////////////////////////

#include <ctime>
#include "esp_err.h"

//////////////////////////////////////////////////////////////////////

static int constexpr LONGEST_LOCATION_NAME = 30;    // America/Argentina/Buenos_Aires is 30

//////////////////////////////////////////////////////////////////////

void timezone_select_init();
void timezone_select_update();

esp_err_t timezone_set(char const *location);
esp_err_t timezone_get_offset_seconds(timeval &current_time, int &offset_seconds);

extern int timezone_offset_seconds;
