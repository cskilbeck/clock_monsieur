#pragma once

//////////////////////////////////////////////////////////////////////

#include <ctime>
#include "esp_err.h"
#include "timezone_data.h"

//////////////////////////////////////////////////////////////////////

void timezone_select_init();
void timezone_select_update();

esp_err_t timezone_set(char const *location);
esp_err_t timezone_update(timeval &current_time, int &offset_seconds);

extern int timezone_offset_seconds;
