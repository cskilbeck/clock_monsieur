#pragma once

#include "freertos/event_groups.h"

extern EventGroupHandle_t system_events;

#define SNTP_UP_BIT BIT0
#define BOOT_MSG_COMPLETE_BIT BIT1
#define GOT_LOCATION_BIT BIT2
#define GOT_TIMEZONE_BIT BIT3
