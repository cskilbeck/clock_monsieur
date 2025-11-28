#pragma once

#include "freertos/event_groups.h"

extern EventGroupHandle_t system_events;

#define SYS_EVENT_SNTP_UP BIT0
#define SYS_EVENT_BOOT_MSG_COMPLETE BIT1
#define SYS_EVENT_GOT_LOCATION BIT2
#define SYS_EVENT_GOT_TIMEZONE BIT3
#define SYS_EVENT_PING_OK BIT4
