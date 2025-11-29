#pragma once

#include "freertos/event_groups.h"

extern EventGroupHandle_t system_events;

#define SYS_EVENT_BOOT_MSG_COMPLETE BIT0
#define SYS_EVENT_WIFI_CONNECTED BIT1
#define SYS_EVENT_NETWORK_CONNECTED BIT2
#define SYS_EVENT_SNTP_SYNCHRONIZED BIT3
#define SYS_EVENT_GOT_LOCATION BIT4
#define SYS_EVENT_GOT_TIMEZONE BIT5
