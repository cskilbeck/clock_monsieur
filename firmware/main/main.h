#pragma once

#include <time.h>

#include "freertos/event_groups.h"

extern EventGroupHandle_t system_events;

extern timeval wall_time;

#define SYS_EVENT_BOOT_MSG_COMPLETE BIT0
#define SYS_EVENT_WIFI_CONNECTED BIT1
#define SYS_EVENT_NETWORK_CONNECTED BIT2
#define SYS_EVENT_SNTP_SYNCHRONIZED BIT3
#define SYS_EVENT_GOT_LOCATION BIT4
#define SYS_EVENT_GOT_TIMEZONE BIT5
#define SYS_EVENT_PROVISIONING BIT6
#define SYS_EVENT_BLE_CONNECTED BIT7
#define SYS_EVENT_PROVISIONING_ERROR BIT8
#define SYS_EVENT_PROVISIONING_DONE BIT9
