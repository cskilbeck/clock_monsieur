#pragma once

#include <freertos/event_groups.h>

esp_err_t provisioning_init();

const int WIFI_CONNECTED_EVENT = BIT0;

extern EventGroupHandle_t wifi_event_group;
