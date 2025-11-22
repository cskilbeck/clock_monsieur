#pragma once

#include <esp_err.h>

esp_err_t check_firmware_version();
void ota_mark_app_valid();
