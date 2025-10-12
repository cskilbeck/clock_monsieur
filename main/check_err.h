#pragma once

#include "esp_err.h"

//////////////////////////////////////////////////////////////////////

#define CHECK_ERR(x)                                       \
    do {                                                   \
        esp_err_t _err = ESP_ERROR_CHECK_WITHOUT_ABORT(x); \
        if(_err != ESP_OK) {                               \
            return _err;                                   \
        }                                                  \
    } while(false)
