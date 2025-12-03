#pragma once

#include "esp_err.h"

struct http_data_context_t
{
    char *buffer;
    size_t buffer_size;
    size_t len;
    int response_code;

    http_data_context_t(char *buff, size_t buffer_len) : buffer(buff), buffer_size(buffer_len)
    {
    }

    void reset()
    {
        buffer[0] = 0;
        len = 0;
    }
};

esp_err_t do_http_request(char const *url, http_data_context_t *ctx, int retries, int retry_delay_ms);