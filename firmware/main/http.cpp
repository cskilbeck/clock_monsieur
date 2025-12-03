//////////////////////////////////////////////////////////////////////

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"

#include "util.h"
#include "http.h"

LOG_CONTEXT("http");

//////////////////////////////////////////////////////////////////////

namespace
{
    esp_err_t http_event_handler(esp_http_client_event_t *evt)
    {
        http_data_context_t *ctx = (http_data_context_t *)evt->user_data;
        if(!ctx) {
            return ESP_FAIL;
        }

        switch(evt->event_id) {
        case HTTP_EVENT_ON_HEADER:
            ctx->len = 0;
            break;
        case HTTP_EVENT_ON_DATA:
            if(ctx->len + evt->data_len < ctx->buffer_size - 1) {
                memcpy(ctx->buffer + ctx->len, evt->data, evt->data_len);
                ctx->len += evt->data_len;
            } else {
                LOG_ERROR("Buffer overflow! Data truncated.");
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            if(ctx->len < sizeof(ctx->buffer) - 1) {
                ctx->buffer[ctx->len] = '\0';
                LOG_DEBUG("Response Size: %d", ctx->len);
                LOG_DEBUG("Response: %s", ctx->buffer);
            }
            break;
        case HTTP_EVENT_ERROR:
            LOG_WARN("HTTP Event Error: %d", evt->event_id);
            break;
        case HTTP_EVENT_DISCONNECTED:
            LOG_DEBUG("Disconnected, OK");
            break;
        default:
            break;
        }
        return ESP_OK;
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

esp_err_t do_http_request(char const *url, http_data_context_t *ctx, int retries, int retry_delay_ms)
{
    esp_http_client_config_t config = { .url = url, .event_handler = http_event_handler, .user_data = ctx };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    DEFER(esp_http_client_cleanup(client));

    do {
        ctx->reset();
        esp_err_t err = esp_http_client_perform(client);
        if(err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            LOG_DEBUG("STATUS_CODE: %d", status_code);
            if(status_code < 300) {
                return ESP_OK;
            }
            switch(status_code) {
            case 503:
                LOG_INFO("SERVER_UNAVAILABLE");
                break;
            case 429:
                LOG_INFO("TOO_MANY_REQUESTS");
                break;
            default:
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else if(err == ESP_ERR_HTTP_EAGAIN) {
            // just retry
            LOG_INFO("HTTP_EAGAIN");
        } else {
            LOG_ERROR("HTTP CLIENT Error %d (%s)", err, esp_err_to_name(err));
            return err;
        }
        retries -= 1;
        if(retries != 0) {
            delay_ms(retry_delay_ms);
        }
    } while(retries != 0);
    return ESP_ERR_INVALID_RESPONSE;
}
