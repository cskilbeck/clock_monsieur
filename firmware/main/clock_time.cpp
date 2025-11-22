//////////////////////////////////////////////////////////////////////

#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "provisioning.h"
#include "time.h"
#include "ota.h"
#include "util.h"


namespace
{
    LOG_CONTEXT("http");
    int timezone_offset_seconds = 0;
}    // namespace

//////////////////////////////////////////////////////////////////////

// http://ip-api.com/json
// {
//     "status": "success",
//     "country": "United Kingdom",
//     "countryCode": "GB",
//     "region": "ENG",
//     "regionName": "England",
//     "city": "Twickenham",
//     "zip": "TW1",
//     "lat": 51.4435,
//     "lon": -0.3289,
//     "timezone": "Europe/London",
//     "isp": "Virgin Media Limited",
//     "org": "Vmcbbuk",
//     "as": "AS5089 Virgin Media Limited",
//     "query": "92.236.115.196"
// }

static const char *TAG = "clock_time";

#define MAX_HTTP_OUTPUT_BUFFER 512

#define TIMEZONEDB_API_KEY "VV0F65261GPD"

bool got_location{ false };
bool got_timezone{ false };
bool got_firmware_version{ false };
bool got_sntp{ false };

double current_lat = 0.0;
double current_lon = 0.0;

//////////////////////////////////////////////////////////////////////

void get_time(struct timeval *tv_now)
{
    gettimeofday(tv_now, NULL);
    tv_now->tv_sec += timezone_offset_seconds;
}

//////////////////////////////////////////////////////////////////////
// Context structure to hold response data for an HTTP request

struct http_data_context_t
{
    char buffer[MAX_HTTP_OUTPUT_BUFFER];
    int len;
    int response_code;

    void reset()
    {
        len = 0;
        buffer[0] = 0;
    }
};

http_data_context_t context;

//////////////////////////////////////////////////////////////////////

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
        if(ctx->len + evt->data_len < sizeof(ctx->buffer) - 1) {
            memcpy(ctx->buffer + ctx->len, evt->data, evt->data_len);
            ctx->len += evt->data_len;
        } else {
            ESP_LOGE(TAG, "Buffer overflow! Data truncated.");
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        if(ctx->len < sizeof(ctx->buffer) - 1) {
            ctx->buffer[ctx->len] = '\0';
            ESP_LOGI(TAG, "Response Size: %d", ctx->len);
            ESP_LOGI(TAG, "Response: %s", ctx->buffer);
        }
        break;
    case HTTP_EVENT_ERROR:
        ESP_LOGW(TAG, "HTTP Event Error: %d", evt->event_id);
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected, OK");
        break;
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t do_http_request(char const *url, http_data_context_t *ctx, int retries, int retry_delay_ms)
{
    esp_http_client_config_t config = { .url = url, .event_handler = http_event_handler, .user_data = ctx };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    DEFER(esp_http_client_cleanup(client));

    while(retries != 0) {
        ctx->reset();

        esp_err_t err = esp_http_client_perform(client);
        if(err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            LOG_INFO("STATUS_CODE: %d", status_code);
            if(status_code < 400) {
                return ESP_OK;
            }
            switch(status_code) {
            case 503:    // SERVER_UNAVAILABLE
            case 429:    // TOO_MANY_REQUESTS
                delay_ms(retry_delay_ms);
                break;
            default:
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else if(err == ESP_ERR_HTTP_EAGAIN) {
            delay_ms(retry_delay_ms);
        } else {
            ESP_LOGE(TAG, "HTTP CLIENT Error %d (%s)", err, esp_err_to_name(err));
            return err;
        }
        retries -= 1;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

//////////////////////////////////////////////////////////////////////
// request to ip-api.com to get lat/lon.

esp_err_t get_location_data()
{
    esp_err_t err = do_http_request("http://ip-api.com/json", &context, 10, 1000);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Can't get location!?");
        return err;
    }

    char *json_response = (char *)context.buffer;

    cJSON *root = cJSON_Parse(json_response);
    if(root == NULL) {
        ESP_LOGE(TAG, "JSON parse error from location data");
        return ESP_ERR_INVALID_RESPONSE;
    }
    DEFER(cJSON_Delete(root));

    cJSON *status = cJSON_GetObjectItemCaseSensitive(root, "status");
    if(status == NULL) {
        ESP_LOGE(TAG, "No status");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if(!cJSON_IsString(status)) {
        ESP_LOGE(TAG, "Status not a string");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if(strcmp(status->valuestring, "success") != 0) {
        char const *message = "unknown reason";
        cJSON *msg = cJSON_GetObjectItemCaseSensitive(root, "message");
        if(msg && cJSON_IsString(msg)) {
            message = msg->valuestring;
        }
        ESP_LOGE(TAG, "Failed: %s (%s)", status->string, message);
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *lat_item = cJSON_GetObjectItemCaseSensitive(root, "lat");
    cJSON *lon_item = cJSON_GetObjectItemCaseSensitive(root, "lon");

    if(!(cJSON_IsNumber(lat_item) && cJSON_IsNumber(lon_item))) {
        ESP_LOGE(TAG, "lat/lon not both numbers!?");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // #define LOCATION_TESTING

#if defined(LOCATION_TESTING)
    ESP_LOGI(TAG, "!!!!!!!!!!!!! LOCATION TESTING !!!!!!!!!!!!!!!!");

    // double const sydney_lat = -33.978364;
    // double const sydney_lon = 151.164000;
    // current_lat = sydney_lat;
    // current_lon = sydney_lon;

    // double const tokyo_lat = 35.691048;
    // double const tokyo_lon = 139.781079;
    // current_lat = tokyo_lat;
    // current_lon = tokyo_lon;

    double const sao_paulo_lat = -23.671787;
    double const sao_paulo_lon = -46.666755;
    current_lat = sao_paulo_lat;
    current_lon = sao_paulo_lon;
    got_location = true;

#else
    current_lat = lat_item->valuedouble;
    current_lon = lon_item->valuedouble;
    got_location = true;
#endif

    ESP_LOGI(TAG, "Location: Lat=%.4f, Lon=%.4f", current_lat, current_lon);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t get_timezone_data()
{
    ESP_LOGI(TAG, "get_timezone_data");

    char url_buffer[256];
    sprintf(url_buffer, "http://api.timezonedb.com/v2.1/get-time-zone?key=%s&format=json&by=position&lat=%.4f&lng=%.4f", TIMEZONEDB_API_KEY,
            current_lat, current_lon);

    esp_err_t err = do_http_request(url_buffer, &context, 10, 3000);
    if(err != ESP_OK) {
        return err;
    }

    char *json_response = (char *)context.buffer;
    cJSON *root = cJSON_Parse(json_response);
    if(root == NULL) {
        ESP_LOGE(TAG, "Error parsing response: %s", json_response);
        return ESP_ERR_INVALID_RESPONSE;
    }
    DEFER(cJSON_Delete(root));

    cJSON *status = cJSON_GetObjectItemCaseSensitive(root, "status");
    if(!cJSON_IsString(status)) {
        ESP_LOGE(TAG, "Error parsing JSON from timezonedb");
        return ESP_ERR_INVALID_RESPONSE;
    }
    if(strcmp(status->valuestring, "OK") != 0) {
        ESP_LOGE(TAG, "Error getting timezone: status = %s", status->valuestring);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // TODO (chs): look at zoneEnd and check timezone again just after then

    cJSON *dst_item = cJSON_GetObjectItemCaseSensitive(root, "dst");
    int dst = cJSON_IsString(dst_item) ? atoi(dst_item->valuestring) : 0;

    cJSON *offset_item = cJSON_GetObjectItemCaseSensitive(root, "gmtOffset");
    if(!(cJSON_IsNumber(offset_item))) {
        ESP_LOGE(TAG, "Bad JSON!?");
        return ESP_ERR_INVALID_RESPONSE;
    }
    timezone_offset_seconds = offset_item->valueint;
    got_timezone = true;
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

void delay_until(long hour, long minute)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    struct tm timeinfo;
    localtime_r(&(tv_now.tv_sec), &timeinfo);

    long seconds_since_midnight = (timeinfo.tm_hour * 3600) + (timeinfo.tm_min * 60) + timeinfo.tm_sec;

    long delay_sec;

    long seconds = hour * 3600 + minute * 60;

    if(seconds_since_midnight < seconds - 1) {
        delay_sec = seconds - seconds_since_midnight;
    } else {
        long day_seconds = 24 * 3600;
        delay_sec = day_seconds - seconds_since_midnight + seconds;
    }
    if(delay_sec > 0) {
        delay_secs(delay_sec);
    }
}

//////////////////////////////////////////////////////////////////////

void sntp_callback(struct timeval *tv)
{
    got_sntp = true;
    ESP_LOGI(TAG, "SNTP IS UP!");
}

//////////////////////////////////////////////////////////////////////
// start this task AFTER provision_init()

void clock_time_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Initializing SNTP for time sync.");
    esp_sntp_set_time_sync_notification_cb(sntp_callback);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");

    // Sit in a loop
    // if not got location, try to get location
    // if got location and not got timezone, try to get timezone
    // if sntp not up and we haven't reset it for more than N seconds, reset it
    // wait 10 seconds
    // loop

    int sntp_reset = 0;

    while(true) {

        // wait for wifi to be connected
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdFALSE, pdFALSE, portMAX_DELAY);

        if(!got_firmware_version) {
            got_firmware_version = check_firmware_version();
        }

        if(!got_location) {
            get_location_data();
        }

        if(got_location && !got_timezone) {
            get_timezone_data();
        }

        ESP_LOGI(TAG, "SNTP%s synchronized", got_sntp ? "" : " NOT");

        if(!got_sntp) {
            if(--sntp_reset < 0) {
                ESP_LOGI(TAG, "RESET SNTP");
                esp_sntp_stop();
                esp_sntp_init();
                sntp_reset = 3;
            }
        }

        if(got_sntp && got_location && got_timezone && got_firmware_version) {
            ESP_LOGI(TAG, "Clock is good, waiting until 4am to do it all again...");
            delay_until(4, 0);
            got_timezone = false;
            got_firmware_version = false;
        } else {
            delay_secs(10);
        }
    }
}
