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
#include "util.h"


namespace
{
    int timezone_offset_seconds = 0;
}

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

double current_lat = 0.0;
double current_lon = 0.0;
char posix_tz_string[128] = "";

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

//////////////////////////////////////////////////////////////////////
// request to ip-api.com to get lat/lon.

esp_err_t get_location_data()
{
    context.reset();

    esp_http_client_config_t config = {
        .url = "http://ip-api.com/json",
        .event_handler = http_event_handler,
        .user_data = &context,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_perform(client);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP Error %d (%s)", err, esp_err_to_name(err));
        return err;
    }
    DEFER(esp_http_client_cleanup(client));

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

#else
    current_lat = lat_item->valuedouble;
    current_lon = lon_item->valuedouble;
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
    ESP_LOGI(TAG, "%s", url_buffer);

    context.reset();

    esp_http_client_config_t config = { .url = url_buffer, .event_handler = http_event_handler, .user_data = &context };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    DEFER(esp_http_client_cleanup(client));

    esp_err_t err = esp_http_client_perform(client);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP Error %d (%s)", err, esp_err_to_name(err));
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
    // cJSON *zone_name_item = cJSON_GetObjectItemCaseSensitive(root, "zoneName");
    // cJSON *abbreviation_item = cJSON_GetObjectItemCaseSensitive(root, "abbreviation");
    // cJSON *next_abbreviation_item = cJSON_GetObjectItemCaseSensitive(root, "nextAbbreviation");
    // const char *abbr = cJSON_IsString(abbreviation_item) ? abbreviation_item->valuestring : "GMT";
    // const char *nxt_abbr = cJSON_IsString(next_abbreviation_item) ? next_abbreviation_item->valuestring : "GMT";

    cJSON *dst_item = cJSON_GetObjectItemCaseSensitive(root, "dst");
    int dst = cJSON_IsString(dst_item) ? atoi(dst_item->valuestring) : 0;

    cJSON *offset_item = cJSON_GetObjectItemCaseSensitive(root, "gmtOffset");
    if(!(cJSON_IsNumber(offset_item))) {
        ESP_LOGE(TAG, "Bad JSON!?");
        return ESP_ERR_INVALID_RESPONSE;
    }
    timezone_offset_seconds = offset_item->valueint;

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

bool is_time_synchronized()
{
    return sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
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
    TickType_t delay_ticks = (delay_sec * 1000) / portTICK_PERIOD_MS;

    if(delay_ticks > 0) {
        vTaskDelay(delay_ticks);
    }
}

//////////////////////////////////////////////////////////////////////
// start this task AFTER provision_init()

// 1. wait for wifi to be connected
// 2. get/set timezone
// 3. start sntp
// 4. wait until 4am
// 5. repeat

void clock_time_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Initializing SNTP for time sync.");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();

    while(true) {

        // wait for wifi to be connected
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdFALSE, pdFALSE, portMAX_DELAY);

        // get timezone from current location
        if(get_location_data() == ESP_OK) {
            get_timezone_data();
        }

        // wait one minute
        vTaskDelay(pdMS_TO_TICKS(60000));

        // if sntp is up, wait until 4am to do it again, else try again straight away?
        if(is_time_synchronized()) {
            delay_until(4, 0);
        }
    }
}
