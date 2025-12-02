//////////////////////////////////////////////////////////////////////

#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_sntp.h"

#include "main.h"
#include "wifi.h"
#include "ping.h"
#include "time.h"
#include "ota.h"
#include "util.h"

LOG_CONTEXT("clock");

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

// http://api.timezonedb.com
// {
//     "status": "OK",
//     "message": "",
//     "countryCode": "GB",
//     "countryName": "United Kingdom",
//     "regionName": "England",
//     "cityName": "Twickenham",
//     "zoneName": "Europe\/London",
//     "abbreviation": "GMT",
//     "gmtOffset": 0,
//     "dst": "0",
//     "zoneStart": 1761440400,
//     "zoneEnd": 1774746000,
//     "nextAbbreviation": "BST",
//     "timestamp": 1764269050,
//     "formatted": "2025-11-27 18:44:10"
// }

#define MAX_HTTP_OUTPUT_BUFFER 512

#define TIMEZONEDB_API_KEY "VV0F65261GPD"

//////////////////////////////////////////////////////////////////////

namespace
{
    int timezone_offset_seconds = 0;
    bool got_location{ false };
    bool got_timezone{ false };
    bool got_firmware_version{ false };

    double current_lat = 0.0;
    double current_lon = 0.0;
    char const *timezone_name = "unknown tz";
    time_t dst_transition_epoch_seconds = 0;

    char const *month[] = { "January", "February", "March",     "April",   "May",      "June",
                            "July",    "August",   "September", "October", "November", "December" };

    char const *weekday[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

    char const *json_string(cJSON const *item, char const *name, char const *default_value = "")
    {
        char const *value = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(item, name));
        return value == nullptr ? default_value : value;
    }

    bool json_double(cJSON const *item, char const *name, double &value)
    {
        cJSON *i = cJSON_GetObjectItemCaseSensitive(item, name);
        if(!cJSON_IsNumber(i)) {
            return false;
        }
        value = cJSON_GetNumberValue(i);
        return true;
    }

    bool json_int64(cJSON const *item, char const *name, int64_t &value)
    {
        double v;
        if(!json_double(item, name, v)) {
            return false;
        }
        value = (int64_t)v;
        return true;
    }

}    // namespace

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

//////////////////////////////////////////////////////////////////////
// request to ip-api.com to get lat/lon.

esp_err_t get_location()
{
    if(got_location) {
        return ESP_OK;
    }
    esp_err_t err = do_http_request("http://ip-api.com/json", &context, 10, 1000);
    if(err != ESP_OK) {
        LOG_ERROR("Can't get location!?");
        return err;
    }

    char *json_response = (char *)context.buffer;

    cJSON *root = cJSON_Parse(json_response);
    if(root == NULL) {
        LOG_ERROR("JSON parse error from location data");
        return ESP_ERR_INVALID_RESPONSE;
    }
    DEFER(cJSON_Delete(root));

    char const *status = json_string(root, "status", "no status?");
    if(strcmp(status, "success") != 0) {
        char const *message = json_string(root, "message", "no message?");
        LOG_ERROR("Failed: %s (%s)", status, message);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // #define LOCATION_TESTING

#if defined(LOCATION_TESTING)
    LOG_INFO("!!!!!!!!!!!!! LOCATION TESTING !!!!!!!!!!!!!!!!");

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
    double lat;
    double lon;
    if(!(json_double(root, "lat", lat) && json_double(root, "lon", lon))) {
        LOG_ERROR("lat/lon not numbers");
        return ESP_ERR_INVALID_RESPONSE;
    }
    current_lat = lat;
    current_lon = lon;
    got_location = true;
#endif
    xEventGroupSetBits(system_events, SYS_EVENT_GOT_LOCATION);
    LOG_INFO("Location: Lat=%.4f, Lon=%.4f", current_lat, current_lon);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t get_timezone()
{
    if(got_timezone) {
        return ESP_OK;
    }
    if(!got_location) {
        return ESP_FAIL;
    }
    LOG_INFO("get_timezone_data");

    char url_buffer[160];
    sprintf(url_buffer, "http://api.timezonedb.com/v2.1/get-time-zone?key=%s&format=json&by=position&lat=%.4f&lng=%.4f", TIMEZONEDB_API_KEY,
            current_lat, current_lon);

    esp_err_t err = do_http_request(url_buffer, &context, 10, 3000);
    if(err != ESP_OK) {
        return err;
    }

    char *json_response = (char *)context.buffer;

    LOG_INFO("TIMEZONE HTTP RESPONSE:\n%s", json_response);

    cJSON *root = cJSON_Parse(json_response);
    if(root == NULL) {
        LOG_ERROR("Error parsing response: %s", json_response);
        return ESP_ERR_INVALID_RESPONSE;
    }
    DEFER(cJSON_Delete(root));

    char const *status = json_string(root, "status");
    if(strcmp(status, "OK") != 0) {
        LOG_ERROR("Error getting timezone: status = %s", status);
        return ESP_ERR_INVALID_RESPONSE;
    }

    char const *dst_str = json_string(root, "dst", "");
    bool dst{ false };
    switch(dst_str[0]) {
    case '0':
        break;
    case '1':
        dst = true;
        break;
    case 0:
        LOG_WARN("No DST");
        break;
    default:
        LOG_WARN("Invalid DST: %s", dst_str);
        break;
    }

    int64_t offset_seconds{ 0 };
    if(!json_int64(root, "gmtOffset", offset_seconds)) {
        LOG_WARN("No gmtOffset");
    }

    time_t zone_end{ 0 };
    if(json_int64(root, "zoneEnd", zone_end)) {
        dst_transition_epoch_seconds = zone_end;
    } else {
        LOG_WARN("No Zone End!");
        dst_transition_epoch_seconds = 0;
    }

    char const *abbreviation = json_string(root, "abbreviation", "???");
    char const *timezone_name = json_string(root, "zoneName", "Unknown TZ?");

    struct tm *tm = gmtime(&zone_end);

    int hour = tm->tm_hour;
    int min = tm->tm_min;
    int mday = tm->tm_mday;
    int year = tm->tm_year + 1900;
    char const *day = weekday[tm->tm_wday];
    char const *mnth = month[tm->tm_mon];

    LOG_CONTEXT("timezone");
    LOG_INFO("%s", abbreviation);
    LOG_INFO("%s", timezone_name);
    LOG_INFO("Offset %d secs", offset_seconds);
    LOG_INFO("DST %s", dst ? "Active" : "Not active");
    LOG_INFO("Next zone at %02d:%02d, %s %d/%s/%d (%lld epoch seconds)", hour, min, day, mday, mnth, year, zone_end);

    timezone_offset_seconds = offset_seconds;
    got_timezone = true;
    xEventGroupSetBits(system_events, SYS_EVENT_GOT_TIMEZONE);

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// Get location and timezone

void timezone_task(void *)
{
    LOG_CONTEXT("timezone");

    while(!(got_location && got_timezone)) {

        LOG_INFO("Wait for network");

        // wait for network to be up
        xEventGroupWaitBits(system_events, SYS_EVENT_NETWORK_CONNECTED, false, true, portMAX_DELAY);

        delay_secs(1);

        get_location();
        get_timezone();

        if(!got_location || !got_timezone) {
            LOG_INFO("Will retry location/timezone in 10 seconds");
            delay_secs(9);
        } else {
            // wait for sntp to be up
            xEventGroupWaitBits(system_events, SYS_EVENT_SNTP_SYNCHRONIZED, false, true, portMAX_DELAY);

            // wait until daylight savings transition
            if(dst_transition_epoch_seconds == 0) {
                LOG_INFO("No daylight savings in this timezone, we're done!");
                vTaskDelete(nullptr);
            }
            while(true) {
                time_t now = time(nullptr);
                int64_t time_until_dst_change = dst_transition_epoch_seconds - now;
                int seconds = 60 * 60;
                if(time_until_dst_change < 60 * 70) {
                    LOG_INFO("Here comes DST!");
                    seconds = time_until_dst_change + 1;
                } else {
                    LOG_INFO("%lld seconds until DST change", time_until_dst_change);
                }
                LOG_INFO("Delaying for %d seconds", seconds);
                delay_secs(seconds);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////

void sntp_callback(struct timeval *tv)
{
    LOG_INFO("SNTP IS UP!");
    xEventGroupSetBits(system_events, SYS_EVENT_SNTP_SYNCHRONIZED);
}

//////////////////////////////////////////////////////////////////////
// Wait for SNTP - if it gets stuck (>60 seconds), reset and wait again

void sntp_task(void *)
{
    while(do_ping_check() != ESP_OK) {
        delay_secs(10);
    }

    xTaskCreatePinnedToCore(timezone_task, "timezone", 1024 * 6, NULL, 1, NULL, 0);

    int constexpr SNTP_WAIT_SECS = 60;
    int constexpr SNTP_WAIT_TICKS = pdMS_TO_TICKS(SNTP_WAIT_SECS * 1000);

    delay_secs(3);

    LOG_INFO("Init SNTP");
    esp_sntp_set_time_sync_notification_cb(sntp_callback);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.google.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();

    while(true) {

        LOG_INFO("Wait for SNTP...");

        // wait for SNTP to get up
        EventBits_t sntp_up = xEventGroupWaitBits(system_events, SYS_EVENT_SNTP_SYNCHRONIZED, false, false, SNTP_WAIT_TICKS);

        if((sntp_up & SYS_EVENT_SNTP_SYNCHRONIZED) == 0) {
            LOG_INFO("RESET SNTP");
            esp_sntp_stop();
            esp_sntp_init();
        } else {
            vTaskDelete(NULL);
        }
    }
}

//////////////////////////////////////////////////////////////////////

void clock_init()
{
    xTaskCreatePinnedToCore(sntp_task, "sntp", 1024 * 4, NULL, 1, NULL, 0);
}
