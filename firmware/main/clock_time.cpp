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
#include "http.h"
#include "wifi.h"
#include "ping.h"
#include "time.h"
#include "ota.h"
#include "settings.h"
#include "timezone.h"
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

#define MAX_HTTP_OUTPUT_BUFFER 512

//////////////////////////////////////////////////////////////////////

namespace
{
    double current_lat = 0.0;
    double current_lon = 0.0;

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

    MAYBE_UNUSED bool json_int64(cJSON const *item, char const *name, int64_t &value)
    {
        double v;
        if(!json_double(item, name, v)) {
            return false;
        }
        value = (int64_t)v;
        return true;
    }

    char http_buffer[MAX_HTTP_OUTPUT_BUFFER];
    http_data_context_t context(http_buffer, sizeof(http_buffer));


}    // namespace

int timezone_offset_seconds = 0;         // timezone offset for right now
int timezone_next_offset_seconds = 0;    // timezone offset for 1 second in the future

timeval utc_wall_time;    // current UTC time

//////////////////////////////////////////////////////////////////////
// request to ip-api.com to get lat/lon.

esp_err_t get_location()
{
    LOG_INFO("Get location for timezone");

    esp_err_t err = do_http_request("http://ip-api.com/json", &context, 10, 5000);
    if(err != ESP_OK) {
        LOG_ERROR("Can't get location!?");
        return err;
    }

    char *json_response = (char *)context.buffer;

    cJSON *root = cJSON_Parse(json_response);
    if(root == nullptr) {
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

    char const *location = json_string(root, "timezone");
    if(location == nullptr) {
        LOG_ERROR("No timezone in ip-api response");
    } else {
        timezone_set(location);
    }

    double lat;
    double lon;
    if(!(json_double(root, "lat", lat) && json_double(root, "lon", lon))) {
        LOG_ERROR("lat/lon not numbers");
        return ESP_ERR_INVALID_RESPONSE;
    }
    current_lat = lat;
    current_lon = lon;

    xEventGroupSetBits(system_events, SYS_EVENT_GOT_LOCATION);
    xEventGroupClearBits(system_events, SYS_EVENT_NEED_LOCATION);

    LOG_INFO("Location: Lat=%.4f, Lon=%.4f", current_lat, current_lon);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// Get location and timezone

void timezone_task(void *)
{
    LOG_CONTEXT("timezone");

    while(true) {

        EventBits_t bits =
            xEventGroupWaitBits(system_events, SYS_EVENT_NETWORK_CONNECTED | SYS_EVENT_NEED_LOCATION, false, true, portMAX_DELAY);

        esp_err_t err = get_location();

        if(err != ESP_OK) {
            LOG_INFO("Will retry location in 10 seconds");
            delay_secs(10);
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

        // wait for SNTP to get up or someone to ask for timezone auto
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
    LOG_INFO("init");

    xTaskCreatePinnedToCore(sntp_task, "sntp", 1024 * 4, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(timezone_task, "timezone", 1024 * 4, NULL, 1, NULL, 0);

    if(settings.timezone_mode == timezone_mode_t::Auto) {
        xEventGroupSetBits(system_events, SYS_EVENT_NEED_LOCATION);
    } else {
        LOG_INFO("Setting timezone to %s", settings.location.name);
        timezone_set(settings.location.name);
    }
}

//////////////////////////////////////////////////////////////////////

void clock_update()
{
    gettimeofday(&utc_wall_time, NULL);
    timezone_offset_seconds = timezone_get_offset_seconds(utc_wall_time.tv_sec);
}
