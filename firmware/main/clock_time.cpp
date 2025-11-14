#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "provisioning.h"
#include "time.h"
#include "util.h"

static const char *TAG = "clock_time";

#define MAX_HTTP_OUTPUT_BUFFER 512

#define IP_API_KEY "825b0130e3baced2cdd5eeb07a73953b"
#define TIMEZONEDB_API_KEY "VV0F65261GPD"


char *response_buffer;

// Global variables to store parsed data
double current_lat = 0.0;
double current_lon = 0.0;
char posix_tz_string[128] = "";

// Context structure to hold response data for a single request
typedef struct
{
    char *buffer;
    int len;
    int max_len;
} http_data_context_t;

// Event handler to capture HTTP response data chunk by chunk
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_data_context_t *ctx = (http_data_context_t *)evt->user_data;
    if(!ctx) {
        return ESP_FAIL;
    }

    switch(evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if(ctx->len + evt->data_len < ctx->max_len) {
            memcpy(ctx->buffer + ctx->len, evt->data, evt->data_len);
            ctx->len += evt->data_len;
        } else {
            ESP_LOGE(TAG, "Buffer overflow! Data truncated.");
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        if(ctx->buffer != NULL && ctx->len < ctx->max_len) {
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

/**
 * @brief Performs the request to ip-api.com to get lat/lon.
 */
esp_err_t get_location_data()
{
    http_data_context_t context = { .buffer = response_buffer, .len = 0, .max_len = MAX_HTTP_OUTPUT_BUFFER };

    // Request URL: http://ip-api.com/json/?fields=lat,lon
    esp_http_client_config_t config = {
        .url = "http://api.ipapi.com/api/check?access_key=" IP_API_KEY "&fields=ip,latitude,longitude",
        .event_handler = http_event_handler,
        .user_data = &context,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if(err == ESP_OK) {
        // Assume the response is stored in output_buffer (from http_event_handler)
        char *json_response = (char *)context.buffer;

        cJSON *root = cJSON_Parse(json_response);
        if(root == NULL) {
            ESP_LOGE(TAG, "JSON parse error from location data");
            ESP_LOGE(TAG, "%s", json_response);
        } else {
            cJSON *lat_item = cJSON_GetObjectItemCaseSensitive(root, "latitude");
            cJSON *lon_item = cJSON_GetObjectItemCaseSensitive(root, "longitude");

            if(!(cJSON_IsNumber(lat_item) && cJSON_IsNumber(lon_item))) {
                ESP_LOGE(TAG, "Error getting lat/lon");
            } else {
                current_lat = lat_item->valuedouble;
                current_lon = lon_item->valuedouble;
                ESP_LOGI(TAG, "Location: Lat=%.4f, Lon=%.4f", current_lat, current_lon);
            }
            cJSON_Delete(root);
        }
    }
    esp_http_client_cleanup(client);
    return err;
}

/**
 * @brief Performs the request to timezonedb.com to get the timezone offset.
 */
esp_err_t get_timezone_data()
{
    ESP_LOGI(TAG, "get_timezone_data");
    char url_buffer[256];
    // Construct the URL with lat/lon and your API key
    sprintf(url_buffer, "http://api.timezonedb.com/v2.1/get-time-zone?key=%s&format=json&by=position&lat=%.4f&lng=%.4f", TIMEZONEDB_API_KEY,
            current_lat, current_lon);
    ESP_LOGI(TAG, "%s", url_buffer);

    http_data_context_t context = { .buffer = response_buffer, .len = 0, .max_len = MAX_HTTP_OUTPUT_BUFFER };
    esp_http_client_config_t config = { .url = url_buffer, .event_handler = http_event_handler, .user_data = &context };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    DEFER(esp_http_client_cleanup(client));

    esp_err_t err = esp_http_client_perform(client);

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "http err: %s", esp_err_to_name(err));
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
    cJSON *offset_item = cJSON_GetObjectItemCaseSensitive(root, "gmtOffset");
    cJSON *zone_name_item = cJSON_GetObjectItemCaseSensitive(root, "zoneName");
    cJSON *abbreviation_item = cJSON_GetObjectItemCaseSensitive(root, "abbreviation");

    if(!(cJSON_IsNumber(offset_item) && cJSON_IsString(zone_name_item))) {
        ESP_LOGE(TAG, "Bad JSON!?");
        return ESP_ERR_INVALID_RESPONSE;
    }
    int gmtOffset_sec = offset_item->valueint;
    const char *zoneName = zone_name_item->valuestring;
    const char *abbr = cJSON_IsString(abbreviation_item) ? abbreviation_item->valuestring : "XXX";

    // The standard POSIX TZ format is "STDOFFSET[DST[DSOFFSET][,rule]]"
    // ESP-IDF uses a simpler format: setenv("TZ", <TZ_STRING>); tzset();
    // Example: "EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"

    // TimeZoneDB's response contains the zone name (e.g., "America/New_York"),
    // which is the simplest string to use for automatic TZ/DST setting.
    snprintf(posix_tz_string, sizeof(posix_tz_string), "%s", zoneName);

    ESP_LOGI(TAG, "Time Zone: %s (Offset: %d seconds)", zoneName, gmtOffset_sec);
    return ESP_OK;
}

/**
 * @brief Sets the timezone and starts the NTP service.
 */
void sntp_sync_with_timezone()
{
    if(strlen(posix_tz_string) > 0) {
        ESP_LOGI(TAG, "Setting Timezone to: %s", posix_tz_string);
        setenv("TZ", posix_tz_string, 1);
        tzset();
    } else {
        // Fallback to UTC if automatic detection fails
        ESP_LOGW(TAG, "No timezone set. Defaulting to UTC/GMT.");
        setenv("TZ", "GMT0", 1);
        tzset();
    }

    ESP_LOGI(TAG, "Initializing SNTP for time sync.");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();
}

bool is_time_synchronized()
{
    return sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
}

void clock_time_task(void *pvParameter)
{
    // wait for wifi to be connected
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

    // allocate HTTP response buffer
    response_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER);
    if(!response_buffer) {
        ESP_LOGE(TAG, "!?CAn't alloc HTTP response buffer");
        return;
    }
    DEFER(free(response_buffer));

    if(get_location_data() != ESP_OK) {
        return;
    }

    if(get_timezone_data() != ESP_OK) {
        return;
    }

    sntp_sync_with_timezone();
    // The task has completed its one-time operation, so we delete it.
    vTaskDelete(NULL);
}
