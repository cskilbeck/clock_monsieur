#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "provisioning.h"
#include "time.h"

static const char *TAG = "clock_time";

#define MAX_HTTP_OUTPUT_BUFFER 512

#define TIMEZONEDB_API_KEY "825b0130e3baced2cdd5eeb07a73953b"

// Global variables to store parsed data
double current_lat = 0.0;
double current_lon = 0.0;
char posix_tz_string[128] = "";

// Event handler to capture HTTP response data chunk by chunk
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;    // Buffer to store response of first request
    static int output_len;         // Stores total length of response

    switch(evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        // Append data to the output buffer
        if(output_buffer == NULL) {
            output_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER);
            output_len = 0;
        }
        if(output_len + evt->data_len < MAX_HTTP_OUTPUT_BUFFER) {
            memcpy(output_buffer + output_len, evt->data, evt->data_len);
            output_len += evt->data_len;
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        // Null-terminate the buffer on finish
        if(output_buffer != NULL) {
            output_buffer[output_len] = '\0';
            ESP_LOGI(TAG, "Total Response Size: %d", output_len);
        }
        break;
    case HTTP_EVENT_DISCONNECTED:
        // Clean up buffer if disconnect occurs before finish
        if(output_buffer != NULL) {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
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
    // Request URL: http://ip-api.com/json/?fields=lat,lon
    esp_http_client_config_t config = {
        .url = "http://ip-api.com/json/?fields=lat,lon",
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if(err == ESP_OK) {
        // Assume the response is stored in output_buffer (from http_event_handler)
        char *json_response = (char *)config.user_data;

        cJSON *root = cJSON_Parse(json_response);
        if(root != NULL) {
            cJSON *lat_item = cJSON_GetObjectItemCaseSensitive(root, "lat");
            cJSON *lon_item = cJSON_GetObjectItemCaseSensitive(root, "lon");

            if(cJSON_IsNumber(lat_item) && cJSON_IsNumber(lon_item)) {
                current_lat = lat_item->valuedouble;
                current_lon = lon_item->valuedouble;
                ESP_LOGI(TAG, "Location: Lat=%.4f, Lon=%.4f", current_lat, current_lon);
            }
            cJSON_Delete(root);
        }
    }

    // Cleanup and return
    esp_http_client_cleanup(client);
    // You'd need to manually free the output buffer here after using it
    // For simplicity, this step is omitted but required in production

    return err;
}

/**
 * @brief Performs the request to timezonedb.com to get the timezone offset.
 */
esp_err_t get_timezone_data()
{
    char url_buffer[256];
    // Construct the URL with lat/lon and your API key
    sprintf(url_buffer, "http://api.timezonedb.com/v2.1/get-time-zone?key=%s&format=json&by=position&lat=%.4f&lng=%.4f", TIMEZONEDB_API_KEY,
            current_lat, current_lon);

    esp_http_client_config_t config = {
        .url = url_buffer,
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if(err == ESP_OK) {
        // Assume the response is stored in output_buffer (from http_event_handler)
        char *json_response = (char *)config.user_data;

        cJSON *root = cJSON_Parse(json_response);
        if(root != NULL) {
            cJSON *status = cJSON_GetObjectItemCaseSensitive(root, "status");
            if(cJSON_IsString(status) && strcmp(status->valuestring, "OK") == 0) {

                cJSON *offset_item = cJSON_GetObjectItemCaseSensitive(root, "gmtOffset");
                cJSON *zone_name_item = cJSON_GetObjectItemCaseSensitive(root, "zoneName");
                cJSON *abbreviation_item = cJSON_GetObjectItemCaseSensitive(root, "abbreviation");

                if(cJSON_IsNumber(offset_item) && cJSON_IsString(zone_name_item)) {
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
                }
            }
            cJSON_Delete(root);
        }
    }

    // Cleanup and return
    esp_http_client_cleanup(client);
    // You'd need to manually free the output buffer here after using it

    return err;
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

    if(get_location_data() == ESP_OK) {
        if(get_timezone_data() == ESP_OK) {
            sntp_sync_with_timezone();
        }
    }
    // The task has completed its one-time operation, so we delete it.
    vTaskDelete(NULL);
}
