//////////////////////////////////////////////////////////////////////

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_partition.h"
#include "esp_crt_bundle.h"

#include "version.h"
#include "main.h"
#include "util.h"
#include "state.h"
#include "wifi.h"

//////////////////////////////////////////////////////////////////////

LOG_CONTEXT("ota");

#define OTA_RECV_BUF_SIZE 1024

#define LATEST_URL "https://clockmonsieur.com/fw/" VERSION_HW "/latest.txt";
#define FIRMWARE_URL_FORMAT_STR "https://clockmonsieur.com/fw/" VERSION_HW "/%s.bin"

//////////////////////////////////////////////////////////////////////
// Mark the current app as valid.
// Call this after a successful boot and functionality check (e.g. WiFi connection)
// to prevent rolling back to the previous version on the next reboot.
// Only necessary if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is set.

void ota_mark_app_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if(esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if(ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            LOG_INFO("First boot of new firmware: Marking as valid to prevent rollback.");
            esp_ota_mark_app_valid_cancel_rollback();
        } else {
            LOG_INFO("App state is not pending verify");
        }
    }
}

//////////////////////////////////////////////////////////////////////
// Helper to check version string from a URL (e.g., https://mysite.com/firmware/version.txt)
// url The URL to the version text file
// buffer Buffer to store the version string
// buffer_len Length of the buffer
// returns ESP_OK on success

esp_err_t get_latest_firmware_version(char *buffer, size_t buffer_len)
{
    esp_http_client_config_t config{};
    config.url = LATEST_URL;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 10000;
    config.max_authorization_retries = 10;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        LOG_ERROR("Failed to initialise HTTP connection");
        return ESP_FAIL;
    }
    DEFER(esp_http_client_cleanup(client));

    esp_err_t err = esp_http_client_open(client, 0);
    if(err != ESP_OK) {
        LOG_ERROR("Failed to open HTTP connection: %s", esp_err_to_name(err));
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if(content_length < 0) {
        LOG_ERROR("HTTP client fetch headers failed");
        return ESP_FAIL;
    }

    if(content_length > buffer_len - 1) {
        LOG_WARN("Content length (%" PRId64 ") > buffer_len (%zu) - will truncate", content_length, buffer_len);
    }

    int data_read = esp_http_client_read_response(client, buffer, buffer_len - 1);

    // trim trailing whitespace/newlines
    while(data_read > 0 && isspace(buffer[data_read - 1])) {
        data_read -= 1;
    }
    if(data_read < 0) {
        LOG_ERROR("Failed to read response");
        return ESP_FAIL;
    }
    buffer[data_read] = 0;    // Null terminate
    LOG_INFO("Remote Version: %s", buffer);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// Performs the OTA update by downloading from URL and writing to the next partition
// pass latest FW version to download (e.g. 1.0.2)

esp_err_t do_ota_firmware_update(const char *latest)
{
    char url[96];
    sprintf(url, FIRMWARE_URL_FORMAT_STR, latest);

    LOG_INFO("Starting OTA Update from: %s", url);

    // Configure HTTP Client with Cert Bundle
    esp_http_client_config_t config{};
    config.url = url;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.keep_alive_enable = true;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == nullptr) {
        LOG_ERROR("Failed to initialise HTTP connection!?");
        return ESP_FAIL;
    }
    DEFER(esp_http_client_cleanup(client));

    // Open Connection
    esp_err_t err = esp_http_client_open(client, 0);
    if(err != ESP_OK) {
        LOG_ERROR("Failed to open HTTP connection: %s", esp_err_to_name(err));
        return err;
    }

    // Get Headers
    int64_t content_length = esp_http_client_fetch_headers(client);
    if(content_length == ESP_FAIL) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    LOG_INFO("Content-length: %" PRId64 "%s", content_length, content_length == 0 ? " (chunked)" : "");

    // Identify Next Partition
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if(update_partition == nullptr) {
        LOG_ERROR("Passive OTA partition not found");
        return ESP_FAIL;
    }
    LOG_INFO("Writing to partition subtype %d at offset 0x%" PRIx32, update_partition->subtype, update_partition->address);

    // Allocate Buffer
    char *upgrade_data_buf = (char *)malloc(OTA_RECV_BUF_SIZE);
    if(upgrade_data_buf == NULL) {
        LOG_ERROR("Couldn't allocate memory to upgrade data buffer");
        return ESP_ERR_NO_MEM;
    }
    DEFER(free(upgrade_data_buf));

    // Prepare OTA (Begin)
    // Use OTA_SIZE_UNKNOWN because we don't trust Content-Length or it might be chunked
    int binary_file_len = 0;
    esp_ota_handle_t update_handle = 0;
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if(err != ESP_OK) {
        LOG_ERROR("esp_ota_begin failed (%s)", esp_err_to_name(err));
        return err;
    }

    // Download and write firmware to the other partition
    while(1) {
        int data_read = esp_http_client_read(client, upgrade_data_buf, OTA_RECV_BUF_SIZE);
        if(data_read < 0) {
            LOG_ERROR("Error: SSL data read error");
            return ESP_FAIL;
        } else if(data_read > 0) {
            // Write Chunk to Flash
            err = esp_ota_write(update_handle, upgrade_data_buf, data_read);
            if(err != ESP_OK) {
                LOG_ERROR("esp_ota_write failed (%s)", esp_err_to_name(err));
                return err;
            }
            binary_file_len += data_read;
            LOG_DEBUG("Written image length %d", binary_file_len);
        } else {
            // Check if connection is done (EOF or server closed)
            if(esp_http_client_is_complete_data_received(client)) {
                LOG_INFO("Connection closed, all data received");
                break;
            }
            LOG_ERROR("Connection closed before all data received? GOT %d bytes", binary_file_len);
            return ESP_FAIL;
        }
    }

    // End OTA
    err = esp_ota_end(update_handle);
    if(err != ESP_OK) {
        LOG_ERROR("esp_ota_end failed (%s)!", esp_err_to_name(err));
        return err;
    }

    // Set Boot Partition
    err = esp_ota_set_boot_partition(update_partition);
    if(err != ESP_OK) {
        LOG_ERROR("esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return err;
    }

    LOG_WARN("OTA Complete. Rebooting in 1 second....");
    delay_secs(1);
    esp_restart();
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

void ota_task(void *)
{
    while(true) {
        xEventGroupWaitBits(system_events, SYS_EVENT_NETWORK_CONNECTED, false, true, portMAX_DELAY);
        delay_secs(30);
        LOG_INFO("Checking firmware version");
        char latest[16];
        esp_err_t err = get_latest_firmware_version(latest, sizeof(latest));
        if(err == ESP_OK) {
            LOG_INFO("FIRMWARE available: %s (currently %s)", latest, VERSION_STR);
            if(strcmp(latest, VERSION_STR) != 0) {
                state_set(ota_state);
                err = do_ota_firmware_update(latest);
                if(err != ESP_OK) {
                    // OTA update failed, wait 30 seconds and try again?
                    continue;
                }
            } else {
                ota_mark_app_valid();
            }
        }
        // wait 24 hours before checking again
        int64_t one_day_seconds = 60 * 60 * 24;
        int64_t one_day_millis = one_day_seconds * 1000;
        int64_t ticks = one_day_millis / configTICK_RATE_HZ;
        vTaskDelay((TickType_t)ticks);
    }
}

//////////////////////////////////////////////////////////////////////

void ota_init()
{
    xTaskCreate(ota_task, "ota", 4096, nullptr, 1, nullptr);
}
