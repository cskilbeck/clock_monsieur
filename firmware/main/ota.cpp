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
#include "util.h"
#include "state.h"
#include "provisioning.h"

static const char *TAG = "CUSTOM_OTA";

#define OTA_RECV_BUF_SIZE 1024

/**
 * @brief Mark the current app as valid.
 * Call this after a successful boot and functionality check (e.g. WiFi connection)
 * to prevent rolling back to the previous version on the next reboot.
 * Only strictly necessary if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is set.
 */
void ota_mark_app_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if(esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if(ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "First boot of new firmware: Marking as valid to prevent rollback.");
            esp_ota_mark_app_valid_cancel_rollback();
        } else {
            ESP_LOGD(TAG, "App state is not pending verify (State: %d)", ota_state);
        }
    }
}

/**
 * @brief Helper to check version string from a URL (e.g., https://mysite.com/firmware/version.txt)
 * @param url The URL to the version text file
 * @param buffer Buffer to store the version string
 * @param buffer_len Length of the buffer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t get_latest_firmware_version(char *buffer, size_t buffer_len)
{
    esp_http_client_config_t config{};
    config.url = "https://clockmonsieur.com/firmware/latest.txt";
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 5000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if(content_length < 0) {
        ESP_LOGE(TAG, "HTTP client fetch headers failed");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int data_read = esp_http_client_read_response(client, buffer, buffer_len - 1);
    // trim trailing whitespace/newlines
    while(data_read > 0 && isspace(buffer[data_read - 1])) {
        data_read -= 1;
    }
    if(data_read >= 0) {
        buffer[data_read] = 0;    // Null terminate
        ESP_LOGI(TAG, "Remote Version: %s", buffer);
    } else {
        ESP_LOGE(TAG, "Failed to read response");
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    return err;
}

/**
 * @brief Performs the OTA update by downloading from URL and writing to the next partition
 * @param url The URL to the .bin file (e.g., https://mysite.com/firmware/app.bin)
 * @return esp_err_t
 */
esp_err_t do_ota_firmware_update(const char *latest)
{
    char url[96];
    sprintf(url, "https://clockmonsieur.com/firmware/%s.bin", latest);

    ESP_LOGI(TAG, "Starting OTA Update from: %s", url);

    esp_err_t err = ESP_OK;
    esp_http_client_handle_t client = NULL;
    const esp_partition_t *update_partition = NULL;
    esp_ota_handle_t update_handle = 0;
    char *upgrade_data_buf = NULL;
    bool image_header_was_checked = false;

    // 1. Configure HTTP Client with Cert Bundle
    esp_http_client_config_t config{};
    config.url = url;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.keep_alive_enable = true;
    config.timeout_ms = 10000;

    client = esp_http_client_init(&config);
    if(client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        return ESP_FAIL;
    }

    // 2. Open Connection
    err = esp_http_client_open(client, 0);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_fetch_headers(client);

    // 3. Identify Next Partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if(update_partition == NULL) {
        ESP_LOGE(TAG, "Passive OTA partition not found");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32, update_partition->subtype, update_partition->address);

    // 4. Allocate Buffer
    upgrade_data_buf = (char *)malloc(OTA_RECV_BUF_SIZE);
    if(upgrade_data_buf == NULL) {
        ESP_LOGE(TAG, "Couldn't allocate memory to upgrade data buffer");
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    int binary_file_len = 0;

    // 5. Prepare OTA (Begin)
    // We use OTA_SIZE_UNKNOWN because we might not trust Content-Length or it might be chunked
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        goto ota_end;
    }

    while(1) {
        int data_read = esp_http_client_read(client, upgrade_data_buf, OTA_RECV_BUF_SIZE);
        if(data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            err = ESP_FAIL;
            goto ota_end;
        } else if(data_read > 0) {
            // 6. Write Chunk to Flash
            err = esp_ota_write(update_handle, upgrade_data_buf, data_read);
            if(err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
                goto ota_end;
            }
            binary_file_len += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_len);
        } else if(data_read == 0) {
            // Check if connection is done (EOF or server closed)
            if(esp_http_client_is_complete_data_received(client)) {
                ESP_LOGI(TAG, "Connection closed, all data received");
                break;
            }
            // If data_read is 0 but not complete, we might need to loop again or handle timeout
            // For simplicity in blocking mode, 0 usually means closed by peer.
            break;
        }
    }

    // 7. End OTA
    err = esp_ota_end(update_handle);
    if(err != ESP_OK) {
        if(err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        goto ota_end;
    }

    // 8. Set Boot Partition
    err = esp_ota_set_boot_partition(update_partition);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        goto ota_end;
    }

    ESP_LOGI(TAG, "OTA Complete. Prepare to restart system!");
    delay_secs(1);
    esp_restart();

ota_end:
    esp_http_client_cleanup(client);
    free(upgrade_data_buf);
    return err;
}

bool check_firmware_version()
{
    char latest[16];
    get_latest_firmware_version(latest, sizeof(latest));
    ESP_LOGI(TAG, "FIRMWARE available: %s (currently %s)", latest, VERSION_STR);
    if(strcmp(latest, VERSION_STR) != 0) {
        state_set(ota_state);
        do_ota_firmware_update(latest);
    }
    return true;
}
