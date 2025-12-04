//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include "wifi.h"
#include "main.h"

static char const *TAG = "wifi";

//////////////////////////////////////////////////////////////////////

#define CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE 0
#define CONFIG_EXAMPLE_PROV_SEC2_PROD_MODE 1

#define PROV_QR_VERSION "v1"
#define PROV_TRANSPORT_BLE "ble"

// Name, subtype of partition containing security info
#define SEC_PARTITION_LABEL "prov_dat"
#define SEC_PARTITION_SUBTYPE ((esp_partition_subtype_t)0x40)

esp_err_t start_provisioning();

int wifi_retries = 0;

//////////////////////////////////////////////////////////////////////

#if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE

#define EXAMPLE_PROV_SEC2_USERNAME "wifiprov"
#define EXAMPLE_PROV_SEC2_PWD "abcd1234"

/* This salt,verifier has been generated for username = "wifiprov" and password = "abcd1234"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
static const char sec2_salt[] = { 0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4 };

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43, 0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59,
    0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24, 0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1,
    0x32, 0x24, 0xda, 0x11, 0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89, 0xaa, 0x16,
    0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e, 0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff,
    0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc, 0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35,
    0x6a, 0xc4, 0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e, 0xc3, 0xa5, 0xf9, 0x68,
    0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e, 0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29,
    0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9, 0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
    0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17, 0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44,
    0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23, 0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17,
    0xbd, 0x08, 0xb8, 0x96, 0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc, 0xa9, 0x38,
    0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2, 0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b,
    0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64, 0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21,
    0xe7, 0x2d, 0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84, 0x53, 0x4a, 0xb3, 0x0c,
    0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb, 0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84,
    0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0, 0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
    0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1, 0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8,
    0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
};

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
    ESP_LOGI(TAG, "Development mode: using hard coded salt");
    *salt = sec2_salt;
    *salt_len = sizeof(sec2_salt);
    return ESP_OK;
}

static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
    ESP_LOGI(TAG, "Development mode: using hard coded verifier");
    *verifier = sec2_verifier;
    *verifier_len = sizeof(sec2_verifier);
    return ESP_OK;
}

char const *provisioning_pop()
{
    return EXAMPLE_PROV_SEC2_PWD;
}

//////////////////////////////////////////////////////////////////////

#else

// PROD MODE - get sec_info from 'prov_dat' partition

//////////////////////////////////////////////////////////////////////
// NOTE: This struct must line up with gen_sec.py

struct sec_info_t
{
    uint8_t password_len;
    char password[15];
    char salt[16];
    char verifier[384];
};

static sec_info_t sec_info{};

//////////////////////////////////////////////////////////////////////

void dump_hex(char *data, size_t len)
{
    char const *sep = "";
    for(int i = 0; i < len; ++i) {
        printf("%s%02x", sep, data[i]);
        if(((i + 1) & 15) == 0) {
            sep = "\n";
        } else {
            sep = " ";
        }
    }
    printf("\n");
}

//////////////////////////////////////////////////////////////////////
// Load the sec_info_t from the 'prov_dat' partition
// This should have been written by gen_sec.py

esp_err_t read_security_info()
{
    char const *TAG = "sec_info";

    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, SEC_PARTITION_SUBTYPE, SEC_PARTITION_LABEL);

    if(!partition) {
        ESP_LOGE(TAG, "Partition '%s' not found!", SEC_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    if(sizeof(struct sec_info_t) > partition->size) {
        ESP_LOGE(TAG, "Struct size (%zu) exceeds partition size (%zu)!", sizeof(struct sec_info_t), partition->size);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = esp_partition_read(partition, 0, &sec_info, sizeof(struct sec_info_t));

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from partition (0x%x)!", err);
    }
#if defined(DEBUG)
    ESP_LOGI(TAG, "---> POP! %s (%d)", sec_info.password, sec_info.password_len);
    ESP_LOGI(TAG, "SALT:");
    dump_hex(sec_info.salt, sizeof(sec_info.salt));
    ESP_LOGI(TAG, "VERIFIER:");
    dump_hex(sec_info.verifier, sizeof(sec_info.verifier));
#endif
    return err;
}

//////////////////////////////////////////////////////////////////////

char const *provisioning_pop()
{
    return sec_info.password;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
    *salt = sec_info.salt;
    *salt_len = sizeof(sec_info.salt);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
    *verifier = sec_info.verifier;
    *verifier_len = sizeof(sec_info.verifier);
    return ESP_OK;
}

#endif    // CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE

//////////////////////////////////////////////////////////////////////
// Event handler for catching system events

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "EVENT: %s, ID = %d", event_base, event_id);

    if(event_base == WIFI_PROV_EVENT) {

        switch(event_id) {

        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;

        case WIFI_PROV_CRED_RECV: {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials:");
            ESP_LOGI(TAG, "    SSID     : %s", (const char *)wifi_sta_cfg->ssid);
            ESP_LOGI(TAG, "    Password : %s", (const char *)wifi_sta_cfg->password);
            break;
        }

        case WIFI_PROV_CRED_FAIL: {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!");
            ESP_LOGE(TAG, "Reason : %s", (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi auth failed" : "Wi-Fi access-point not found");
            ESP_LOGE(TAG, "Please reset to factory and retry provisioning");

            /* Reset the state machine on provisioning failure.
             * It allows the provisioning manager to retry the provisioning process
             * based on the number of attempts specified in wifi_conn_attempts. After attempting
             * the maximum number of retries, the provisioning manager will reset the state machine
             * and the provisioning process will be terminated.
             */
            wifi_prov_mgr_reset_sm_state_on_failure();
            break;
        }

        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            xEventGroupSetBits(system_events, SYS_EVENT_PROVISIONING_DONE);
            break;

        case WIFI_PROV_END:
            /* De-initialize manager once provisioning is finished */
            xEventGroupClearBits(system_events, SYS_EVENT_PROVISIONING | SYS_EVENT_BLE_CONNECTED | SYS_EVENT_PROVISIONING_IN_PROGRESS);
            wifi_prov_mgr_deinit();
            break;

        default:
            break;
        }
    } else if(event_base == WIFI_EVENT) {
        switch(event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_retries += 1;
            ESP_LOGI(TAG, "Disconnected");
            xEventGroupClearBits(system_events, SYS_EVENT_SNTP_SYNCHRONIZED | SYS_EVENT_NETWORK_CONNECTED | SYS_EVENT_WIFI_CONNECTED);
            if(wifi_retries < 10) {
                ESP_LOGI(TAG, "Connecting to the AP again for %d time", wifi_retries);
                esp_wifi_connect();
            } else {
                ESP_LOGI(TAG, "Wifi ain't happening - let's try provisioning");
                wifi_prov_mgr_reset_sm_state_for_reprovision();
            }
            break;
        default:
            break;
        }
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupClearBits(system_events, SYS_EVENT_PROVISIONING);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "======================= Connected with IP Address:" IPSTR " =======================", IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(system_events, SYS_EVENT_WIFI_CONNECTED);
    } else if(event_base == PROTOCOMM_TRANSPORT_BLE_EVENT) {
        switch(event_id) {
        case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
            xEventGroupSetBits(system_events, SYS_EVENT_BLE_CONNECTED);
            ESP_LOGI(TAG, "BLE transport: Connected!");
            break;
        case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
            xEventGroupClearBits(system_events, SYS_EVENT_BLE_CONNECTED | SYS_EVENT_PROVISIONING_IN_PROGRESS);
            ESP_LOGI(TAG, "BLE transport: Disconnected!");
            break;
        default:
            break;
        }
    } else if(event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch(event_id) {
        case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
            ESP_LOGI(TAG, "Secured session established!");
            xEventGroupSetBits(system_events, SYS_EVENT_PROVISIONING_IN_PROGRESS);
            break;
        case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
            ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
            xEventGroupSetBits(system_events, SYS_EVENT_PROVISIONING_ERROR);
            break;
        case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
            ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
            xEventGroupSetBits(system_events, SYS_EVENT_PROVISIONING_ERROR);
            break;
        default:
            break;
        }
    }
}

//////////////////////////////////////////////////////////////////////

static void get_device_service_name(char *service_name, size_t max)
{
    snprintf(service_name, max, "Clock Monsieur");
}

//////////////////////////////////////////////////////////////////////

void wifi_prov_app_callback(void *user_data, wifi_prov_cb_event_t event, void *event_data)
{
    /**
     * This is blocking callback, any configurations that needs to be set when a particular
     * provisioning event is triggered can be set here.
     */
    switch(event) {
    case WIFI_PROV_SET_STA_CONFIG: {
        /**
         * Wi-Fi configurations can be set here before the Wi-Fi is enabled in
         * STA mode.
         */
        wifi_config_t *wifi_config = (wifi_config_t *)event_data;
        (void)wifi_config;
        break;
    }
    default:
        break;
    }
}

//////////////////////////////////////////////////////////////////////

esp_err_t start_provisioning()
{
    ESP_LOGI(TAG, "Starting provisioning");
    xEventGroupSetBits(system_events, SYS_EVENT_PROVISIONING);

    /* What is the Device Service Name that we want
     * This translates to :
     *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
     *     - device name when scheme is wifi_prov_scheme_ble
     */
    char service_name[16];
    get_device_service_name(service_name, sizeof(service_name));

    wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
    /* The username must be the same one, which has been used in the generation of salt and verifier */

    /* This is the structure for passing security parameters
     * for the protocomm security 2.
     * If dynamically allocated, sec2_params pointer and its content
     * must be valid till WIFI_PROV_END event is triggered.
     */
    wifi_prov_security2_params_t sec2_params = {};

    ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
    ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

    wifi_prov_security2_params_t *sec_params = &sec2_params;
    /* What is the service key (could be NULL)
     * This translates to :
     *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
     *          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
     *     - simply ignored when scheme is wifi_prov_scheme_ble
     */
    const char *service_key = NULL;
    /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
     * set a custom 128 bit UUID which will be included in the BLE advertisement
     * and will correspond to the primary GATT service that provides provisioning
     * endpoints as GATT characteristics. Each GATT characteristic will be
     * formed using the primary service UUID as base, with different auto assigned
     * 12th and 13th bytes (assume counting starts from 0th byte). The client side
     * applications must identify the endpoints by reading the User Characteristic
     * Description descriptor (0x2901) for each characteristic, which contains the
     * endpoint name of the characteristic */
    uint8_t custom_service_uuid[] = {
        /* LSB <---------------------------------------
         * ---------------------------------------> MSB */
        0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
    };

    /* If your build fails with linker errors at this point, then you may have
     * forgotten to enable the BT stack or BTDM BLE settings in the SDK (e.g. see
     * the sdkconfig.defaults in the example project) */
    wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

    /* Do not stop and de-init provisioning even after success, so that we can restart it later. */
    ESP_ERROR_CHECK(wifi_prov_mgr_disable_auto_stop(1000));

    /* Start provisioning service */
    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)sec_params, service_name, service_key));
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

const wifi_prov_event_handler_t wifi_prov_event_handler = {
    .event_cb = wifi_prov_app_callback,
    .user_data = NULL,
};

//////////////////////////////////////////////////////////////////////

esp_err_t wifi_init()
{
#if CONFIG_EXAMPLE_PROV_SEC2_PROD_MODE
    ESP_ERROR_CHECK(read_security_info());
#endif

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Configuration for the provisioning manager */
    wifi_prov_mgr_config_t config{};
    config.wifi_prov_conn_cfg = { .wifi_conn_attempts = 3 };
    config.scheme = wifi_prov_scheme_ble;
    config.app_event_handler = wifi_prov_event_handler;
    config.scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM;

    /* Get Wi-Fi Station configuration */
    wifi_config_t wifi_cfg;
    if(esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK && strlen((const char *)wifi_cfg.sta.ssid) != 0) {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

        /* Start Wi-Fi station */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    } else {
        /* Initialize provisioning manager with the configuration parameters set above */
        ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
        ESP_ERROR_CHECK(start_provisioning());
    }
    return ESP_OK;
}
