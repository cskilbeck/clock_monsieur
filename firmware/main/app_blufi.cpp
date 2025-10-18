//////////////////////////////////////////////////////////////////////

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_blufi.h"
#include "esp_crc.h"
#include "esp_random.h"
#include "esp_blufi_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"

#include "util.h"
#include "app_blufi.h"

//////////////////////////////////////////////////////////////////////

LOG_CONTEXT("blufi");

//////////////////////////////////////////////////////////////////////

#define SEC_TYPE_DH_PARAM_LEN 0x00
#define SEC_TYPE_DH_PARAM_DATA 0x01
#define SEC_TYPE_DH_P 0x02
#define SEC_TYPE_DH_G 0x03
#define SEC_TYPE_DH_PUBLIC 0x04

#define EXAMPLE_INVALID_REASON 255
#define EXAMPLE_INVALID_RSSI -128

#define DH_SELF_PUB_KEY_LEN 128
#define SHARE_KEY_LEN 128
#define PSK_LEN 16

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

const int CONNECTED_BIT = BIT0;

//////////////////////////////////////////////////////////////////////

struct blufi_security
{
    uint8_t self_public_key[DH_SELF_PUB_KEY_LEN];
    uint8_t share_key[SHARE_KEY_LEN];
    size_t share_len;
    uint8_t psk[PSK_LEN];
    uint8_t *dh_param;
    int dh_param_len;
    uint8_t iv[16];
    mbedtls_dhm_context dhm;
    mbedtls_aes_context aes;
};

//////////////////////////////////////////////////////////////////////

static struct blufi_security *blufi_sec;

static wifi_config_t sta_config;
static wifi_config_t ap_config;
static EventGroupHandle_t wifi_event_group;
static uint8_t example_wifi_retry = 0;
static bool gl_sta_connected = false;
static bool gl_sta_got_ip = false;
static bool ble_is_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;
static wifi_sta_list_t gl_sta_list;
static bool gl_sta_is_connecting = false;
static esp_blufi_extra_info_t gl_sta_conn_info;

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_host_init(void)
{
    int ret;
    ret = esp_bluedroid_init();
    if(ret) {
        LOG_ERROR("%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ret = esp_bluedroid_enable();
    if(ret) {
        LOG_ERROR("%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    LOG_INFO("BD ADDR: " ESP_BD_ADDR_STR ": ", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_host_deinit(void)
{
    int ret;
    ret = esp_blufi_profile_deinit();
    if(ret != ESP_OK) {
        return ret;
    }

    ret = esp_bluedroid_disable();
    if(ret) {
        LOG_ERROR("%s deinit bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ret = esp_bluedroid_deinit();
    if(ret) {
        LOG_ERROR("%s deinit bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_gap_register_callback(void)
{
    int rc;
    rc = esp_ble_gap_register_callback(esp_blufi_gap_event_handler);
    if(rc) {
        return rc;
    }
    return esp_blufi_profile_init();
}

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_host_and_cb_init(esp_blufi_callbacks_t *example_callbacks)
{
    esp_err_t ret = ESP_OK;

    ret = esp_blufi_host_init();
    if(ret) {
        LOG_ERROR("%s initialise host failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_blufi_register_callbacks(example_callbacks);
    if(ret) {
        LOG_ERROR("%s blufi register failed, error code = %x", __func__, ret);
        return ret;
    }

    ret = esp_blufi_gap_register_callback();
    if(ret) {
        LOG_ERROR("%s gap register failed, error code = %x", __func__, ret);
        return ret;
    }

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_controller_init()
{
    esp_err_t ret = ESP_OK;
#if CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if(ret) {
        LOG_ERROR("%s initialize bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret) {
        LOG_ERROR("%s enable bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ret;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t esp_blufi_controller_deinit()
{
    esp_err_t ret = ESP_OK;
    ret = esp_bt_controller_disable();
    if(ret) {
        LOG_ERROR("%s disable bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_deinit();
    if(ret) {
        LOG_ERROR("%s deinit bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

//////////////////////////////////////////////////////////////////////

static int myrand(void *rng_state, unsigned char *output, size_t len)
{
    esp_fill_random(output, len);
    return (0);
}

//////////////////////////////////////////////////////////////////////

static void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free)
{
    if(data == NULL || len < 3) {
        LOG_ERROR("BLUFI Invalid data format");
        esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
        return;
    }

    int ret;
    uint8_t type = data[0];

    if(blufi_sec == NULL) {
        LOG_ERROR("BLUFI Security is not initialized");
        esp_blufi_send_error_info(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    switch(type) {
    case SEC_TYPE_DH_PARAM_LEN:
        blufi_sec->dh_param_len = ((data[1] << 8) | data[2]);
        if(blufi_sec->dh_param) {
            free(blufi_sec->dh_param);
            blufi_sec->dh_param = NULL;
        }
        blufi_sec->dh_param = (uint8_t *)malloc(blufi_sec->dh_param_len);
        if(blufi_sec->dh_param == NULL) {
            blufi_sec->dh_param_len = 0; /* Reset length to avoid using unallocated memory */
            esp_blufi_send_error_info(ESP_BLUFI_DH_MALLOC_ERROR);
            LOG_ERROR("%s, malloc failed", __func__);
            return;
        }
        break;
    case SEC_TYPE_DH_PARAM_DATA: {
        if(blufi_sec->dh_param == NULL) {
            LOG_ERROR("%s, blufi_sec->dh_param == NULL", __func__);
            esp_blufi_send_error_info(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        if(len < (blufi_sec->dh_param_len + 1)) {
            LOG_ERROR("%s, invalid dh param len", __func__);
            esp_blufi_send_error_info(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        uint8_t *param = blufi_sec->dh_param;
        memcpy(blufi_sec->dh_param, &data[1], blufi_sec->dh_param_len);
        ret = mbedtls_dhm_read_params(&blufi_sec->dhm, &param, &param[blufi_sec->dh_param_len]);
        if(ret) {
            LOG_ERROR("%s read param failed %d", __func__, ret);
            esp_blufi_send_error_info(ESP_BLUFI_READ_PARAM_ERROR);
            return;
        }
        free(blufi_sec->dh_param);
        blufi_sec->dh_param = NULL;

        const int dhm_len = mbedtls_dhm_get_len(&blufi_sec->dhm);

        if(dhm_len > DH_SELF_PUB_KEY_LEN) {
            LOG_ERROR("%s dhm len not support %d", __func__, dhm_len);
            esp_blufi_send_error_info(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        ret = mbedtls_dhm_make_public(&blufi_sec->dhm, dhm_len, blufi_sec->self_public_key, DH_SELF_PUB_KEY_LEN, myrand, NULL);
        if(ret) {
            LOG_ERROR("%s make public failed %d", __func__, ret);
            esp_blufi_send_error_info(ESP_BLUFI_MAKE_PUBLIC_ERROR);
            return;
        }

        ret = mbedtls_dhm_calc_secret(&blufi_sec->dhm, blufi_sec->share_key, SHARE_KEY_LEN, &blufi_sec->share_len, myrand, NULL);
        if(ret) {
            LOG_ERROR("%s mbedtls_dhm_calc_secret failed %d", __func__, ret);
            esp_blufi_send_error_info(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        ret = mbedtls_md5(blufi_sec->share_key, blufi_sec->share_len, blufi_sec->psk);

        if(ret) {
            LOG_ERROR("%s mbedtls_md5 failed %d", __func__, ret);
            esp_blufi_send_error_info(ESP_BLUFI_CALC_MD5_ERROR);
            return;
        }

        mbedtls_aes_setkey_enc(&blufi_sec->aes, blufi_sec->psk, PSK_LEN * 8);

        /* alloc output data */
        *output_data = &blufi_sec->self_public_key[0];
        *output_len = dhm_len;
        *need_free = false;

    } break;
    case SEC_TYPE_DH_P:
        break;
    case SEC_TYPE_DH_G:
        break;
    case SEC_TYPE_DH_PUBLIC:
        break;
    }
}

//////////////////////////////////////////////////////////////////////

static int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    if(!blufi_sec) {
        return -1;
    }

    memcpy(iv0, blufi_sec->iv, sizeof(blufi_sec->iv));
    iv0[0] = iv8; /* set iv8 as the iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufi_sec->aes, MBEDTLS_AES_ENCRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if(ret) {
        return -1;
    }

    return crypt_len;
}

//////////////////////////////////////////////////////////////////////

static int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    if(!blufi_sec) {
        return -1;
    }

    memcpy(iv0, blufi_sec->iv, sizeof(blufi_sec->iv));
    iv0[0] = iv8; /* set iv8 as the iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufi_sec->aes, MBEDTLS_AES_DECRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if(ret) {
        return -1;
    }

    return crypt_len;
}

//////////////////////////////////////////////////////////////////////

static uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len)
{
    /* This iv8 ignore, not used */
    return esp_crc16_be(0, data, len);
}

//////////////////////////////////////////////////////////////////////

static esp_err_t blufi_security_init(void)
{
    blufi_sec = (struct blufi_security *)malloc(sizeof(struct blufi_security));
    if(blufi_sec == NULL) {
        return ESP_FAIL;
    }

    memset(blufi_sec, 0x0, sizeof(struct blufi_security));

    mbedtls_dhm_init(&blufi_sec->dhm);
    mbedtls_aes_init(&blufi_sec->aes);

    memset(blufi_sec->iv, 0x0, sizeof(blufi_sec->iv));
    return 0;
}

//////////////////////////////////////////////////////////////////////

static void blufi_security_deinit(void)
{
    if(blufi_sec == NULL) {
        return;
    }
    if(blufi_sec->dh_param) {
        free(blufi_sec->dh_param);
        blufi_sec->dh_param = NULL;
    }
    mbedtls_dhm_free(&blufi_sec->dhm);
    mbedtls_aes_free(&blufi_sec->aes);

    memset(blufi_sec, 0x0, sizeof(struct blufi_security));

    free(blufi_sec);
    blufi_sec = NULL;
}

//////////////////////////////////////////////////////////////////////

static void example_record_wifi_conn_info(int rssi, uint8_t reason)
{
    memset(&gl_sta_conn_info, 0, sizeof(esp_blufi_extra_info_t));
    if(gl_sta_is_connecting) {
        gl_sta_conn_info.sta_max_conn_retry_set = true;
        gl_sta_conn_info.sta_max_conn_retry = CONFIG_CLOCK_WIFI_CONNECTION_MAXIMUM_RETRY;
    } else {
        gl_sta_conn_info.sta_conn_rssi_set = true;
        gl_sta_conn_info.sta_conn_rssi = rssi;
        gl_sta_conn_info.sta_conn_end_reason_set = true;
        gl_sta_conn_info.sta_conn_end_reason = reason;
    }
}

//////////////////////////////////////////////////////////////////////

static void example_wifi_connect(void)
{
    example_wifi_retry = 0;
    gl_sta_is_connecting = (esp_wifi_connect() == ESP_OK);
    example_record_wifi_conn_info(EXAMPLE_INVALID_RSSI, EXAMPLE_INVALID_REASON);
}

//////////////////////////////////////////////////////////////////////

static bool example_wifi_reconnect(void)
{
    bool ret;
    if(gl_sta_is_connecting && example_wifi_retry++ < CONFIG_CLOCK_WIFI_CONNECTION_MAXIMUM_RETRY) {
        LOG_INFO("BLUFI WiFi starts reconnection");
        gl_sta_is_connecting = (esp_wifi_connect() == ESP_OK);
        example_record_wifi_conn_info(EXAMPLE_INVALID_RSSI, EXAMPLE_INVALID_REASON);
        ret = true;
    } else {
        ret = false;
    }
    return ret;
}

//////////////////////////////////////////////////////////////////////

static int softap_get_current_connection_number(void)
{
    esp_err_t ret;
    ret = esp_wifi_ap_get_sta_list(&gl_sta_list);
    if(ret == ESP_OK) {
        return gl_sta_list.num;
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_mode_t mode;

    switch(event_id) {
    case IP_EVENT_STA_GOT_IP: {
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_wifi_get_mode(&mode);

        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(info.sta_bssid, gl_sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = gl_sta_ssid;
        info.sta_ssid_len = gl_sta_ssid_len;
        gl_sta_got_ip = true;
        if(ble_is_connected == true) {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, softap_get_current_connection_number(), &info);
        } else {
            LOG_INFO("BLUFI BLE is not connected yet");
        }
        break;
    }
    default:
        break;
    }
    return;
}

//////////////////////////////////////////////////////////////////////

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_connected_t *event;
    wifi_event_sta_disconnected_t *disconnected_event;
    wifi_mode_t mode;

    switch(event_id) {

    case WIFI_EVENT_STA_START:
        example_wifi_connect();
        break;

    case WIFI_EVENT_STA_CONNECTED:
        gl_sta_connected = true;
        gl_sta_is_connecting = false;
        event = (wifi_event_sta_connected_t *)event_data;
        memcpy(gl_sta_bssid, event->bssid, 6);
        memcpy(gl_sta_ssid, event->ssid, event->ssid_len);
        gl_sta_ssid_len = event->ssid_len;
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        /* Only handle reconnection during connecting */
        if(gl_sta_connected == false && example_wifi_reconnect() == false) {
            gl_sta_is_connecting = false;
            disconnected_event = (wifi_event_sta_disconnected_t *)event_data;
            example_record_wifi_conn_info(disconnected_event->rssi, disconnected_event->reason);
        }
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        gl_sta_connected = false;
        gl_sta_got_ip = false;
        memset(gl_sta_ssid, 0, 32);
        memset(gl_sta_bssid, 0, 6);
        gl_sta_ssid_len = 0;
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;

    case WIFI_EVENT_AP_START:
        esp_wifi_get_mode(&mode);

        /* TODO: get config or information of softap, then set to report extra_info */
        if(ble_is_connected == true) {
            if(gl_sta_connected) {
                esp_blufi_extra_info_t info;
                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                memcpy(info.sta_bssid, gl_sta_bssid, 6);
                info.sta_bssid_set = true;
                info.sta_ssid = gl_sta_ssid;
                info.sta_ssid_len = gl_sta_ssid_len;
                esp_blufi_send_wifi_conn_report(mode, gl_sta_got_ip ? ESP_BLUFI_STA_CONN_SUCCESS : ESP_BLUFI_STA_NO_IP,
                                                softap_get_current_connection_number(), &info);
            } else if(gl_sta_is_connecting) {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONNECTING, softap_get_current_connection_number(), &gl_sta_conn_info);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_get_current_connection_number(), &gl_sta_conn_info);
            }
        } else {
            LOG_INFO("BLUFI BLE is not connected yet");
        }
        break;

    case WIFI_EVENT_SCAN_DONE: {
        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);
        if(apCount == 0) {
            LOG_INFO("Nothing AP found");
            break;
        }
        wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
        if(!ap_list) {
            LOG_ERROR("malloc error, ap_list is NULL");
            esp_wifi_clear_ap_list();
            break;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
        esp_blufi_ap_record_t *blufi_ap_list = (esp_blufi_ap_record_t *)malloc(apCount * sizeof(esp_blufi_ap_record_t));
        if(!blufi_ap_list) {
            if(ap_list) {
                free(ap_list);
            }
            LOG_ERROR("malloc error, blufi_ap_list is NULL");
            break;
        }
        for(int i = 0; i < apCount; ++i) {
            blufi_ap_list[i].rssi = ap_list[i].rssi;
            memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
        }

        if(ble_is_connected == true) {
            esp_blufi_send_wifi_list(apCount, blufi_ap_list);
        } else {
            LOG_INFO("BLUFI BLE is not connected yet");
        }

        esp_wifi_scan_stop();
        free(ap_list);
        free(blufi_ap_list);
        break;
    }

    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        // LOG_INFO("station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        // LOG_INFO("station " MACSTR " leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
        break;
    }

    default:
        break;
    }
    return;
}

//////////////////////////////////////////////////////////////////////

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    example_record_wifi_conn_info(EXAMPLE_INVALID_RSSI, EXAMPLE_INVALID_REASON);
    ESP_ERROR_CHECK(esp_wifi_start());
}

//////////////////////////////////////////////////////////////////////

static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    /* actually, should post to blufi_task handle the procedure,
     * now, as a example, we do it more simply */
    switch(event) {

    case ESP_BLUFI_EVENT_INIT_FINISH:
        LOG_INFO("BLUFI init finish");
        esp_blufi_adv_start();
        break;

    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        LOG_INFO("BLUFI deinit finish");
        break;

    case ESP_BLUFI_EVENT_BLE_CONNECT:
        LOG_INFO("BLUFI ble connect");
        ble_is_connected = true;
        esp_blufi_adv_stop();
        blufi_security_init();
        break;

    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        LOG_INFO("BLUFI ble disconnect");
        ble_is_connected = false;
        blufi_security_deinit();
        esp_blufi_adv_start();
        break;

    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        LOG_INFO("BLUFI Set WIFI opmode %d", param->wifi_mode.op_mode);
        ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
        break;

    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        LOG_INFO("BLUFI request wifi connect to AP");
        /* there is no wifi callback when the device has already connected to this wifi
        so disconnect wifi before connection. */
        esp_wifi_disconnect();
        example_wifi_connect();
        break;

    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        LOG_INFO("BLUFI request wifi disconnect from AP");
        esp_wifi_disconnect();
        break;

    case ESP_BLUFI_EVENT_REPORT_ERROR:
        LOG_ERROR("BLUFI report error, error code %d", param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;

    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        wifi_mode_t mode;
        esp_blufi_extra_info_t info;

        esp_wifi_get_mode(&mode);

        if(gl_sta_connected) {
            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            esp_blufi_send_wifi_conn_report(mode, gl_sta_got_ip ? ESP_BLUFI_STA_CONN_SUCCESS : ESP_BLUFI_STA_NO_IP,
                                            softap_get_current_connection_number(), &info);
        } else if(gl_sta_is_connecting) {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONNECTING, softap_get_current_connection_number(), &gl_sta_conn_info);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_get_current_connection_number(), &gl_sta_conn_info);
        }
        LOG_INFO("BLUFI get wifi status from AP");
        break;
    }

    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        LOG_INFO("blufi close a gatt connection");
        esp_blufi_disconnect();
        break;

    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;

    case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        LOG_INFO("Recv STA BSSID %s", sta_config.sta.ssid);
        break;

    case ESP_BLUFI_EVENT_RECV_STA_SSID:
        if(param->sta_ssid.ssid_len >= sizeof(sta_config.sta.ssid) / sizeof(sta_config.sta.ssid[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            LOG_INFO("Invalid STA SSID");
            break;
        }
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        LOG_INFO("Recv STA SSID %s", sta_config.sta.ssid);
        break;

    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        if(param->sta_passwd.passwd_len >= sizeof(sta_config.sta.password) / sizeof(sta_config.sta.password[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            LOG_INFO("Invalid STA PASSWORD");
            break;
        }
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
        sta_config.sta.threshold.authmode = EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        LOG_INFO("Recv STA PASSWORD %s", sta_config.sta.password);
        break;

    case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        if(param->softap_ssid.ssid_len >= sizeof(ap_config.ap.ssid) / sizeof(ap_config.ap.ssid[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            LOG_INFO("Invalid SOFTAP SSID");
            break;
        }
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
        ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        LOG_INFO("Recv SOFTAP SSID %s, ssid len %d", ap_config.ap.ssid, ap_config.ap.ssid_len);
        break;

    case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        if(param->softap_passwd.passwd_len >= sizeof(ap_config.ap.password) / sizeof(ap_config.ap.password[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            LOG_INFO("Invalid SOFTAP PASSWD");
            break;
        }
        strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
        ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        LOG_INFO("Recv SOFTAP PASSWORD %s len = %d", ap_config.ap.password, param->softap_passwd.passwd_len);
        break;

    case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if(param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        LOG_INFO("Recv SOFTAP MAX CONN NUM %d", ap_config.ap.max_connection);
        break;

    case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if(param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        LOG_INFO("Recv SOFTAP AUTH MODE %d", ap_config.ap.authmode);
        break;

    case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if(param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        LOG_INFO("Recv SOFTAP CHANNEL %d", ap_config.ap.channel);
        break;

    case ESP_BLUFI_EVENT_GET_WIFI_LIST: {
        wifi_scan_config_t scanConf = { .ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = false };
        esp_err_t ret = esp_wifi_scan_start(&scanConf, true);
        if(ret != ESP_OK) {
            esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
        }
        break;
    }

    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        LOG_INFO("Recv Custom Data %" PRIu32 "", param->custom_data.data_len);
        ESP_LOG_BUFFER_HEX("Custom Data", param->custom_data.data, param->custom_data.data_len);
        break;

    case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;

    case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;

    case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;

    case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;

    case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;

    case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}

//////////////////////////////////////////////////////////////////////

static esp_blufi_callbacks_t example_callbacks = {
    .event_cb = example_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

//////////////////////////////////////////////////////////////////////

esp_err_t app_blufi_init()
{
    initialise_wifi();
    esp_err_t ret = esp_blufi_controller_init();
    if(ret) {
        LOG_ERROR("%s BLUFI controller init failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_blufi_host_and_cb_init(&example_callbacks);
    if(ret) {
        LOG_ERROR("%s initialise failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    LOG_INFO("BLUFI VERSION %04x", esp_blufi_get_version());
    return ESP_OK;
}