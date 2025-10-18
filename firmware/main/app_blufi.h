#ifndef APP_BLUFI_H
#define APP_BLUFI_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Defines the events reported back to the application via the callback.
 */
typedef enum
{
    BLUFI_EVENT_INIT_DONE = 0,       ///< BLUFI stack has initialized and is ready to accept connections.
    BLUFI_EVENT_RECV_CREDENTIALS,    ///< Received Wi-Fi SSID and Password from the BLE client.
    BLUFI_EVENT_CONNECTING,          ///< ESP32 is attempting to connect to the received Wi-Fi AP.
    BLUFI_EVENT_CONNECTED,           ///< Successfully connected to the Wi-Fi AP and received an IP.
    BLUFI_EVENT_FAIL,                ///< Failed to connect to the Wi-Fi AP.
    BLUFI_EVENT_DEINIT_DONE,         ///< BLUFI has been deinitialized (usually after a successful connection).
    BLUFI_EVENT_UNKNOWN              ///< An unknown event occurred.
} app_blufi_event_t;

/**
 * @brief Type definition for the application callback function.
 *
 * The main application should implement a function matching this signature
 * to receive status updates for display/LED control.
 *
 * @param event The BLUFI event that occurred.
 */
typedef void (*app_blufi_callback_t)(app_blufi_event_t event);

/**
 * @brief Initializes the Wi-Fi and Bluetooth stacks and starts the BLUFI provisioning service.
 *
 * This function should be called once from the main application to start the BLUFI process.
 *
 * @param cb The callback function to notify the application of status changes.
 */
esp_err_t app_blufi_init(app_blufi_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif    // APP_BLUFI_H
