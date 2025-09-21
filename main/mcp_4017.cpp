//////////////////////////////////////////////////////////////////////

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mcp_4017.h"

//////////////////////////////////////////////////////////////////////

#define MCP4017_I2C_ADDR 0x2F

static char const *TAG = "MCP4017";

/************************************************************************
 * @brief Set the wiper position of the MCP4017 digital potentiometer.
 * Performs a blocking I2C write operation
 * @param i2c_port The I2C port number.
 * @param wiper_value The 8-bit value (0-255) for the wiper position.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */

esp_err_t mcp4017_set_wiper(i2c_port_t i2c_port, uint8_t wiper_value)
{
    esp_err_t err;

    static i2c_cmd_handle_t cmd = nullptr;

    if(cmd == nullptr) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MCP4017_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, wiper_value, true);
        i2c_master_stop(cmd);
    }

    err = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "MCP4017 transaction failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Set MCP4017 wiper to %u", wiper_value);
    }
    return err;
}
