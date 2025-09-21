//////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "veml_3235.h"

//////////////////////////////////////////////////////////////////////

static const char *TAG = "VEML_3235";

#define VEML3235_I2C_ADDR 0x10
#define VEML3235_ALS_CONF 0x00
#define VEML3235_ALS_DATA 0x04

/************************************************************************
 * @brief Read the ambient light value from the VEML3235 sensor.
 * Performs a blocking I2C read operation.
 * @param i2c_port The I2C port number.
 * @param lux_value A pointer to a variable to store the 16-bit lux value.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */

esp_err_t veml3235_read_lux(i2c_port_t i2c_port, uint16_t *lux_value)
{
    esp_err_t err;
    static uint8_t data_buf[2];
    static i2c_cmd_handle_t cmd = nullptr;

    if(cmd == nullptr) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (VEML3235_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, VEML3235_ALS_DATA, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (VEML3235_I2C_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data_buf, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
    }

    err = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "VEML3235 transaction failed: %s", esp_err_to_name(err));
    } else {
        *lux_value = (data_buf[1] << 8) | data_buf[0];
        ESP_LOGI(TAG, "Read Lux value: %u", *lux_value);
    }

    return err;
}
