//////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_task.h"
#include "veml_3235.h"

//////////////////////////////////////////////////////////////////////

static const char *TAG = "VEML_3235";

#define VEML3235_I2C_ADDR 0x10
#define VEML3235_ALS_CONF 0x00
#define VEML3235_W_DATA 0x04
#define VEML3235_ALS_DATA 0x05

#define I2C_TIMEOUT_MS 100

static i2c_master_dev_handle_t veml_i2c_handle;

//////////////////////////////////////////////////////////////////////

static esp_err_t read_register(uint8_t reg_addr, uint16_t &value)
{
    uint8_t reg[1] = { reg_addr };
    uint8_t data[2];

    esp_err_t err = i2c_master_transmit_receive(veml_i2c_handle, reg, sizeof(reg), data, sizeof(data), I2C_TIMEOUT_MS);

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "VEML3235 transaction failed: %s", esp_err_to_name(err));
    } else {
        value = (data[1] << 8) | data[0];
    }
    return err;
}

//////////////////////////////////////////////////////////////////////

// Initialize the VEML3235 - write 2 config bytes to register 0x00
esp_err_t veml3235_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML3235_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &veml_i2c_handle));

    // Register address + 2 data bytes
    uint8_t write_data[3] = { VEML3235_ALS_CONF, 0b0100000, 0b00000001 };

    return i2c_master_transmit(veml_i2c_handle, write_data, sizeof(write_data), I2C_TIMEOUT_MS);
}

//////////////////////////////////////////////////////////////////////

esp_err_t veml3235_read_lux(uint16_t lux_value[2])
{
    esp_err_t err = read_register(VEML3235_W_DATA, lux_value[0]);
    if(err != ESP_OK) {
        return err;
    }
    return read_register(VEML3235_ALS_DATA, lux_value[1]);
}
