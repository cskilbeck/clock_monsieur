//////////////////////////////////////////////////////////////////////

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_task.h"
#include "mcp_4017.h"

//////////////////////////////////////////////////////////////////////

#define MCP4017_I2C_ADDR 0x2f

// static char const *TAG = "MCP4017";

static i2c_master_dev_handle_t mcp_i2c_handle;

//////////////////////////////////////////////////////////////////////

esp_err_t mcp4017_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP4017_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mcp_i2c_handle));
    return ESP_OK;
}

esp_err_t mcp4017_set_wiper(uint8_t wiper_value)
{
    uint8_t write_data[1] = { wiper_value };
    return i2c_master_transmit(mcp_i2c_handle, write_data, sizeof(write_data), I2C_TIMEOUT_MS);
}
