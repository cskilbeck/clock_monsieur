//////////////////////////////////////////////////////////////////////

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio_defs.h"
#include "lux.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////
// I2C Port and GPIO pins

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000    // 100 kHz

#define VEML3235_I2C_ADDR 0x10
#define VEML3235_ALS_CONF 0x00
#define VEML3235_W_DATA 0x04
#define VEML3235_ALS_DATA 0x05

#define I2C_TIMEOUT_MS 100

LOG_CONTEXT("lux");

namespace
{
    const char *TAG = "I2C_DRIVER";

    uint16_t brightness;

    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_master_dev_handle_t veml_i2c_handle;

    //////////////////////////////////////////////////////////////////////

    esp_err_t read_register(uint8_t reg_addr, uint16_t &value)
    {
        uint8_t reg[1] = { reg_addr };
        uint8_t data[2];

        ESP_CHECK(i2c_master_transmit_receive(veml_i2c_handle, reg, sizeof(reg), data, sizeof(data), I2C_TIMEOUT_MS));

        value = (data[1] << 8) | data[0];
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // Initialize the VEML3235 - write 2 config bytes to register 0x00

    esp_err_t veml3235_init(i2c_master_bus_handle_t bus)
    {
        i2c_master_dev_handle_t handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VEML3235_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        ESP_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &handle));

        // Register address + 2 data bytes
        uint8_t write_data[3] = { VEML3235_ALS_CONF, 0b0100000, 0b00000001 };

        ESP_CHECK(i2c_master_transmit(handle, write_data, sizeof(write_data), I2C_TIMEOUT_MS));
        veml_i2c_handle = handle;
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t veml3235_read_lux(uint16_t lux_value[2])
    {
        if(veml_i2c_handle == nullptr) {
            lux_value[0] = 64;
            lux_value[1] = 64;
            return ESP_OK;
        }
        ESP_CHECK(read_register(VEML3235_W_DATA, lux_value[0]));
        ESP_CHECK(read_register(VEML3235_ALS_DATA, lux_value[1]));
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    void i2c_task(void *pvParameters)
    {
        LOG_INFO("i2c_task begins");

        i2c_master_bus_config_t conf{};
        conf.sda_io_num = I2C_MASTER_SDA_GPIO;
        conf.scl_io_num = I2C_MASTER_SCL_GPIO;
        conf.clk_source = I2C_CLK_SRC_DEFAULT;
        conf.glitch_ignore_cnt = 7;
        conf.flags = { .enable_internal_pullup = true };
        ESP_LOG_ERR(i2c_new_master_bus(&conf, &i2c_bus_handle));

        ESP_LOG_ERR(veml3235_init(i2c_bus_handle));

        float lux = -1.0f;

        while(true) {

            uint16_t lux_value[2];
            if(veml3235_read_lux(lux_value) == ESP_OK) {
                brightness = lux_value[1];
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void lux_init()
{
    xTaskCreatePinnedToCore(i2c_task, "i2c_task", 4096, NULL, 5, NULL, 0);
}

//////////////////////////////////////////////////////////////////////

uint16_t lux_get()
{
    return brightness;
}