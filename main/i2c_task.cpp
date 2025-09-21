//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "veml_3235.h"
#include "mcp_4017.h"

//////////////////////////////////////////////////////////////////////
// I2C Port and GPIO pins

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_GPIO GPIO_NUM_40
#define I2C_MASTER_SCL_GPIO GPIO_NUM_39
#define I2C_MASTER_FREQ_HZ 100000    // 100 kHz

static const char *TAG = "I2C_DRIVER";

//////////////////////////////////////////////////////////////////////

/**
 * @brief I2C master initialization function.
 * @param port The I2C port number to use.
 * @param sda_io_num The GPIO number for the SDA pin.
 * @param scl_io_num The GPIO number for the SCL pin.
 * @param clk_speed_hz The I2C clock speed in Hz.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */

static esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed_hz)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.scl_io_num = scl_io_num;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_speed_hz;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;    // Use default clock source

    esp_err_t err = i2c_param_config(port, &conf);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(port, conf.mode, 0, 0, 0);    // No buffer for master mode
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully on port %d", port);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

/**
 * @brief Asynchronous I2C communication task.
 * @param void *pvParameters
 * This task continuously reads from the VEML3235 and adjusts the MCP4017
 * based on the light level, demonstrating asynchronous, non-blocking behavior.
 */

static void i2c_task(void *pvParameters)
{
    uint16_t lux_value;
    uint8_t wiper_value = 0;
    while(true) {
        // read ambient light
        if(veml3235_read_lux(I2C_MASTER_PORT, &lux_value) == ESP_OK) {
            wiper_value = (uint8_t)std::min(255, (int)(lux_value / 4));
            mcp4017_set_wiper(I2C_MASTER_PORT, wiper_value);
        }
        // TODO: make it fade smoothly to target value
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//////////////////////////////////////////////////////////////////////

void i2c_task_start()
{
    ESP_ERROR_CHECK(i2c_master_init(I2C_MASTER_PORT, I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ));
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);
}
