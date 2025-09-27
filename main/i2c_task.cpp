//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio_defs.h"
#include "i2c_task.h"
#include "veml_3235.h"
#include "mcp_4017.h"

//////////////////////////////////////////////////////////////////////
// I2C Port and GPIO pins

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000    // 100 kHz

static const char *TAG = "I2C_DRIVER";

i2c_master_bus_handle_t i2c_bus_handle = NULL;

//////////////////////////////////////////////////////////////////////

static esp_err_t i2c_master_init(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed_hz)
{
    i2c_master_bus_config_t conf = {
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &i2c_bus_handle));
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static void i2c_task(void *pvParameters)
{
    uint16_t lux_value[2];
    ESP_ERROR_CHECK(i2c_master_init(I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ));
    ESP_ERROR_CHECK(veml3235_init(i2c_bus_handle));
    ESP_ERROR_CHECK(mcp4017_init(i2c_bus_handle));

    float illum = 0.0f;
    while(true) {

        if(veml3235_read_lux(lux_value) == ESP_OK) {
            if(lux_value[0] > 32767) {
                lux_value[0] = 32767;
            }
            if(lux_value[1] > 32767) {
                lux_value[1] = 32767;
            }
            illum = illum * 0.9f + lux_value[1] / 16384.0f;
            uint8_t wiper_value = 127 - (illum / 20.0f * 127);
            mcp4017_set_wiper(0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//////////////////////////////////////////////////////////////////////

void i2c_task_start()
{
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);
}
