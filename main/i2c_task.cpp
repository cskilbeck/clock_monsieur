//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio_defs.h"
#include "i2c_task.h"
#include "veml_3235.h"

//////////////////////////////////////////////////////////////////////
// I2C Port and GPIO pins

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000    // 100 kHz

static const char *TAG = "I2C_DRIVER";

uint8_t brightness;

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
    veml3235_init(i2c_bus_handle);

    float target = 0;

    while(true) {

        if(veml3235_read_lux(lux_value) == ESP_OK) {
            if(lux_value[0] > 32767) {
                lux_value[0] = 32767;
            }
            if(lux_value[1] > 32767) {
                lux_value[1] = 32767;
            }
            if(target == 0) {
                target = (float)lux_value[1];
            } else {
                float diff = (lux_value[1] - target) / 8;
                target += diff;
            }
            int b = (int)target >> 8;
            if(b < 2) {
                b = 2;
            }
            if(b > 127) {
                b = 127;
            }
            brightness = (uint8_t)b;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//////////////////////////////////////////////////////////////////////

void i2c_task_start()
{
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);
}
