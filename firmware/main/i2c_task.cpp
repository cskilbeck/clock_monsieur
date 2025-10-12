//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "check_err.h"
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

static esp_err_t do_i2c_task(void *pvParameters)
{
    uint16_t lux_value[2];
    i2c_master_bus_config_t conf{};
    conf.sda_io_num = I2C_MASTER_SDA_GPIO;
    conf.scl_io_num = I2C_MASTER_SCL_GPIO;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.glitch_ignore_cnt = 7;
    conf.flags = { .enable_internal_pullup = true };
    CHECK_ERR(i2c_new_master_bus(&conf, &i2c_bus_handle));

    CHECK_ERR(veml3235_init(i2c_bus_handle));

    float lux = -1.0f;

    while(true) {

        uint16_t lux_value[2];
        if(veml3235_read_lux(lux_value) == ESP_OK) {
            float target = lux_value[1] / 32767.0f;
            if(lux < 0) {
                lux = (float)target;
            }
            lux += (target - lux) / 8.0f;

            // ghetto inverse gamma ramp
            float t = 2 * lux - lux * lux;

            // scale lux to 10..127
            int base = 10;
            int max = 127;
            int range = max - base;
            brightness = (uint8_t)(t * range) + base;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//////////////////////////////////////////////////////////////////////

static void i2c_task(void *pvParameters)
{
    (void)do_i2c_task(pvParameters);
}

//////////////////////////////////////////////////////////////////////

void i2c_task_start()
{
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);
}
