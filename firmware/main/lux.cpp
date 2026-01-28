//////////////////////////////////////////////////////////////////////

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio_defs.h"
#include "console.h"
#include "state.h"
#include "button.h"
#include "display.h"
#include "lux.h"
#include "settings.h"
#include "util.h"
#include <algorithm>

//////////////////////////////////////////////////////////////////////
// I2C Port and GPIO pins

#define I2C_TIMEOUT_MS 100

#define VEML3235_DEVICE 0
#define LTR303_DEVICE 1
#define ALS_PT243_DEVICE 2

#define USE_DEVICE ALS_PT243_DEVICE

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000    // 100 kHz

#define VEML3235_I2C_ADDR 0x10
#define VEML3235_ALS_CONF 0x00
#define VEML3235_W_DATA 0x04
#define VEML3235_ALS_DATA 0x05

#define LTR303_I2C_ADDR 0x29
#define LTR303_CONF_ADDR 0x80
#define LTR303_RATE_ADDR 0x85
#define LTR303_DATA_CH1 0x88
#define LTR303_DATA_CH0 0x8A

#define LTR303_SETUP_CONF 0b00011101
#define LTR303_SETUP_RATE 0b00011011

#if USE_DEVICE == VEML3235_DEVICE
#define INIT_DEVICE veml3235_init
#define READ_LUX veml3235_read_lux
#elif USE_DEVICE == LTR303_DEVICE
#define INIT_DEVICE ltr303_init
#define READ_LUX ltr303_read_lux
#elif USE_DEVICE == ALS_PT243_DEVICE
#define INIT_DEVICE als_pt243_init
#define READ_LUX als_pt243_read_lux
#else
#error "What ambient sensor device to use?"
#endif

LOG_CONTEXT("lux");

namespace
{
    const char *TAG = "I2C_DRIVER";

    uint16_t brightness;

    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    i2c_master_dev_handle_t i2c_device_handle;

    //////////////////////////////////////////////////////////////////////

    [[maybe_unused]] esp_err_t read_register(uint8_t reg_addr, uint16_t &value)
    {
        uint8_t reg[1] = { reg_addr };
        uint8_t data[2];

        ESP_CHECK(i2c_master_transmit_receive(i2c_device_handle, reg, sizeof(reg), data, sizeof(data), I2C_TIMEOUT_MS));

        value = (data[1] << 8) | data[0];
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    [[maybe_unused]] esp_err_t write_register(uint8_t reg_addr, uint8_t value)
    {
        uint8_t reg[2] = { reg_addr, value };
        return i2c_master_transmit(i2c_device_handle, reg, sizeof(reg), I2C_TIMEOUT_MS);
    }

    //////////////////////////////////////////////////////////////////////
    // Initialize the VEML3235 - write 2 config bytes to register 0x00

#if USE_DEVICE == VEML3235_DEVICE
    esp_err_t veml3235_init()
    {
        i2c_master_bus_config_t conf{};
        conf.sda_io_num = I2C_MASTER_SDA_GPIO;
        conf.scl_io_num = I2C_MASTER_SCL_GPIO;
        conf.clk_source = I2C_CLK_SRC_DEFAULT;
        conf.glitch_ignore_cnt = 7;
        conf.flags = { .enable_internal_pullup = true };
        ESP_LOG_ERR(i2c_new_master_bus(&conf, &i2c_bus_handle));

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
        i2c_device_handle = handle;
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t veml3235_read_lux(uint16_t lux_value[2])
    {
        if(i2c_device_handle == nullptr) {
            lux_value[0] = 64;
            lux_value[1] = 64;
            return ESP_OK;
        }
        ESP_CHECK(read_register(VEML3235_W_DATA, lux_value[0]));
        ESP_CHECK(read_register(VEML3235_ALS_DATA, lux_value[1]));
        return ESP_OK;
    }
#elif USE_DEVICE == LTR303_DEVICE
    //////////////////////////////////////////////////////////////////////
    // Initialize the LTR303

    esp_err_t ltr303_init()
    {
        i2c_master_bus_config_t conf{};
        conf.sda_io_num = I2C_MASTER_SDA_GPIO;
        conf.scl_io_num = I2C_MASTER_SCL_GPIO;
        conf.clk_source = I2C_CLK_SRC_DEFAULT;
        conf.glitch_ignore_cnt = 7;
        conf.flags = { .enable_internal_pullup = true };
        ESP_LOG_ERR(i2c_new_master_bus(&conf, &i2c_bus_handle));

        i2c_master_dev_handle_t handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = LTR303_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        ESP_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &handle));

        i2c_device_handle = handle;

        ESP_CHECK(write_register(LTR303_CONF_ADDR, LTR303_SETUP_CONF));
        ESP_CHECK(write_register(LTR303_RATE_ADDR, LTR303_SETUP_RATE));

        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t ltr303_read_lux(uint16_t lux_value[2])
    {
        if(i2c_device_handle == nullptr) {
            lux_value[0] = 64;
            lux_value[1] = 64;
            return ESP_OK;
        }
        ESP_CHECK(read_register(LTR303_DATA_CH1, lux_value[0]));
        ESP_CHECK(read_register(LTR303_DATA_CH0, lux_value[1]));
        return ESP_OK;
    }
#elif USE_DEVICE == ALS_PT243_DEVICE

    esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
    {
        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;
        bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if(!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = channel,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if(ret == ESP_OK) {
                calibrated = true;
            }
        }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if(!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if(ret == ESP_OK) {
                calibrated = true;
            }
        }
#endif

        *out_handle = handle;
        if(ret == ESP_OK) {
            ESP_LOGI(TAG, "Calibration Success");
        } else if(ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
            ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
        } else {
            ESP_LOGE(TAG, "Invalid arg or no memory");
        }
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    template <size_t WINDOW_SIZE> struct max_deque_t
    {
        //////////////////////////////////////////////////////////////////////

        max_deque_t()
        {
            front = 0;
            rear = -1;
            size = 0;
            current_index = 0;
        }

        //////////////////////////////////////////////////////////////////////

        void add_reading(int new_reading)
        {
            int new_index = current_index % WINDOW_SIZE;
            readings[new_index] = new_reading;
            if(size > 0 && current_index >= WINDOW_SIZE) {
                if(indices[front] == new_index) {
                    front = (front + 1) % WINDOW_SIZE;
                    size--;
                }
            }
            while(size > 0 && readings[indices[rear]] <= new_reading) {
                rear = (rear - 1 + WINDOW_SIZE) % WINDOW_SIZE;
                size--;
            }
            rear = (rear + 1) % WINDOW_SIZE;
            indices[rear] = new_index;
            size++;
            current_index++;
        }

        //////////////////////////////////////////////////////////////////////

        int get_max()
        {
            return size == 0 ? -1 : readings[indices[front]];
        }

        //////////////////////////////////////////////////////////////////////

        int16_t indices[WINDOW_SIZE];
        int16_t readings[WINDOW_SIZE];
        int16_t current_index;
        int16_t front;
        int16_t rear;
        int16_t size;
    };

    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;

    esp_err_t als_pt243_init()
    {
        adc_oneshot_unit_init_cfg_t init_config1{};
        init_config1.unit_id = ADC_UNIT_1;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

        adc_oneshot_chan_cfg_t config{};
        config.atten = ADC_ATTEN_DB_12;
        config.bitwidth = ADC_BITWIDTH_DEFAULT;
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));

        ESP_ERROR_CHECK(adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle));

        return ESP_OK;
    }

    float const filter_value = 0.95f;
    float filtered = 0;

    float scale = 65535 / 3150.0f;

    max_deque_t<16> max_deque;

    esp_err_t als_pt243_read_lux(uint16_t lux_value[2])
    {
        int adc_raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw));

        int voltage;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));

        max_deque.add_reading(voltage);
        int cur_max = max_deque.get_max();

        filtered = (filtered * filter_value) + (cur_max * (1.0f - filter_value));

        lux_value[0] = std::min((int)(filtered * scale), 65535);
        lux_value[1] = lux_value[0];
        return ESP_OK;
    }
#endif
    //////////////////////////////////////////////////////////////////////

    void ambient_sensor_task(void *pvParameters)
    {
        LOG_INFO("ambient_sensor_task begins");

        ESP_LOG_ERR(INIT_DEVICE());

        while(true) {

            uint16_t lux_value[2];
            if(READ_LUX(lux_value) == ESP_OK) {
                brightness = max(lux_value[0], lux_value[1]);
            }
            delay_ms(200);
        }
    }

    float smoothed_ambient = 0.0f;
    float smooth_factor = 0.01f;

}    // namespace

//////////////////////////////////////////////////////////////////////

void lux_init()
{
    xTaskCreatePinnedToCore(ambient_sensor_task, "ambient_sensor_task", 2560, NULL, 2, NULL, 0);
}

//////////////////////////////////////////////////////////////////////

void lux_update()
{
    int lux = 128;

    // ambient light response
    float ambient = (float)brightness * (1.0f / 65535);

    smoothed_ambient = smooth_factor * ambient + (1.0f - smooth_factor) * smoothed_ambient;

    float t = smoothed_ambient;
    int max = settings.get_brightness();
    if(settings.auto_brightness == auto_brightness_t::Auto) {
        int base = (max + 1) / 64;
        int range = max - base;
        lux = (int)(t * range) + base;
    } else {
        max += 1;
        max *= max;
        max >>= 8;
        lux = max - 1;
    }
    display->set_ambient(lux);
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"lux", "show lux", "">
{
    void on_command(int argc, char **argv) override
    {
        state_set(lux_state);
    }
} lux_command;

//////////////////////////////////////////////////////////////////////

void lux_state_t::on_update()
{
    if(button_select.pressed) {
        state_set(clock_state);
    }
    char hex[5];
    sprintf(hex, "%04X", brightness);
    gfx.clear();
    font_5x7_font.draw_string(gfx, hex, 0, 0, 1.0f);
    int b = brightness * 60 >> 16;
    for(int i = 0; i <= b; ++i) {
        gfx.set_second_only(1.0f, i);
    }
    gfx.display();
    display->set_ambient(255);
}