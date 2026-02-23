//////////////////////////////////////////////////////////////////////

#include <algorithm>

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

//////////////////////////////////////////////////////////////////////

LOG_CONTEXT("lux");

namespace
{
    uint16_t brightness;

    esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
    {
        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;
        bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if(!calibrated) {
            LOG_INFO("calibration scheme version is Curve Fitting");
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
            LOG_INFO("calibration scheme version is Line Fitting");
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
            LOG_INFO("Calibration Success");
        } else if(ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
            LOG_WARN("eFuse not burnt, skip software calibration");
        } else {
            LOG_ERROR("Invalid arg or no memory");
        }
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    template <size_t WINDOW_SIZE> struct max_deque_t
    {
        // Ensure WINDOW size is a power of 2
        static_assert((WINDOW_SIZE & (WINDOW_SIZE - 1)) == 0);

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
        ESP_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw));

        int voltage;
        ESP_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));

        max_deque.add_reading(voltage);
        int cur_max = max_deque.get_max();

        filtered = (filtered * filter_value) + (cur_max * (1.0f - filter_value));

        lux_value[0] = std::min((int)(filtered * scale), 65535);
        lux_value[1] = lux_value[0];
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    void ambient_sensor_task(void *pvParameters)
    {
        LOG_INFO("ambient_sensor_task begins");

        ESP_LOG_ERR(als_pt243_init());

        while(true) {

            uint16_t lux_value[2];
            if(als_pt243_read_lux(lux_value) == ESP_OK) {
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
    // ambient light response
    float ambient = (float)brightness * (1.0f / 65535);
    smoothed_ambient = smooth_factor * ambient + (1.0f - smooth_factor) * smoothed_ambient;
    brightness_range_t brightness_range = settings.get_brightness_range();
    int lux = brightness_range.max_brightness;
    if(settings.auto_brightness == auto_brightness_t::Auto) {
        int range = brightness_range.max_brightness - brightness_range.min_brightness;
        lux = (int)(smoothed_ambient * range) + brightness_range.min_brightness;
    }
    display->set_ambient(lux);
}

//////////////////////////////////////////////////////////////////////

struct : console_command_t<"lux", "show lux", "">
{
    void on_command(int argc, char **argv) override
    {
        state_set<lux_state_t>();
    }
} lux_command;

//////////////////////////////////////////////////////////////////////

void lux_state_t::on_update()
{
    if(button_select.pressed) {
        state_set<clock_state_t>();
    }
    char hex[5];
    sprintf(hex, "%02X", display->get_ambient());
    gfx.clear();
    font_5x7_font.draw_string(gfx, hex, 0, 0, 1.0f);
    int b = brightness * 60 >> 16;
    for(int i = 0; i <= b; ++i) {
        gfx.set_second_only(1.0f, i);
    }
    gfx.display();
}