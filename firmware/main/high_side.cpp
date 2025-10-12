//////////////////////////////////////////////////////////////////////

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//////////////////////////////////////////////////////////////////////

#define SWITCH_00 GPIO_NUM_4
#define SWITCH_01 GPIO_NUM_5
#define SWITCH_02 GPIO_NUM_17
#define SWITCH_03 GPIO_NUM_18
#define SWITCH_04 GPIO_NUM_13
#define SWITCH_05 GPIO_NUM_14
#define SWITCH_06 GPIO_NUM_12
#define SWITCH_07 GPIO_NUM_21
#define SWITCH_08 GPIO_NUM_10
#define SWITCH_09 GPIO_NUM_8
#define SWITCH_10 GPIO_NUM_9
#define SWITCH_11 GPIO_NUM_15
#define SWITCH_12 GPIO_NUM_16
#define SWITCH_13 GPIO_NUM_6
#define SWITCH_14 GPIO_NUM_7
#define SWITCH_15 GPIO_NUM_11

static gpio_num_t high_side_gpios[16] = {
    SWITCH_00, SWITCH_01, SWITCH_02, SWITCH_03, SWITCH_04, SWITCH_05, SWITCH_06, SWITCH_07,
    SWITCH_08, SWITCH_09, SWITCH_10, SWITCH_11, SWITCH_12, SWITCH_13, SWITCH_14, SWITCH_15,
};

static int enabled_channel = -1;

//////////////////////////////////////////////////////////////////////

esp_err_t high_side_init()
{
    uint32_t mask = 0;

    for(size_t i = 0; i < 16; ++i) {
        mask |= 1 << (int)high_side_gpios[i];
        gpio_set_level(high_side_gpios[i], 1);
    }

    gpio_config_t cfg{};
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.pin_bit_mask = mask;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    return gpio_config(&cfg);
}

//////////////////////////////////////////////////////////////////////
// 0 means all off else switch on channel 1..16

void high_side_set_channel(int channel)
{
    if(enabled_channel >= 0) {
        gpio_set_level(high_side_gpios[enabled_channel], 1);
    }
    enabled_channel = channel;
    if(enabled_channel >= 0) {
        gpio_set_level(high_side_gpios[enabled_channel], 0);
    }
}
