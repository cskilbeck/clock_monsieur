//////////////////////////////////////////////////////////////////////

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "i2c_task.h"
#include "high_side.h"
#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "display";

#define ON_FRAME_BIT (1 << 0)

static EventGroupHandle_t display_event_group;
static TaskHandle_t frame_task;
static gptimer_handle_t frame_timer = NULL;

//////////////////////////////////////////////////////////////////////

static void IRAM_ATTR on_frame(void *)
{
    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        tlc5948_set_grayscale();
        xEventGroupSetBits(display_event_group, ON_FRAME_BIT);
    }
}

//////////////////////////////////////////////////////////////////////

static bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(frame_task, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

//////////////////////////////////////////////////////////////////////

void init_display()
{
    tlc5948_init();
    i2c_task_start();

    display_event_group = xEventGroupCreate();

    xTaskCreate(on_frame, "on_frame", 2048, nullptr, 20, &frame_task);

    constexpr uint64_t frame_rate = 400;

    constexpr uint64_t timer_freq = 40000000;
    constexpr uint64_t alarm_count = (timer_freq + (frame_rate / 2)) / frame_rate;
    constexpr uint64_t max_timer_count = 1llu << 54;
    static_assert(alarm_count < max_timer_count);

    gptimer_config_t timer_config{};
    timer_config.clk_src = GPTIMER_CLK_SRC_APB;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = timer_freq;
    timer_config.flags.intr_shared = true, gptimer_new_timer(&timer_config, &frame_timer);

    gptimer_alarm_config_t alarm_config{};
    alarm_config.alarm_count = alarm_count;
    alarm_config.reload_count = 0;
    alarm_config.flags.auto_reload_on_alarm = true;

    gptimer_event_callbacks_t cbs{};
    cbs.on_alarm = gptimer_isr_callback;

    gptimer_register_event_callbacks(frame_timer, &cbs, frame_task);
    gptimer_enable(frame_timer);
    gptimer_set_alarm_action(frame_timer, &alarm_config);
    gptimer_start(frame_timer);
}

//////////////////////////////////////////////////////////////////////

void waitvb()
{
    xEventGroupWaitBits(display_event_group, ON_FRAME_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
}
