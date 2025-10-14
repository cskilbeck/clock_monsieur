//////////////////////////////////////////////////////////////////////
// gptimer fires at 16 x refresh rate (because 16 columns)
// When the timer fires:
//  - toggle latch (to latch previously sent grayscale and fcontrol data)
//  - kick off fcontrol spi send
// When fcontrol spi send completes
//  - toggle latch (to pre-latch fcontrol data)
//  - kick off grayscale spi send
// And wait for the timer to fire again

#include <cstdint>
#include <cstring>

#include "esp_log.h"

#include "driver/spi_master.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_periph.h"

#include "hal/gpio_hal.h"
#include "hal/gpio_ll.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "gpio_defs.h"
#include "i2c_task.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////

#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK

#define STATE_IDLE 0
#define STATE_BEGIN 1
#define STATE_SENT 2

#define VBLANK_BIT BIT0

#define SPI2_MOSI_PIN TLC5948_PIN_MOSI
#define SPI2_CLK_PIN TLC5948_PIN_SCLK
#define SPI2_MISO_PIN -1
#define SPI2_CS_PIN -1
#define LATCH_PIN TLC5948_PIN_XLAT

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "display";

    LOG_CONTEXT("display");

    //////////////////////////////////////////////////////////////////////

    struct fcontrol_data_t
    {
        uint16_t pad0[7];

        uint16_t lattmg0 : 1;
        uint16_t idmena : 1;
        uint16_t idmrpt : 1;
        uint16_t idmcur : 2;
        uint16_t oldena : 1;
        uint16_t psmode : 3;
        uint16_t : 7;

        uint8_t dsprpt : 1;
        uint8_t tmgrst : 1;
        uint8_t espwm : 1;
        uint8_t lodvlt : 2;
        uint8_t lsdvlt : 2;
        uint8_t lattmg1 : 1;

        uint8_t global_bc : 7;
        uint8_t blank : 1;

        uint8_t dc[14];

        void set_dc(int channel, uint8_t value);
    };

    static_assert(sizeof(fcontrol_data_t) == 256 / 8);

    //////////////////////////////////////////////////////////////////////

    EventGroupHandle_t event_group_handle = NULL;

    volatile int current_state = STATE_IDLE;
    TaskHandle_t display_task_handle = NULL;

    i2s_chan_handle_t i2s_tx_chan_handle;

    spi_device_handle_t spi2_device_handle;
    gptimer_handle_t gptimer_handle = NULL;

    spi_transaction_t spi_transaction[2];

    int s_current_slot = 0;

    uint32_t s_fcntrl_data[8];
    uint32_t s_grayscale_data[8];

    fcontrol_data_t fcontrol;
    uint16_t grayscale_buffer[2][256];
    uint16_t *backbuffer;
    int backbuffer_index = 0;

    //////////////////////////////////////////////////////////////////////

    void fcontrol_data_t::set_dc(int channel, uint8_t value)
    {
        int bit_pos = channel * 7;
        int byte_idx = 13 - (bit_pos >> 3);
        int bit_offset = bit_pos & 7;
        dc[byte_idx] = (dc[byte_idx] & ~(0x7F << bit_offset)) | (value << bit_offset);
        if(bit_offset > 1) {
            bit_offset = 8 - bit_offset;
            int upper_bits = 7 - bit_offset;
            byte_idx -= 1;
            dc[byte_idx] = (dc[byte_idx] & ~((1 << upper_bits) - 1)) | (value >> bit_offset);
        }
    }

    //////////////////////////////////////////////////////////////////////

    void spi2_isr(spi_transaction_t *trans)
    {
        if(trans->cmd == 1) {    // Command 1: FCNTRL (Buffer A). This requires the task to continue the sequence.
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            current_state = STATE_SENT;
            vTaskNotifyGiveFromISR(display_task_handle, &xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }

    //////////////////////////////////////////////////////////////////////

    bool IRAM_ATTR timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
    {
        current_state = STATE_BEGIN;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(display_task_handle, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t gpio_init(void)
    {
        gpio_config_t io_conf{};
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << LATCH_PIN);
        return gpio_config(&io_conf);
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t gsclk_init(void)
    {
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(TLC5948_I2S_NUM, I2S_ROLE_MASTER);

        ESP_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan_handle, NULL));

        i2s_std_config_t std_cfg = {};

        std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
        std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
        std_cfg.clk_cfg.sample_rate_hz = 125000;    // 125000 * 256 = 32MHz

        std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_8BIT;
        std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
        std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
        std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
        std_cfg.slot_cfg.ws_width = 0;
        std_cfg.slot_cfg.ws_pol = false;

        std_cfg.gpio_cfg.mclk = TLC5948_PIN_GSCLK;
        std_cfg.gpio_cfg.bclk = I2S_GPIO_UNUSED;
        std_cfg.gpio_cfg.ws = I2S_GPIO_UNUSED;
        std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
        std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;

        ESP_CHECK(i2s_channel_init_std_mode(i2s_tx_chan_handle, &std_cfg));
        ESP_CHECK(i2s_channel_enable(i2s_tx_chan_handle));
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t spi2_init(void)
    {
        esp_err_t ret;

        // 3. Bus Configuration
        spi_bus_config_t buscfg{};
        buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
        buscfg.mosi_io_num = SPI2_MOSI_PIN;
        buscfg.miso_io_num = SPI2_MISO_PIN;
        buscfg.sclk_io_num = SPI2_CLK_PIN;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 32;

        // 4. Device Configuration
        spi_device_interface_config_t devcfg{};
        devcfg.clock_speed_hz = 26666666;    // Target 26.666MHz
        devcfg.mode = 0;
        devcfg.spics_io_num = SPI2_CS_PIN;    // No CS line required (-1)
        devcfg.queue_size = 2;
        devcfg.command_bits = 1;    // One command bit as requested
        devcfg.address_bits = 0;
        devcfg.dummy_bits = 0;
        devcfg.flags = 0;
        devcfg.post_cb = spi2_isr;

        // Initialize the SPI bus (SPI2_HOST)
        ESP_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

        auto cleanup = DEFERRED[]()
        {
            spi_bus_free(SPI2_HOST);
        };

        // Add the device to the bus
        ESP_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi2_device_handle));

        cleanup.cancel();

        LOG_INFO("SPI2 initialized successfully.");
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t gptimer_init(void)
    {
        gptimer_config_t timer_config{};
        timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        timer_config.direction = GPTIMER_COUNT_UP;
        timer_config.resolution_hz = 40000000;
        timer_config.intr_priority = 0;

        gptimer_alarm_config_t alarm_config{};
        alarm_config.alarm_count = 6000;
        alarm_config.reload_count = 0;
        alarm_config.flags.auto_reload_on_alarm = true;

        gptimer_event_callbacks_t cbs{};
        cbs.on_alarm = timer_isr;

        gptimer_handle = nullptr;

        auto cleanup = DEFERRED([=]() {
            gptimer_disable(gptimer_handle);
            gptimer_del_timer(gptimer_handle);
        });

        ESP_CHECK(gptimer_new_timer(&timer_config, &gptimer_handle));
        ESP_CHECK(gptimer_set_alarm_action(gptimer_handle, &alarm_config));
        ESP_CHECK(gptimer_register_event_callbacks(gptimer_handle, &cbs, NULL));
        ESP_CHECK(gptimer_enable(gptimer_handle));
        ESP_CHECK(gptimer_start(gptimer_handle));

        cleanup.cancel();

        LOG_INFO("GPTimer initialized and started for 15.00094 kHz interrupt.");
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t spi2_send(uint8_t command_bit, const uint32_t *mosi_data)
    {
        int slot_index = s_current_slot;
        spi_transaction_t *t = &spi_transaction[slot_index];
        t->cmd = command_bit & 0x1;
        t->length = 256;
        t->tx_buffer = mosi_data;
        t->user = (void *)slot_index;

        ESP_CHECK(spi_device_queue_trans(spi2_device_handle, t, 0));

        s_current_slot = 1 - s_current_slot;
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    __attribute__((always_inline)) inline void memcpy256(void *dst, void *src)
    {
        uint32_t *d = (uint32_t *)dst;
        uint32_t *s = (uint32_t *)src;
        d[0] = s[0];
        d[1] = s[1];
        d[2] = s[2];
        d[3] = s[3];
        d[4] = s[4];
        d[5] = s[5];
        d[6] = s[6];
        d[7] = s[7];
    }

    //////////////////////////////////////////////////////////////////////

    __attribute__((always_inline)) inline void toggle_latch()
    {
        auto gpio = GPIO_LL_GET_HW(GPIO_PORT_0);
        gpio_ll_set_level(gpio, LATCH_PIN, 1);

        // No need for any delay nops here because the volatile write
        // issues a `memw` instruction which takes some time (gpio stays
        // high for ~60nS)

        gpio_ll_set_level(gpio, LATCH_PIN, 0);
    }

    //////////////////////////////////////////////////////////////////////

    IRAM_ATTR void display_task(void *)
    {
        LOG_INFO("DISPLAY TASK BEGINS");

        ESP_VOID(gpio_init());
        ESP_VOID(spi2_init());
        ESP_VOID(gptimer_init());
        ESP_VOID(gsclk_init());

        LOG_INFO("Entering main loop, waiting for Timer ISR notification (15kHz).");

        int column = 0;
        int frames = 0;

        while(1) {

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            switch(current_state) {

            case STATE_IDLE:
                LOG_WARN("display reached idle state!?");
                break;

            case STATE_BEGIN: {

                // timer fired, latch the grayscale data that was sent last time
                toggle_latch();

                column = (column + 1) & 15;

                if(column == 0) {
                    frames = (frames + 1) & 7;
                    if(frames == 0) {
                        backbuffer = grayscale_buffer[backbuffer_index];
                        backbuffer_index = 1 - backbuffer_index;
                        xEventGroupSetBits(event_group_handle, VBLANK_BIT);
                    }
                }

                // Prepare fcntrl buffer
                memcpy256(s_fcntrl_data, (uint32_t *)&fcontrol);

                // start sending it with command = 1
                esp_err_t err = spi2_send(0x1, s_fcntrl_data);

                if(err != ESP_OK) {
                    LOG_ERROR("Failed to send FCNTRL: %s. Skipping rest of frame.", esp_err_to_name(err));
                    // If this fails, we must reset state and wait for the next timer tick (Step 8)
                    current_state = STATE_IDLE;
                    continue;
                }

                // prepare the grayscale data while the fcontrol is being sent
                uint16_t *src = grayscale_buffer[backbuffer_index] + column * 16;
                for(int i = 0; i < 8; i++) {
                    uint32_t a = __builtin_bswap16(*src++);
                    uint32_t b = __builtin_bswap16(*src++);
                    s_grayscale_data[i] = (a << 16) | b;
                }

                // Set state back to IDLE, waiting for SPI ISR completion of Send A
                current_state = STATE_IDLE;
            } break;

            case STATE_SENT: {

                // fcontrol send completed, notify TLC5948
                toggle_latch();

                // start sending grayscale data (command = 0)
                ESP_LOG_ERR(spi2_send(0x0, s_grayscale_data));

                current_state = STATE_IDLE;
            } break;

            default:
                esp_system_abort("INVALID display state");
                break;
            }
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void display_init(void)
{
    // setup default fcontrol
    memset(&fcontrol, 0, sizeof(fcontrol));
    fcontrol.tmgrst = 1;
    fcontrol.global_bc = 127;
    for(int i = 0; i < 16; ++i) {
        fcontrol.set_dc(i, 127);
    }
    // Create VBLANK EventGroup
    event_group_handle = xEventGroupCreate();
    if(event_group_handle == NULL) {
        LOG_ERROR("Failed to create EventGroup!");
        return;
    }

    i2c_task_start();

    // core 1, priority 15
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, nullptr, 15, &display_task_handle, 1);
}

//////////////////////////////////////////////////////////////////////

uint16_t *display_update()
{
    xEventGroupWaitBits(event_group_handle, VBLANK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    return backbuffer;
}
