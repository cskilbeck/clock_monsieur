//////////////////////////////////////////////////////////////////////
// gptimer fires at 16 x refresh rate (because 16 columns)
// When the timer fires:
//  - toggle latch (to latch previously sent grayscale and fcontrol data)
//  - kick off fcontrol spi send
//  - BLOCKING wait for fcontrol spi to complete
//  - toggle latch (to pre-latch fcontrol data)
//  - kick off grayscale spi send
// And wait for the timer to fire again (grayscale spi completes before then)

#include "driver/spi_master.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_periph.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

#include "hal/gpio_hal.h"
#include "hal/gpio_ll.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "display.h"
#include "gpio_defs.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////

#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK

#define SPI2_MOSI_PIN TLC5948_PIN_MOSI
#define SPI2_CLK_PIN TLC5948_PIN_SCLK
#define SPI2_MISO_PIN -1
#define SPI2_CS_PIN -1
#define LATCH_PIN TLC5948_PIN_XLAT

#define VBLANK_BIT BIT0

//////////////////////////////////////////////////////////////////////

namespace
{
    LOG_CONTEXT("display");

    //////////////////////////////////////////////////////////////////////

    display_data_t __attribute__((__aligned__(32))) display_data[2];

    EventGroupHandle_t event_group_handle = NULL;
    TaskHandle_t display_task_handle = NULL;

    i2s_chan_handle_t i2s_tx_chan_handle;
    gptimer_handle_t gptimer_handle = NULL;

    int buffer_index = 0;
    display_data_t *back_buffer = display_data + 0;
    display_data_t *front_buffer = display_data + 1;

    DRAM_ATTR gpio_num_t const high_side_gpios[16] = {
        HIGH_SIDE_GPIO_00, HIGH_SIDE_GPIO_01, HIGH_SIDE_GPIO_02, HIGH_SIDE_GPIO_03, HIGH_SIDE_GPIO_04, HIGH_SIDE_GPIO_05,
        HIGH_SIDE_GPIO_06, HIGH_SIDE_GPIO_07, HIGH_SIDE_GPIO_08, HIGH_SIDE_GPIO_09, HIGH_SIDE_GPIO_10, HIGH_SIDE_GPIO_11,
        HIGH_SIDE_GPIO_12, HIGH_SIDE_GPIO_13, HIGH_SIDE_GPIO_14, HIGH_SIDE_GPIO_15,
    };

    //////////////////////////////////////////////////////////////////////
    // Init GPIOs for TLC5948 LATCH and high side switches

    esp_err_t gpio_init(void)
    {
        uint64_t mask = 0;

        for(size_t i = 0; i < 16; ++i) {
            mask |= 1ULL << (int)high_side_gpios[i];
            gpio_set_level(high_side_gpios[i], 0);
        }
        mask |= 1ULL << LATCH_PIN;
        gpio_set_level(LATCH_PIN, 0);

        gpio_config_t io_conf{};
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = mask;
        return gpio_config(&io_conf);
    }

    //////////////////////////////////////////////////////////////////////
    // Init I2S MCLK @ 32MHz for TLC5948 GSCLK

    esp_err_t i2s_init(void)
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
    // Init SPI2

    esp_err_t spi2_init(void)
    {
        esp_err_t ret;

        // Initialize SPI bus GPIOs (SPI2_HOST)
        spi_bus_config_t buscfg{};
        buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
        buscfg.mosi_io_num = SPI2_MOSI_PIN;
        buscfg.miso_io_num = SPI2_MISO_PIN;
        buscfg.sclk_io_num = SPI2_CLK_PIN;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 32;

        ESP_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

        // Set up SPI clocks and transfer stuff
        GPSPI2.clock.clkcnt_n = 2;    // N: 2 (= 3) so 80Mhz / 3 = 26.6666MHz
        GPSPI2.clock.clkcnt_l = 2;    // L: 2 (= 3)
        GPSPI2.clock.clkcnt_h = 0;    // H: 0 (= 1) so 1/3rd duty cycle for clock (seems to work)
        GPSPI2.clock.clkdiv_pre = 0;
        GPSPI2.clock.clk_equ_sysclk = 0;

        GPSPI2.clk_gate.mst_clk_sel = 1;    // 80MHz PLL CLK
        GPSPI2.clk_gate.mst_clk_active = 1;
        GPSPI2.clk_gate.clk_en = 1;

        GPSPI2.user.val = 0;
        GPSPI2.user.usr_mosi = 1;
        GPSPI2.user.usr_command = 1;
        GPSPI2.user2.usr_command_bitlen = 1 - 1;
        GPSPI2.ms_dlen.ms_data_bitlen = 256 - 1;
        GPSPI2.cmd.update = 1;
        while(GPSPI2.cmd.update) {
        }

        LOG_INFO("SPI2 initialized successfully.");
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // Per-column timer ISR

    bool timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(display_task_handle, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    //////////////////////////////////////////////////////////////////////
    // Init gptimer for per-column IRQ

    esp_err_t gptimer_init(void)
    {
        gptimer_config_t timer_config{};
        timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        timer_config.direction = GPTIMER_COUNT_UP;
        timer_config.resolution_hz = 40000000;
        timer_config.intr_priority = 0;

        // For 2048 levels of grayscale, at 32MHz GSCLK we need 64uS
        // 64uS per column = 1024uS per frame = 976.5625 Hz refresh rate
        // 2560 would be correct value for 64uS @ 40MHz gptimer clock
        // But 2600 gives a tiny bit of headroom...
        gptimer_alarm_config_t alarm_config{};
        alarm_config.alarm_count = 2600;
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

        LOG_INFO("GPTimer initialized");
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // Toggle TLC5948 LATCH for at least 30nS

    __attribute__((always_inline)) inline void toggle_latch()
    {
        gpio_ll_set_level(&GPIO, LATCH_PIN, 1);

        // No need for any delay nops here because the volatile write
        // issues a `memw` instruction which takes some time (gpio stays
        // high for ~60nS)

        gpio_ll_set_level(&GPIO, LATCH_PIN, 0);
    }

    //////////////////////////////////////////////////////////////////////
    // Start an SPI2 send with 1 CMD bit, 256 MOSI bits

    __attribute__((always_inline)) inline void spi_kick(uint32_t const *data, uint16_t command)
    {
        GPSPI2.user2.usr_command_value = command;
        GPSPI2.cmd.update = 1;
        uint32_t *p = (uint32_t *)GPSPI2.data_buf;
        // Apparently Xtensa LX7 write buffer is 4 entries
        uint32_t s0;
        uint32_t s1;
        uint32_t s2;
        uint32_t s3;
        asm volatile("l32i.n %0, %4, 0\n"
                     "l32i.n %1, %4, 4\n"
                     "l32i.n %2, %4, 8\n"
                     "l32i.n %3, %4, 12\n"

                     "s32i.n %0, %5, 0\n"
                     "s32i.n %1, %5, 4\n"
                     "s32i.n %2, %5, 8\n"
                     "s32i.n %3, %5, 12\n"

                     "l32i.n %0, %4, 16\n"
                     "l32i.n %1, %4, 20\n"
                     "l32i.n %2, %4, 24\n"
                     "l32i.n %3, %4, 28\n"

                     "s32i.n %0, %5, 16\n"
                     "s32i.n %1, %5, 20\n"
                     "s32i.n %2, %5, 24\n"
                     "s32i.n %3, %5, 28\n"

                     "memw\n"    // write buffer flush fence

                     : "=&r"(s0), "=&r"(s1), "=&r"(s2), "=&r"(s3)
                     : "r"(data), "r"(p)
                     :    // no need for memory or flags clobber, only HW registers are written to
        );
        GPSPI2.cmd.usr = 1;    // start the transfer
    }

    //////////////////////////////////////////////////////////////////////
    // Main display task

    IRAM_ATTR void display_task(void *)
    {
        LOG_INFO("DISPLAY TASK BEGINS");

        ESP_VOID(gpio_init());
        ESP_VOID(spi2_init());
        ESP_VOID(gptimer_init());
        ESP_VOID(i2s_init());

        LOG_INFO("Entering main loop");

        int prev_column = 0;
        int current_column = 0;
        uint32_t frame = 0;

        float lux = -1.0f;

        while(1) {

            // wait for timer to fire
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // BLOCKING wait for grayscale SPI complete (which it should have anyway)
            while(GPSPI2.cmd.usr) {
            }

            // switch off previous column
            gpio_ll_set_level(&GPIO, high_side_gpios[prev_column], 0);

            // latch in the grayscale data from previous loop (display of current column starts now)
            toggle_latch();

            // next column
            prev_column = current_column;
            current_column = (current_column + 1) & 15;

            // switch on current column so it actually lights up
            gpio_ll_set_level(&GPIO, high_side_gpios[current_column], 1);

            // start sending fcntl data
            spi_kick((uint32_t const *)&front_buffer->fcontrol, 0xffff);

            // if done all columns 8 times, swap front,back buffer
            if(current_column == 0) {
                frame += 1;
                if((frame & 7) == 0) {
                    buffer_index = 1 - buffer_index;
                    back_buffer = display_data + buffer_index;
                    front_buffer = display_data + (1 - buffer_index);
                    xEventGroupSetBits(event_group_handle, VBLANK_BIT);
                }
            }

            // BLOCKING wait for fcntl SPI complete
            while(GPSPI2.cmd.usr) {
            }

            // latch in the fcntl data
            toggle_latch();

            // start sending grayscale data
            spi_kick((uint32_t const *)(front_buffer->grayscale_buffer + current_column * 16), 0);
        }
    }
}    // namespace

//////////////////////////////////////////////////////////////////////
// Set the dot correction (i.e. brightness) for a channel

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
// Kick off display task and ambient lux task

void display_init(void)
{
    // setup default fcontrol
    for(int i = 0; i < 2; ++i) {
        auto &fcontrol = display_data[i].fcontrol;
        memset(&fcontrol, 0, sizeof(fcontrol));
        fcontrol.tmgrst = 1;
        fcontrol.global_bc = 0;
        for(int i = 0; i < 16; ++i) {
            fcontrol.set_dc(i, 0);
        }
    }
    // Create VBLANK EventGroup
    event_group_handle = xEventGroupCreate();
    if(event_group_handle == NULL) {
        LOG_ERROR("Failed to create EventGroup!");
        return;
    }

    // core 1, priority 15
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, nullptr, 15, &display_task_handle, 1);
}

//////////////////////////////////////////////////////////////////////
// Wait for display to refresh 8 times

display_data_t &display_update()
{
    xEventGroupWaitBits(event_group_handle, VBLANK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    return *back_buffer;
}
