//////////////////////////////////////////////////////////////////////

#include <cstdint>

#include "driver/spi_master.h"
#include "driver/i2s_std.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

#include "hal/spi_hal.h"
#include "hal/spi_ll.h"
#include "hal/gpio_ll.h"

#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"

#include "esp_log.h"

#include "gpio_defs.h"
#include "i2c_task.h"
#include "display.h"

//////////////////////////////////////////////////////////////////////

#define TLC5948_HOST SPI2_HOST

#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK

#define ON_FRAME_BIT 1

// RMT ticks at 1uS = 1MHz
#define RMT_RESOLUTION_HZ 1000000

// HIGH (1), LOW (31) uS = 32uS total = 1000000 / 32 = 31250Hz, but...
// We send GS,FCNTL on each alternate IRQ, so actually 15625Hz
// And we have to send 16 times for the whole panel, so 976.5625Hz refresh rate
// Then we switch the display ram every 8 refreshes, so FPS = ~122hz

#define RMT_HIGH_PULSE_US 1
#define RMT_LOW_SPACE_US (32 - RMT_HIGH_PULSE_US)

#define CMD_BITS 1
#define DATA_BITS 256

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "display";

    //////////////////////////////////////////////////////////////////////
    // Format of the control data

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
    };

    static_assert(sizeof(fcontrol_data_t) == 256 / 8);

    //////////////////////////////////////////////////////////////////////
    // helper for setting up SPI buffers

    struct tlc5948_control_t
    {
        uint16_t brightness[16];
        fcontrol_data_t fcntrl;

        void reset();
        void set_dc(int channel, uint8_t value);
        void set_brightness(int channel, uint16_t value);
    };

    //////////////////////////////////////////////////////////////////////
    // Double buffered SPI data

    struct buffer_t
    {
        static constexpr size_t buffer_size = 256 / 32;

        uint32_t buffer[2][buffer_size];
        int index;

        void reset()
        {
            index = 0;
            memset(buffer[0], 0, buffer_size);
            memset(buffer[1], 0, buffer_size);
        }
    };

    // Display task
    TaskHandle_t display_task_handle;

    // For notifying display task that spi sends have been kicked off
    volatile EventGroupHandle_t display_event_group;

    // RMT stuff for pulsing tlc5948 LATCH
    rmt_channel_handle_t tx_chan = NULL;
    rmt_encoder_handle_t copy_encoder = NULL;

    // SPI for sending to tlc5948
    spi_device_handle_t spi_device_handle;

    // I2S for tlc5948 GSCLK
    i2s_chan_handle_t i2s_tx_chan_handle;

    // One each for sending grayscale and fcontrol
    buffer_t grayscale_buffer;
    buffer_t fcontrol_buffer;
    buffer_t *buffers[2] = { &grayscale_buffer, &fcontrol_buffer };

    // each time latch goes high, SPI sends the next brightness or fcntrl
    int brightness_fcntrl_toggle = 0;

    uint16_t display_ram[2][256];
    int backbuffer_id = 0;

    uint16_t *backbuffer = display_ram[0];

    tlc5948_control_t tlc5948_control{};

    //////////////////////////////////////////////////////////////////////
    // copy current user values into SPI buffer

    void set_grayscale()
    {
        int which = grayscale_buffer.index ^ 1;
        uint32_t *src = (uint32_t *)tlc5948_control.brightness;
        uint32_t *dst = grayscale_buffer.buffer[which];
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        dst[3] = src[3];
        dst[4] = src[4];
        dst[5] = src[5];
        dst[6] = src[6];
        dst[7] = src[7];
        grayscale_buffer.index = which;
    }

    //////////////////////////////////////////////////////////////////////
    // copy current user values into SPI buffer

    void set_fcntrl()
    {
        int which = fcontrol_buffer.index ^ 1;
        uint32_t *src = (uint32_t *)&tlc5948_control.fcntrl;
        uint32_t *dst = fcontrol_buffer.buffer[which];
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        dst[3] = src[3];
        dst[4] = src[4];
        dst[5] = src[5];
        dst[6] = src[6];
        dst[7] = src[7];
        fcontrol_buffer.index = which;
    }

    //////////////////////////////////////////////////////////////////////

    void tlc5948_control_t::reset()
    {
        memset(&fcntrl, 0, sizeof(fcntrl));
        memset(brightness, 0, sizeof(brightness));
        fcntrl.tmgrst = 1;
        fcntrl.global_bc = 64;
        for(int i = 0; i < 16; ++i) {
            set_dc(i, 64);
        }
        set_grayscale();
        set_fcntrl();
    }

    //////////////////////////////////////////////////////////////////////

    void tlc5948_control_t::set_brightness(int channel, uint16_t value)
    {
        // assert(channel >= 0 && channel <= 15);
        // value: full brightness is 2045 (ish), anything above that = full on

        brightness[channel] = __builtin_bswap16(value);
    }

    //////////////////////////////////////////////////////////////////////

    void tlc5948_control_t::set_dc(int channel, uint8_t value)
    {
        // assert(channel >= 0 && channel <= 15);
        // assert(value < 128);

        int bit_pos = channel * 7;
        int byte_idx = 13 - (bit_pos >> 3);
        int bit_offset = bit_pos & 7;
        fcntrl.dc[byte_idx] = (fcntrl.dc[byte_idx] & ~(0x7F << bit_offset)) | (value << bit_offset);
        if(bit_offset > 1) {
            bit_offset = 8 - bit_offset;
            int upper_bits = 7 - bit_offset;
            byte_idx -= 1;
            fcntrl.dc[byte_idx] = (fcntrl.dc[byte_idx] & ~((1 << upper_bits) - 1)) | (value >> bit_offset);
        }
    }

    //////////////////////////////////////////////////////////////////////
    // latch went high (RMT) which latches the last thing sent (gs or fcntl)
    // send SPI for next thing and notify display_task if we sent both

    void IRAM_ATTR gpio_isr_handler(void *arg)
    {
        // kick off SPI send
        buffer_t &buffer = *buffers[brightness_fcntrl_toggle];
        uint32_t *data = buffer.buffer[buffer.index];

        GPSPI2.user2.usr_command_value = brightness_fcntrl_toggle ? 0xffff : 0;
        GPSPI2.cmd.update = 1;

        GPSPI2.data_buf[0] = data[0];
        GPSPI2.data_buf[1] = data[1];
        GPSPI2.data_buf[2] = data[2];
        GPSPI2.data_buf[3] = data[3];
        GPSPI2.data_buf[4] = data[4];
        GPSPI2.data_buf[5] = data[5];
        GPSPI2.data_buf[6] = data[6];
        GPSPI2.data_buf[7] = data[7];

        GPSPI2.cmd.usr = 1;

        // toggle sending grayscale, fcontrol
        brightness_fcntrl_toggle ^= 1;

        if(brightness_fcntrl_toggle == 0) {
            // notify display_task that both were sent
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(display_task_handle, &woken);
            if(woken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }

    //////////////////////////////////////////////////////////////////////
    // Call this on the CPU which display_task will be running on

    esp_err_t setup_rmt_gpio_interrupt()
    {
        // must call this BEFORE setting up RMT apparently
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

        rmt_tx_channel_config_t tx_chan_config{};
        tx_chan_config.gpio_num = TLC5948_PIN_XLAT;
        tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
        tx_chan_config.mem_block_symbols = 64;
        tx_chan_config.trans_queue_depth = 4;
        tx_chan_config.flags.invert_out = false;
        tx_chan_config.flags.with_dma = false;
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan));

        ESP_ERROR_CHECK(rmt_enable(tx_chan));

        rmt_copy_encoder_config_t encoder_config{};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoder));

        rmt_symbol_word_t one_cycle_symbol{};
        one_cycle_symbol.level0 = 1;    // HIGH level first
        one_cycle_symbol.duration0 = RMT_HIGH_PULSE_US;
        one_cycle_symbol.level1 = 0;    // LOW level second
        one_cycle_symbol.duration1 = RMT_LOW_SPACE_US;

        rmt_transmit_config_t tx_config{};
        tx_config.loop_count = -1;    // -1 means infinite loop!
        tx_config.flags.eot_level = 0;
        ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, &one_cycle_symbol, sizeof(one_cycle_symbol), &tx_config));

        ESP_ERROR_CHECK(gpio_input_enable(TLC5948_PIN_XLAT));

        ESP_ERROR_CHECK(gpio_set_intr_type(TLC5948_PIN_XLAT, GPIO_INTR_POSEDGE));

        ESP_ERROR_CHECK(gpio_isr_handler_add(TLC5948_PIN_XLAT, gpio_isr_handler, nullptr));

        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    esp_err_t tlc5948_init(void)
    {
        ESP_LOGI(TAG, "TLC5948 INIT");

        // 1. Init the SPI bus using the driver (sets up the GPIOs)

        spi_bus_config_t buscfg = {
            .mosi_io_num = TLC5948_PIN_MOSI,
            .miso_io_num = -1,
            .sclk_io_num = TLC5948_PIN_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .data4_io_num = -1,
            .data5_io_num = -1,
            .data6_io_num = -1,
            .data7_io_num = -1,
            .max_transfer_sz = 0,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(TLC5948_HOST, &buscfg, SPI_DMA_CH_AUTO));

        // 2. Set up the SPI clocks and transfer stuff

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
        GPSPI2.user2.usr_command_bitlen = CMD_BITS - 1;
        GPSPI2.ms_dlen.ms_data_bitlen = DATA_BITS - 1;
        GPSPI2.cmd.update = 1;
        while(GPSPI2.cmd.update) {
        }

        // 3. Initialize I2S for GSCLK (32MHz)

        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(TLC5948_I2S_NUM, I2S_ROLE_MASTER);

        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan_handle, NULL));

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

        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan_handle, &std_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan_handle));

        // 4. init the control structures and buffers

        fcontrol_buffer.reset();
        grayscale_buffer.reset();
        tlc5948_control.reset();

        ESP_LOGI(TAG, "TLC5948 Initialized. GSCLK on pin %d at 32MHz. SPI on host %d.", TLC5948_PIN_GSCLK, TLC5948_HOST);

        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////

    void display_task(void *)
    {
        i2c_task_start();
        tlc5948_init();
        setup_rmt_gpio_interrupt();

        int frames = 0;
        while(true) {
            // wait for notification from ISR
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // setup the SPI buffers for next time
            uint8_t b = brightness;
            tlc5948_control.fcntrl.global_bc = b;
            int offset = (frames & 15) * 16;
            for(int i = 0; i < 16; ++i) {
                tlc5948_control.set_dc(i, b);
                tlc5948_control.set_brightness(i, display_ram[backbuffer_id][offset + i]);
            }
            set_grayscale();
            set_fcntrl();
            frames = (frames + 1) & 127;
            if(frames == 0) {
                backbuffer_id ^= 1;
                backbuffer = display_ram[backbuffer_id];
                // Notify client when all 16 banks have gone out 8 times (= ~122Hz)
                xEventGroupSetBits(display_event_group, ON_FRAME_BIT);
            }
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void display_init()
{
    ESP_LOGI(TAG, "init");
    display_event_group = xEventGroupCreate();
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, nullptr, 20, &display_task_handle, 1);
}

//////////////////////////////////////////////////////////////////////

uint16_t *display_update()
{
    xEventGroupWaitBits(display_event_group, ON_FRAME_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    return backbuffer;
}
