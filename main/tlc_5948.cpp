//////////////////////////////////////////////////////////////////////

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
#include "esp_rom_sys.h"

#include "gpio_defs.h"
#include "i2c_task.h"
#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "display";

tlc5948_control_t tlc5948_control{};

// These must be 1,2
#define ON_GRAYSCALE_BIT 0b00000001
#define ON_FCONTROL_BIT 0b00000010

#define RMT_PULSE_GPIO_NUM TLC5948_PIN_XLAT
#define RMT_RESOLUTION_HZ 1000000

// HIGH (1), LOW (2499) uS = 400Hz for now = 200Hz framerate
// because two latches per frame (grayscale + fcontrol)
#define HIGH_PULSE_US (1)
#define LOW_SPACE_US (32 - HIGH_PULSE_US)

#define CMD_BITS 1
#define DATA_BITS 256

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

//////////////////////////////////////////////////////////////////////

namespace
{
    // Display task
    TaskHandle_t frame_task;

    // For notifying display task that spi sends have been kicked off
    volatile EventGroupHandle_t display_event_group;

    // RMT stuff for pulsing tlc5948 LATCH
    rmt_channel_handle_t tx_chan = NULL;
    rmt_encoder_handle_t copy_encoder = NULL;

    // SPI for sending to tlc5948
    spi_device_handle_t spi_device_handle;
    spi_transaction_t spi_transaction{};

    // I2S for tlc5948 GSCLK
    i2s_chan_handle_t i2s_tx_chan_handle;

    // One each for sending grayscale and fcontrol
    buffer_t grayscale_buffer;
    buffer_t fcontrol_buffer;
    buffer_t *buffers[2] = { &grayscale_buffer, &fcontrol_buffer };

    int which_to_send = 0;
    // fcontrol layout bit offsets
    constexpr size_t tag_pos = 0;
    constexpr size_t tag_len = 120;

    constexpr size_t fcntl_pos = tag_pos + tag_len;
    constexpr size_t fcntl_len = 18;

    constexpr size_t global_bc_pos = fcntl_pos + fcntl_len;
    constexpr size_t global_bc_len = 7;

    constexpr size_t dc_pos = global_bc_pos + global_bc_len;
    constexpr size_t dc_len = 16 * 7;

    constexpr size_t ctl_total = dc_pos + dc_len;

    static_assert(ctl_total == 257);

}    // namespace

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

void tlc5948_control_t::set_brightness(int channel, uint16_t value)
{
    brightness[channel] = (value << 8) | (value >> 8);
}

//////////////////////////////////////////////////////////////////////

void tlc5948_control_t::reset()
{
    memset(this, 0, sizeof(*this));
    fcntrl.tmgrst = 1;
    fcntrl.global_bc = 127;
    for(int i = 0; i < 16; ++i) {
        set_dc(i, 127);
        set_brightness(i, 0);
    }
}

//////////////////////////////////////////////////////////////////////

static void spi_send(uint32_t const *const data, int cmd)
{
    GPSPI2.user2.usr_command_value = cmd ? 0xffff : 0;
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
}

//////////////////////////////////////////////////////////////////////

static void set_grayscale()
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

static void set_fcntrl()
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

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t woken = pdFALSE;

    buffer_t &buffer = *buffers[which_to_send];
    spi_send(buffer.buffer[buffer.index], which_to_send);

    // for notifying client that something was sent
    // 1 - we sent grayscale (ON_GRAYSCALE_BIT)
    // 2 - we sent fcontrol (ON_FCONTROL_BIT)
    int sent = which_to_send + 1;

    // toggle sending grayscale, fcontrol
    which_to_send ^= 1;

    // notify display_waitvb
    xEventGroupSetBitsFromISR(display_event_group, sent, &woken);

    if(woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

//////////////////////////////////////////////////////////////////////

static esp_err_t setup_rmt_gpio_interrupt(TaskHandle_t task_to_notify)
{
    // must call this BEFORE setting up RMT apparently
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    rmt_tx_channel_config_t tx_chan_config{};
    tx_chan_config.gpio_num = RMT_PULSE_GPIO_NUM;
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
    one_cycle_symbol.duration0 = HIGH_PULSE_US;
    one_cycle_symbol.level1 = 0;    // LOW level second
    one_cycle_symbol.duration1 = LOW_SPACE_US;

    rmt_transmit_config_t tx_config{};
    tx_config.loop_count = -1;    // -1 means infinite loop!
    tx_config.flags.eot_level = 0;
    ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, &one_cycle_symbol, sizeof(one_cycle_symbol), &tx_config));

    ESP_ERROR_CHECK(gpio_input_enable(RMT_PULSE_GPIO_NUM));

    ESP_ERROR_CHECK(gpio_set_intr_type(RMT_PULSE_GPIO_NUM, GPIO_INTR_POSEDGE));

    ESP_ERROR_CHECK(gpio_isr_handler_add(RMT_PULSE_GPIO_NUM, gpio_isr_handler, (void *)task_to_notify));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static esp_err_t tlc5948_init(void)
{
    ESP_LOGI(TAG, "TLC5948 INIT");

    // 1. Init the SPI bus using the driver

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

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 27666666,
        .spics_io_num = -1,
        .queue_size = 2,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TLC5948_HOST, &devcfg, &spi_device_handle));

    // GPIOs for SPI2 are set up, now set up the clock and other SPI stuff

    GPSPI2.clock.clkcnt_n = 2;
    GPSPI2.clock.clkcnt_l = 2;
    GPSPI2.clock.clkcnt_h = 0;
    GPSPI2.clock.clkdiv_pre = 0;
    GPSPI2.clock.clk_equ_sysclk = 0;
    GPSPI2.clk_gate.clk_en = 1;
    GPSPI2.user.val = 0;
    GPSPI2.user.usr_mosi = 1;
    GPSPI2.user.usr_command = 1;
    GPSPI2.user2.usr_command_bitlen = CMD_BITS - 1;
    GPSPI2.ms_dlen.ms_data_bitlen = DATA_BITS - 1;
    GPSPI2.cmd.update = 1;
    while(GPSPI2.cmd.update) {
    }

    // 2. Initialize I2S for GSCLK (32MHz)
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

    // 3. init the control structures and buffers
    tlc5948_control.reset();
    fcontrol_buffer.reset();
    grayscale_buffer.reset();

    set_fcntrl();

    ESP_LOGI(TAG, "TLC5948 Initialized. GSCLK on pin %d at 32MHz. SPI on host %d.", TLC5948_PIN_GSCLK, TLC5948_HOST);

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

void display_init()
{
    ESP_LOGI(TAG, "init");

    display_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "display_event_group: %x", display_event_group);

    i2c_task_start();

    tlc5948_init();

    setup_rmt_gpio_interrupt(frame_task);
}

//////////////////////////////////////////////////////////////////////

void display_waitvb()
{
    xEventGroupWaitBits(display_event_group, ON_GRAYSCALE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
}

void display_update()
{
    set_grayscale();
    set_fcntrl();
}