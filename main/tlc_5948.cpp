//////////////////////////////////////////////////////////////////////

#include "driver/spi_master.h"
#include "driver/i2s_std.h"
#include "driver/rmt_tx.h"

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
#define LOW_SPACE_US (62 - HIGH_PULSE_US)

// Display task
static TaskHandle_t frame_task;

// For notifying display task that spi sends have been kicked off
static volatile EventGroupHandle_t display_event_group;

// RMT stuff for pulsing tlc5948 LATCH
static rmt_channel_handle_t tx_chan = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

// SPI for sending to tlc5948
static spi_device_handle_t spi_device_handle;
static spi_transaction_t spi_transaction{};

// I2S for tlc5948 GSCLK
static i2s_chan_handle_t i2s_tx_chan_handle;

// Double buffered SPI data
struct buffer_t
{
    static constexpr size_t buffer_size = 33;    // 33 * 8 = 264, > 257 bits required

    uint8_t buffer[2][buffer_size];
    int index;

    buffer_t()
    {
        index = 0;
        memset(buffer[0], 0, buffer_size);
        memset(buffer[1], 0, buffer_size);
    }
};

// One each for sending grayscale and fcontrol
buffer_t grayscale_buffer;
buffer_t fcontrol_buffer;
buffer_t *buffers[2] = { &grayscale_buffer, &fcontrol_buffer };
int which_to_send = 0;

// fcontrol layout bit offsets
static constexpr size_t tag_pos = 0;
static constexpr size_t tag_len = 120;

static constexpr size_t fcntl_pos = tag_pos + tag_len;
static constexpr size_t fcntl_len = 18;

static constexpr size_t global_bc_pos = fcntl_pos + fcntl_len;
static constexpr size_t global_bc_len = 7;

static constexpr size_t dc_pos = global_bc_pos + global_bc_len;
static constexpr size_t dc_len = 16 * 7;

static constexpr size_t ctl_total = dc_pos + dc_len;

static_assert(ctl_total == 257);

//////////////////////////////////////////////////////////////////////
// overwrite up to 32 bits in an array of bytes

static void set_bits(uint8_t a[], uint32_t const val, size_t const offset, size_t const count)
{
    uint32_t const src = val & (((uint64_t)1 << count) - 1);
    size_t index = offset / 8;
    size_t const bit_pos = offset % 8;
    size_t len = 8 - bit_pos;
    if(count < len) {
        len = count;
    }
    size_t bits_done = 0;
    if(len != 0) {
        uint8_t const mask = ((1U << len) - 1) << (8 - (bit_pos + len));
        uint8_t const fragment = (uint8_t)(src >> (count - len) << (8 - (bit_pos + len)));
        a[index] = (a[index] & ~mask) | (fragment & mask);
        bits_done += len;
    }
    for(index += 1; (bits_done + 8) <= count; ++index) {
        a[index] = (uint8_t)(src >> (count - (bits_done + 8)));
        bits_done += 8;
    }
    len = count - bits_done;
    if(len != 0) {
        uint8_t const mask = ((1U << len) - 1) << (8 - len);
        uint8_t const fragment = (uint8_t)(src << (8 - len));
        a[index] = (a[index] & ~mask) | (fragment & mask);
    }
}

//////////////////////////////////////////////////////////////////////

static esp_err_t tlc5948_init(void)
{
    ESP_LOGI(TAG, "TLC5948 INIT");

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
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TLC5948_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 32000000,
        .spics_io_num = TLC5948_PIN_XLAT,
        .queue_size = 2,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TLC5948_HOST, &devcfg, &spi_device_handle));

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

    ESP_LOGI(TAG, "TLC5948 Initialized. GSCLK on pin %d at 32MHz. SPI on host %d.", TLC5948_PIN_GSCLK, TLC5948_HOST);

    tlc5948_control.fcntrl.dsprpt = 1;
    tlc5948_control.global_bc = 127;
    for(int i = 0; i < 16; ++i) {
        tlc5948_control.dc[i] = 127;
    }
    for(int i = 0; i < 16; ++i) {
        tlc5948_control.brightness[i] = 0;
    }

    for(int i = 0; i < 2; ++i) {
        fcontrol_buffer.index = i;
        grayscale_buffer.index = i;
        display_set_grayscale();
        display_set_fcntrl();
    }

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

static void IRAM_ATTR on_frame(void *)
{
    while(true) {
        // wait for latch to get toggle (tlc5948 just latched last spi send)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Kick off the next SPI send
        // alternates between sending grayscale and fcontrol buffers
        buffer_t &buffer = *buffers[which_to_send];
        buffer.index ^= 1;
        spi_transaction.length = 257;
        spi_transaction.tx_buffer = buffer.buffer;
        ESP_ERROR_CHECK(spi_device_queue_trans(spi_device_handle, &spi_transaction, portMAX_DELAY));

        // for notifying client that something was sent
        // 1 - we sent grayscale (ON_GRAYSCALE_BIT)
        // 2 - we sent fcontrol (ON_FCONTROL_BIT)
        int sent = which_to_send + 1;

        // toggle sending grayscale, fcontrol
        which_to_send ^= 1;

        // notify display_waitvb
        xEventGroupSetBits(display_event_group, sent);
    }
}

//////////////////////////////////////////////////////////////////////

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &woken);
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

void display_init()
{
    ESP_LOGI(TAG, "init");

    display_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "display_event_group: %x", display_event_group);

    i2c_task_start();

    xTaskCreate(on_frame, "on_frame", 2048, nullptr, 20, &frame_task);

    tlc5948_init();

    setup_rmt_gpio_interrupt(frame_task);
}

//////////////////////////////////////////////////////////////////////
// Put the bits into one of the grayscale buffers and set
// the pointer so it gets picked up next time

void display_set_fcntrl()
{
    uint8_t *buffer = fcontrol_buffer.buffer[fcontrol_buffer.index];
    set_bits(buffer, 0x1, tag_pos, 1);
    set_bits(buffer, tlc5948_control.fcntrl.fcntrl_data, fcntl_pos, fcntl_len);
    set_bits(buffer, tlc5948_control.global_bc, global_bc_pos, global_bc_len);
    size_t pos = dc_pos;
    for(int i = 0; i < 16; ++i) {
        set_bits(buffer, tlc5948_control.dc[i], pos, 7);
        pos += 7;
    }
}

//////////////////////////////////////////////////////////////////////

void display_set_grayscale()
{
    uint8_t *buffer = grayscale_buffer.buffer[grayscale_buffer.index];
    set_bits(buffer, 0x0, tag_pos, 1);
    size_t pos = 1;
    for(int i = 0; i < 16; ++i) {
        set_bits(buffer, tlc5948_control.brightness[i], pos, 16);
        pos += 16;
    }
}

//////////////////////////////////////////////////////////////////////

void display_waitvb()
{
    xEventGroupWaitBits(display_event_group, ON_GRAYSCALE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
}
