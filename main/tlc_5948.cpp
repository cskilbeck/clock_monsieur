//////////////////////////////////////////////////////////////////////

#include <cstring>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_rom_sys.h"    // For esp_rom_delay_us
#include <string.h>         // For memset

#include "gpio_defs.h"
#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

tlc5948_control_t tlc5948_control{};

static char const *TAG = "TLC5948";

static spi_device_handle_t spi_device_handle;
static i2s_chan_handle_t i2s_tx_chan_handle;
static spi_transaction_t spi_transaction{};

static uint8_t tx_buffer[33] = { 0 };

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

static void tlc5948_pulse_xlat(void)
{
    // gpio_set_level(TLC5948_PIN_XLAT, 1);
    // esp_rom_delay_us(1);
    // gpio_set_level(TLC5948_PIN_XLAT, 0);
}

//////////////////////////////////////////////////////////////////////

static esp_err_t spi_send(uint8_t *buffer, size_t bit_count)
{
    spi_transaction.length = bit_count;
    spi_transaction.tx_buffer = buffer;
    ESP_ERROR_CHECK(spi_device_queue_trans(spi_device_handle, &spi_transaction, portMAX_DELAY));
    return ESP_OK;
}

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

esp_err_t tlc5948_init(void)
{
    ESP_LOGI(TAG, "TLC5948 INIT");

    gpio_config_t io_conf{};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TLC5948_PIN_XLAT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
    gpio_set_level(TLC5948_PIN_XLAT, 0);

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
    tlc5948_set_fcntrl();
    tlc5948_set_grayscale();
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

void tlc5948_set_fcntrl()
{
    ESP_LOGI(TAG, "SET TLC5948 fcntrl");
    memset(tx_buffer, 0, sizeof(tx_buffer));
    set_bits(tx_buffer, 0x1, tag_pos, 1);
    set_bits(tx_buffer, tlc5948_control.fcntrl.fcntrl_data, fcntl_pos, fcntl_len);
    set_bits(tx_buffer, tlc5948_control.global_bc, global_bc_pos, global_bc_len);
    size_t pos = dc_pos;
    for(int i = 0; i < 16; ++i) {
        set_bits(tx_buffer, tlc5948_control.dc[i], pos, 7);
        pos += 7;
    }
    spi_send(tx_buffer, 257);
    tlc5948_pulse_xlat();
}

//////////////////////////////////////////////////////////////////////

void tlc5948_set_grayscale()
{
    memset(tx_buffer, 0, sizeof(tx_buffer));
    set_bits(tx_buffer, 0x0, tag_pos, 1);
    size_t pos = 1;
    for(int i = 0; i < 16; ++i) {
        set_bits(tx_buffer, tlc5948_control.brightness[i], pos, 16);
        pos += 16;
    }
    spi_send(tx_buffer, 257);
    tlc5948_pulse_xlat();
}
