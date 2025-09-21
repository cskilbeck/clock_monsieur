//////////////////////////////////////////////////////////////////////

#include <cstring>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2s_std.h"

#include "tlc_5948.h"

//////////////////////////////////////////////////////////////////////

#define PIN_NUM_MOSI GPIO_NUM_4
#define PIN_NUM_MISO -1    // Not used for TLC5948 (data is write-only)
#define PIN_NUM_CLK GPIO_NUM_5
#define PIN_NUM_CS -1    // Not used, manual latching is required
#define PIN_NUM_GSCLK GPIO_NUM_6
#define PIN_NUM_LAT GPIO_NUM_7
#define PIN_NUM_BLANK GPIO_NUM_8

//////////////////////////////////////////////////////////////////////

#define BSWAP16(x) (((x & 0xff) << 8) | ((x >> 8) & 0xff))

//////////////////////////////////////////////////////////////////////

static i2s_chan_handle_t i2s_chan;
static spi_device_handle_t spi;
static bool gs_clk_disabled = true;

//////////////////////////////////////////////////////////////////////
// A SPI send completed
// set LAT high
// start GS_CLK if it was stopped

static void lat_callback(spi_transaction_t *)
{
    gpio_set_level(PIN_NUM_LAT, 1);
    if(gs_clk_disabled) {
        i2s_channel_enable(i2s_chan);
        gs_clk_disabled = false;
    }
}

//////////////////////////////////////////////////////////////////////
// Set/Clear BLANK

void tlc5948_set_blank(bool blank_on)
{
    gpio_set_level(PIN_NUM_BLANK, blank_on ? 1 : 0);
}

//////////////////////////////////////////////////////////////////////

void tlc5948_init()
{
    spi_bus_config_t buscfg{};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadhd_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.max_transfer_sz = 16 * 2 + 1;    // Max transfer size in bytes for a single TLC5948

    spi_device_interface_config_t devcfg{};
    devcfg.mode = 0;                     // SPI Mode 0: CPOL=0, CPHA=0
    devcfg.clock_speed_hz = 20000000;    // 20 MHz clock speed
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 7;
    devcfg.post_cb = lat_callback;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // MCLK_freq = sample_rate_hz * num_channels * mclk_multiple
    // sample_rate_hz = 125000, num_channels = 1 (mono), mclk_multiple = 256 (default I2S_MCLK_MULTIPLE_256)
    // MCLK = 125000 * 1 * 256 = 32MHz
    i2s_std_config_t tx_std_cfg{};
    tx_std_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(125000),
    tx_std_cfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    tx_std_cfg.gpio_cfg.mclk = GPIO_NUM_2;
    tx_std_cfg.gpio_cfg.bclk = GPIO_NUM_NC;
    tx_std_cfg.gpio_cfg.ws = GPIO_NUM_NC;
    tx_std_cfg.gpio_cfg.dout = GPIO_NUM_NC;
    tx_std_cfg.gpio_cfg.din = GPIO_NUM_NC;

    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &i2s_chan, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_chan, &tx_std_cfg));
}

//////////////////////////////////////////////////////////////////////
// Set 16 LED PWM values, 12 bits each

void tlc5948_set_grayscale(uint16_t *gs_data)
{
    uint16_t tx_buffer[16];
    for(int i = 0; i < 16; i++) {
        tx_buffer[i] = BSWAP16(gs_data[i]);
    }
    gpio_set_level(PIN_NUM_LAT, 0);

    spi_transaction_t t{};
    t.length = 16 * 8;
    t.tx_buffer = tx_buffer;

    ESP_ERROR_CHECK(spi_device_queue_trans(spi, &t, portMAX_DELAY));
}

//////////////////////////////////////////////////////////////////////

void tlc5948_set_control_data(tlc5948_control_data *control_data)
{
    uint8_t *dc_data = reinterpret_cast<uint8_t *>(control_data);
    uint8_t buffer[16];
    uint32_t bits = 0x7Eul << 1;
    int got = 8;
    int dst = 0;
    for(int i = 0; i < 17; i++) {
        bits |= (dc_data[i] & 0x7Ful) << got;
        got += 7;
        if(got >= 8) {
            got -= 8;
            buffer[dst++] = (bits >> got) & 0xFF;
            bits >>= 8;
        }
    }
    if(got > 0) {
        buffer[dst] = bits & 0xFF;
    }

    // stop GS_CLK
    i2s_channel_disable(i2s_chan);
    gs_clk_disabled = true;

    // set LAT low
    gpio_set_level(PIN_NUM_LAT, 0);

    spi_transaction_t t{};
    t.length = 16 * 8;
    t.tx_buffer = buffer;

    // send it, and wait for it
    ESP_ERROR_CHECK(spi_device_queue_trans(spi, &t, portMAX_DELAY));
}
