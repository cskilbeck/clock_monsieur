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

#include "../../font/font_data.h"

//////////////////////////////////////////////////////////////////////

#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK

#define SPI2_MOSI_PIN TLC5948_PIN_MOSI
#define SPI2_CLK_PIN TLC5948_PIN_SCLK
#define SPI2_MISO_PIN -1
#define SPI2_CS_PIN -1
#define LATCH_PIN TLC5948_PIN_XLAT

#define VBLANK_BIT BIT0

//////////////////////////////////////////////////////////////////////

uint16_t const matrix_lookup[7][26] = {
    { 0xC7, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xC0, 0xCF, 0xCE, 0xCD, 0xCC, 0xCB,
      0xF7, 0xF6, 0xF5, 0xF4, 0xF3, 0xF2, 0xF1, 0xF0, 0xFF, 0xFE, 0xFD, 0xFC, 0xFB },

    { 0xD7, 0xD6, 0xD5, 0xD4, 0xD3, 0xD2, 0xD1, 0xD0, 0xDF, 0xDE, 0xDD, 0xDC, 0xDB,
      0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x1F, 0x1E, 0x1D, 0x1C, 0x1B },

    { 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, 0xBF, 0xBE, 0xBD, 0xBC, 0xBB,
      0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B },

    { 0xA7, 0xA6, 0xA5, 0xA4, 0xA3, 0xA2, 0xA1, 0xA0, 0xAF, 0xAE, 0xAD, 0xAC, 0xAB,
      0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B },

    { 0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x8F, 0x8E, 0x8D, 0x8C, 0x8B,
      0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x3F, 0x3E, 0x3D, 0x3C, 0x3B },

    { 0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x9F, 0x9E, 0x9D, 0x9C, 0x9B,
      0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, 0x50, 0x5F, 0x5E, 0x5D, 0x5C, 0x5B },

    { 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71, 0x70, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B,
      0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x40, 0x4F, 0x4E, 0x4D, 0x4C, 0x4B },
};

uint16_t const hour_lookup[12] = {
    235, 226, 26, 10, 90, 106, 111, 101, 153, 169, 218, 234,
};

uint16_t const second_lookup[60] = {
    236, 231, 230, 229, 228, 227, 225, 248, 250, 249, 24,  25,  40,  42,  41,  8,   9,   56,  58,  57,
    88,  89,  72,  74,  104, 105, 107, 108, 109, 110, 96,  97,  98,  99,  100, 102, 103, 121, 122, 120,
    154, 152, 137, 138, 136, 170, 168, 185, 184, 217, 216, 201, 202, 200, 232, 233, 224, 239, 238, 237,
};

uint16_t const seconds_hours_lookup[120] = {
    236, 235, 231, 231, 230, 230, 229, 229, 228, 228, 227, 226, 225, 225, 248, 248, 250, 250, 249, 249, 24,  26,  25,  25,
    40,  40,  42,  42,  41,  41,  8,   10,  9,   9,   56,  56,  58,  58,  57,  57,  88,  90,  89,  89,  72,  72,  74,  74,
    104, 104, 105, 106, 107, 107, 108, 108, 109, 109, 110, 110, 96,  111, 97,  97,  98,  98,  99,  99,  100, 100, 102, 101,
    103, 103, 121, 121, 122, 122, 120, 120, 154, 153, 152, 152, 137, 137, 138, 138, 136, 136, 170, 169, 168, 168, 185, 185,
    184, 184, 217, 217, 216, 218, 201, 201, 202, 202, 200, 200, 232, 232, 233, 234, 224, 224, 239, 239, 238, 238, 237, 237,
};

//////////////////////////////////////////////////////////////////////

void display_t::set_pixel(uint16_t color, uint8_t pixel)
{
    grayscale_buffer[pixel] = color;
}

//////////////////////////////////////////////////////////////////////

void display_t::set_second(uint16_t color, uint8_t second)
{
    uint16_t const *s = seconds_hours_lookup + second * 2;
    grayscale_buffer[s[0]] = color;
    grayscale_buffer[s[1]] = color;
}

//////////////////////////////////////////////////////////////////////

int display_t::draw_char(int c, int x, int y, int color)
{
    constexpr int glyph_height = 7;
    constexpr int glyph_width = 6;
    constexpr int screen_height = 7;
    constexpr int screen_width = 26;

    // takes ascii but font starts at ' ' (32)
    c -= 32;
    if(c < 0 || c > 95) {
        c = 95;
    }

    glyph_t &glyph = font_6x7[c];

    int shift_bits = glyph.shift;
    int width_bits = glyph.width;

    c *= glyph_height;
    uint8_t const *char_data = glyph.data;

    // fully clipped?
    if(y <= -glyph_height) {
        return width_bits;
    }

    if(x <= -width_bits) {
        return width_bits;
    }

    // y clip
    int y_end = y + glyph_height;
    if(y < 0) {
        int clip = -y;
        char_data += clip;
        y_end = glyph_height - clip;
        y = 0;
    }
    if(y_end > screen_height) {
        y_end = screen_height;
    }

    // x clip
    int shift = shift_bits;
    int x_end = x + width_bits;
    if(x < 0) {
        int clip = -x;
        shift += clip;
        x_end = glyph_width - clip;
        x = 0;
    }
    if(x_end > screen_width) {
        x_end = screen_width;
    }

    // draw it
    for(; y < y_end; ++y) {
        uint8_t row = *char_data++;
        row >>= shift;
        for(int sx = x; sx < x_end; ++sx) {
            if((row & 1) != 0) {
                grayscale_buffer[matrix_lookup[y][sx]] = color;
            }
            row >>= 1;
        }
    }
    return width_bits;
}

//////////////////////////////////////////////////////////////////////

int display_t::draw_string(const char *str, int x, int y, int color)
{
    int width = 0;
    while(*str) {
        int w = draw_char(*str, x, y, color) + 1;
        ++str;
        x += w;
        width += w;
    }
    return width;
}

//////////////////////////////////////////////////////////////////////

int measure_string(const char *str)
{
    int width = 0;
    while(int c = *str++) {
        c -= 32;
        if(c < 0 || c > 95) {
            c = 95;
        }
        width += font_6x7[c].width + 1;
    }
    return width;
}

//////////////////////////////////////////////////////////////////////

void display_t::draw_time(int hours, int minutes, int color, int colon_color)
{
    char buffer[16];
    hours %= 12;
    if(hours == 0) {
        hours = 12;
    }
    minutes %= 60;
    sprintf(buffer, "%2d%02d", hours, minutes);
    draw_char(buffer[0], 0, 0, color);
    draw_char(buffer[1], 6, 0, color);
    draw_char(buffer[2], 14, 0, color);
    draw_char(buffer[3], 20, 0, color);
    grayscale_buffer[matrix_lookup[2][12]] = colon_color;
    grayscale_buffer[matrix_lookup[4][12]] = colon_color;
}

//////////////////////////////////////////////////////////////////////

void display_t::set_ambient(int b)
{
    uint8_t c = (uint8_t)(b / 2);
    fcontrol.global_bc = c;
    uint8_t d = (uint8_t)((b + 1) / 2);
    if(d > 127) {
        d = 127;
    }
    for(int i = 0; i < 16; ++i) {
        fcontrol.set_dc(i, d);
    }
}

//////////////////////////////////////////////////////////////////////

void display_t::cls(int color)
{
    for(int i = 0; i < 256; ++i) {
        grayscale_buffer[i] = color;
    }
}

namespace
{
    LOG_CONTEXT("display");

    //////////////////////////////////////////////////////////////////////

    display_t __attribute__((__aligned__(32))) display_data[2];

    EventGroupHandle_t event_group_handle = NULL;
    TaskHandle_t display_task_handle = NULL;

    i2s_chan_handle_t i2s_tx_chan_handle;
    gptimer_handle_t gptimer_handle = NULL;

    int buffer_index = 0;
    display_t *back_buffer = display_data + 0;
    display_t *front_buffer = display_data + 1;

    // The order here is important. No two lines from the same half bridge
    // may be adjacent (the half bridge can't switch A on and B off instantly)

    DRAM_ATTR gpio_num_t const high_side_gpios[16] = {
        // clang-format off
        HIGH_SIDE_GPIO_06,
        HIGH_SIDE_GPIO_05,
        HIGH_SIDE_GPIO_07,
        HIGH_SIDE_GPIO_04,
        HIGH_SIDE_GPIO_02,
        HIGH_SIDE_GPIO_00,
        HIGH_SIDE_GPIO_03,
        HIGH_SIDE_GPIO_01,
        HIGH_SIDE_GPIO_14,
        HIGH_SIDE_GPIO_12,
        HIGH_SIDE_GPIO_15,
        HIGH_SIDE_GPIO_13,
        HIGH_SIDE_GPIO_10,
        HIGH_SIDE_GPIO_08,
        HIGH_SIDE_GPIO_11,
        HIGH_SIDE_GPIO_09,
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

        // For 2048 levels of grayscale, at 32MHz grayscale PWM clock we need 64uS

        // gptimer clock is 40 MHz
        // we do 16 refresh cycles per frame
        // so 40000000 / 16 = 2500000 cycles per refresh
        // For a target framerate of 913 fps, we get
        // 2500000 / 913 = 2738.22563, rounding, we get
        // 2500000 / 2738 = 913.0752374 fps
        // Each refresh cycle has ?? uS
        // Giving a max PWM counter of ??

        gptimer_alarm_config_t alarm_config{};
        alarm_config.alarm_count = 2738;    // 
        // alarm_config.alarm_count = 2600;
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
            gpio_ll_set_level(&GPIO, high_side_gpios[prev_column], 1);

            // next column
            current_column = (current_column + 1) & 15;
            prev_column = current_column;

            // latch in the grayscale data from previous loop (display of current column starts now)
            toggle_latch();

            // switch on current column so it actually lights up
            gpio_ll_set_level(&GPIO, high_side_gpios[current_column], 0);

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

display_t &display_update()
{
    xEventGroupWaitBits(event_group_handle, VBLANK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    return *back_buffer;
}
