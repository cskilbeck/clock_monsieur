//////////////////////////////////////////////////////////////////////
//
// Buzzer driver — RMT+DMA square-wave drive
//                 with per-cycle ADSR envelope and melody sequencer
//
// Signal path
// ───────────
//  ESP32-S3 GPIO (RMT) ──► R4 (1k5) ──► BSS138 gate ──► BSS138 drain ──► buzzer ──► +5V
//
//  The RMT peripheral generates a square wave via DMA.  Each RMT symbol
//  encodes one complete tone cycle: duration0 = high (duty), duration1 = low.
//  A custom encoder modulates the duty ratio per-cycle to produce a smooth
//  raised-cosine ADSR envelope (~2700 updates/sec at resonance).
//
//  MLT-7525 buzzer: rated 3.6 V (range 2.5–4.5 V), 95 mA, 2.7 kHz.
//  50% duty square wave at 5 V supply ≈ 93 mA RMS — within rating.
//
// Why per-cycle RMT beats LEDC tick-based modulation
// ──────────────────────────────────────────────────
//  LEDC: envelope updates once per FreeRTOS tick (~10 ms at 100 Hz),
//  giving coarse 2-step attacks and incomplete releases.
//
//  RMT+DMA: each DMA symbol = one tone cycle, so envelope resolution
//  equals the tone frequency (~2700 updates/sec).  DMA runs autonomously;
//  the ISR fires only when a ping-pong buffer node is consumed (~5×/sec),
//  making CPU usage near-zero.
//
//  Each symbol is self-contained (one full cycle), so buffer-boundary
//  phase continuity is trivial — no 50 Hz DMA-glitch artifacts.
//
//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

#include "buzzer.h"
#include "melodies.h"
#include "gpio_defs.h"
#include "util.h"
#include "console.h"

//////////////////////////////////////////////////////////////////////

LOG_CONTEXT("buzzer");

//////////////////////////////////////////////////////////////////////

namespace
{
    //////////////////////////////////////////////////////////////////////
    // RMT configuration

    static constexpr uint32_t RMT_RESOLUTION_HZ = 5000000;    // 5 MHz (200 ns ticks)
    static constexpr size_t RMT_MEM_SYMBOLS = 512;            // DMA buffer: 256 per ping-pong node

    //////////////////////////////////////////////////////////////////////
    // Cosine ramp LUT — 256 entries, values 0..1000 (permil).
    //
    // cosine_ramp[i] = (1 − cos(π·i/255)) × 500
    //   i=0  → 0    (ramp start)
    //   i=255→ 1000  (ramp end)
    //
    // Computed once at init; stored in DRAM for safe ISR access.

    static uint16_t cosine_ramp[256];

    static void init_cosine_ramp()
    {
        for(int i = 0; i < 256; ++i) {
            double a = M_PI * i / 255.0;
            cosine_ramp[i] = (uint16_t)((1.0 - cos(a)) * 500.0 + 0.5);
        }
    }

    //////////////////////////////////////////////////////////////////////
    // Custom RMT encoder — produces one RMT symbol per tone cycle,
    // modulating duty (amplitude) according to the ADSR envelope.

    struct buzzer_encoder_t
    {
        rmt_encoder_t base;             // must be first member
        rmt_encoder_t *copy_encoder;    // ESP-IDF built-in copy helper

        // Note parameters (set by task before rmt_transmit)
        uint32_t cycle_ticks;     // RMT_RESOLUTION_HZ / freq_hz
        uint32_t total_cycles;    // 0 = indefinite
        uint32_t attack_cycles;
        uint32_t decay_cycles;
        uint32_t release_cycles;
        uint16_t sustain_permil;    // sustain_level × 1000
        uint16_t volume_permil;     // amplitude × 1000

        // Runtime state (ISR context)
        uint32_t cycle_index;
    };

    //////////////////////////////////////////////////////////////////////
    // encode() — IRAM_ATTR, runs in ISR context.
    //
    // Fills the DMA buffer with RMT symbols, one per tone cycle.
    // Each symbol: duration0 = high phase (duty), duration1 = low phase.
    // The duty ratio encodes the ADSR envelope amplitude.

    static size_t IRAM_ATTR buzzer_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size,
                                          rmt_encode_state_t *ret_state)
    {
        buzzer_encoder_t *enc = __containerof(encoder, buzzer_encoder_t, base);
        size_t encoded_symbols = 0;
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;

        uint32_t half = enc->cycle_ticks / 2;

        while(true) {

            // Finite note complete?
            if(enc->total_cycles > 0 && enc->cycle_index >= enc->total_cycles) {
                *ret_state = RMT_ENCODING_COMPLETE;
                return encoded_symbols;
            }

            // ── ADSR amplitude (0..1000) ────────────────────────

            uint32_t idx = enc->cycle_index;
            uint32_t amplitude;

            uint32_t atk_end = enc->attack_cycles;
            uint32_t decay_end = atk_end + enc->decay_cycles;
            // Sustain ends where release begins (only for finite notes)
            uint32_t sus_end = (enc->total_cycles > 0) ? enc->total_cycles - enc->release_cycles : 0xFFFFFFFF;

            if(idx < atk_end && atk_end > 0) {
                // Attack: ramp 0 → 1000
                uint32_t lut_idx = idx * 255 / atk_end;
                amplitude = cosine_ramp[lut_idx];

            } else if(idx < decay_end && enc->decay_cycles > 0) {
                // Decay: ramp 1000 → sustain
                uint32_t phase = idx - atk_end;
                uint32_t lut_idx = phase * 255 / enc->decay_cycles;
                amplitude = 1000 - (uint32_t)(1000 - enc->sustain_permil) * cosine_ramp[lut_idx] / 1000;

            } else if(idx < sus_end) {
                // Sustain: constant level
                amplitude = enc->sustain_permil;

            } else if(enc->total_cycles > 0) {
                // Release: ramp sustain → 0
                uint32_t phase = idx - sus_end;
                uint32_t rel_len = enc->release_cycles;
                if(rel_len > 0) {
                    uint32_t lut_idx = phase * 255 / rel_len;
                    if(lut_idx > 255)
                        lut_idx = 255;
                    amplitude = (uint32_t)enc->sustain_permil * (1000 - cosine_ramp[lut_idx]) / 1000;
                } else {
                    amplitude = 0;
                }

            } else {
                amplitude = enc->sustain_permil;
            }

            // ── Scale by volume ─────────────────────────────────

            uint32_t amp = amplitude * enc->volume_permil / 1000;

            // ── Compute symbol durations ────────────────────────

            uint32_t high_ticks = amp * half / 1000;

            // Minimum 1 tick high to avoid degenerate RMT symbols
            if(high_ticks < 1)
                high_ticks = 1;

            uint32_t low_ticks = enc->cycle_ticks - high_ticks;
            // Clamp to 15-bit max (only matters for very low freq + low duty)
            if(low_ticks > 32767)
                low_ticks = 32767;

            rmt_symbol_word_t symbol = {};
            symbol.duration0 = high_ticks;
            symbol.level0 = 1;
            symbol.duration1 = low_ticks;
            symbol.level1 = 0;

            // ── Write symbol via copy encoder ───────────────────

            size_t ret = enc->copy_encoder->encode(enc->copy_encoder, channel, &symbol, sizeof(symbol), &session_state);
            encoded_symbols += ret;

            if(session_state & RMT_ENCODING_MEM_FULL) {
                *ret_state = RMT_ENCODING_MEM_FULL;
                return encoded_symbols;
            }

            enc->cycle_index++;
        }
    }

    static esp_err_t buzzer_encoder_reset(rmt_encoder_t *encoder)
    {
        buzzer_encoder_t *enc = __containerof(encoder, buzzer_encoder_t, base);
        enc->cycle_index = 0;
        return enc->copy_encoder->reset(enc->copy_encoder);
    }

    static esp_err_t buzzer_encoder_del(rmt_encoder_t *encoder)
    {
        buzzer_encoder_t *enc = __containerof(encoder, buzzer_encoder_t, base);
        enc->copy_encoder->del(enc->copy_encoder);
        free(enc);
        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // RMT handles and task state

    static rmt_channel_handle_t rmt_channel = nullptr;
    static buzzer_encoder_t *buzzer_enc = nullptr;

    //////////////////////////////////////////////////////////////////////
    // Command queue

    enum cmd_type_t
    {
        CMD_TONE,
        CMD_MELODY,
        CMD_STOP
    };

    struct cmd_t
    {
        cmd_type_t type;
        float freq_hz;           // CMD_TONE
        float amplitude;         // CMD_TONE  (0.0–1.0)
        uint16_t duration_ms;    // CMD_TONE  (0 = indefinite)
        uint16_t attack_ms;      // CMD_TONE
        uint16_t decay_ms;       // CMD_TONE
        float sustain_level;     // CMD_TONE  (0.0–1.0)
        uint16_t release_ms;     // CMD_TONE
        note_t const *notes;     // CMD_MELODY
        int count;               // CMD_MELODY
        bool loop;               // CMD_MELODY
    };

    static QueueHandle_t cmd_queue = nullptr;
    static TaskHandle_t task_handle = nullptr;
    static volatile bool buzzer_playing = false;

    //////////////////////////////////////////////////////////////////////
    // configure_encoder — task context, called before rmt_transmit.
    // No race: DMA hasn't started yet when this runs.

    static void configure_encoder(float freq_hz, uint16_t on_ms, uint16_t attack_ms, uint16_t decay_ms, float sustain_level,
                                  uint16_t release_ms, float volume)
    {
        uint32_t freq = (uint32_t)(freq_hz + 0.5f);

        buzzer_enc->cycle_ticks = RMT_RESOLUTION_HZ / freq;
        buzzer_enc->total_cycles = on_ms > 0 ? freq * on_ms / 1000 : 0;
        buzzer_enc->attack_cycles = freq * attack_ms / 1000;
        buzzer_enc->decay_cycles = freq * decay_ms / 1000;
        buzzer_enc->release_cycles = on_ms > 0 ? freq * release_ms / 1000 : 0;
        buzzer_enc->sustain_permil = (uint16_t)(sustain_level * 1000.0f + 0.5f);
        buzzer_enc->volume_permil = (uint16_t)(volume * 1000.0f + 0.5f);
        buzzer_enc->cycle_index = 0;

        // Ensure attack + decay + release ≤ total_cycles
        if(buzzer_enc->total_cycles > 0) {
            uint32_t adsr = buzzer_enc->attack_cycles + buzzer_enc->decay_cycles + buzzer_enc->release_cycles;
            if(adsr > buzzer_enc->total_cycles) {
                buzzer_enc->attack_cycles = buzzer_enc->attack_cycles * buzzer_enc->total_cycles / adsr;
                buzzer_enc->decay_cycles = buzzer_enc->decay_cycles * buzzer_enc->total_cycles / adsr;
                buzzer_enc->release_cycles = buzzer_enc->total_cycles - buzzer_enc->attack_cycles - buzzer_enc->decay_cycles;
            }
        }

        buzzer_enc->copy_encoder->reset(buzzer_enc->copy_encoder);
    }

    //////////////////////////////////////////////////////////////////////
    // on_trans_done — ISR callback, notifies task when DMA finishes.

    static bool IRAM_ATTR on_trans_done(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx)
    {
        BaseType_t higher_prio_woken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &higher_prio_woken);
        return higher_prio_woken == pdTRUE;
    }

    //////////////////////////////////////////////////////////////////////
    // abort_transmission — stop active RMT output immediately.

    static void abort_transmission()
    {
        rmt_disable(rmt_channel);
        rmt_enable(rmt_channel);
        gpio_set_level(BUZZER_GPIO, 0);    // safety: ensure MOSFET off
    }

    //////////////////////////////////////////////////////////////////////
    // silence — wait up to gap_ms doing nothing (output off).
    // Returns true and re-queues the command if one arrives early.

    static bool silence(uint16_t gap_ms)
    {
        if(gap_ms == 0)
            return false;
        cmd_t cmd;
        if(xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(gap_ms)) == pdPASS) {
            xQueueSendToFront(cmd_queue, &cmd, 0);
            return true;
        }
        return false;
    }

    //////////////////////////////////////////////////////////////////////
    // play_rmt_note — start DMA playback and poll for completion
    //                 or preemption.
    //
    // Returns true  if a command arrived (playback interrupted).
    // Returns false if the note completed normally.

    static bool play_rmt_note(float freq_hz, uint16_t on_ms, uint16_t attack_ms, uint16_t decay_ms, float sustain_level,
                              uint16_t release_ms, float volume)
    {
        configure_encoder(freq_hz, on_ms, attack_ms, decay_ms, sustain_level, release_ms, volume);

        // Clear any pending notification from a previous transmission
        ulTaskNotifyTake(pdTRUE, 0);

        uint8_t dummy = 0;
        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;         // single pass; encoder controls length
        tx_config.flags.eot_level = 0;    // GPIO low when transmission ends

        ESP_LOG_ERR(rmt_transmit(rmt_channel, &buzzer_enc->base, &dummy, sizeof(dummy), &tx_config));

        // Poll loop: 10 ms resolution for preemption
        while(true) {
            cmd_t cmd;
            if(xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdPASS) {
                // Preempted by new command — abort and re-queue
                abort_transmission();
                xQueueSendToFront(cmd_queue, &cmd, 0);
                return true;
            }

            // Check if transmission completed (ISR notification)
            if(ulTaskNotifyTake(pdTRUE, 0) > 0) {
                return false;    // note completed normally
            }
        }
    }

    //////////////////////////////////////////////////////////////////////
    // buzzer_task — outer loop dispatches commands from cmd_queue.

    static void buzzer_task(void *)
    {
        LOG_INFO("started on core %d", (int)xPortGetCoreID());

        while(true) {
            cmd_t cmd;
            xQueueReceive(cmd_queue, &cmd, portMAX_DELAY);

            if(cmd.type == CMD_STOP) {
                abort_transmission();
                continue;
            }

            buzzer_playing = true;

            if(cmd.type == CMD_TONE) {
                LOG_INFO("tone %.1f Hz  amp %.0f%%  dur %u ms", (double)cmd.freq_hz, (double)(cmd.amplitude * 100.0f),
                         (unsigned)cmd.duration_ms);

                uint16_t rel = cmd.duration_ms > 0 ? cmd.release_ms : 0;
                play_rmt_note(cmd.freq_hz, cmd.duration_ms, cmd.attack_ms, cmd.decay_ms, cmd.sustain_level, rel, cmd.amplitude);

            } else {    // CMD_MELODY

                LOG_INFO("melody %d notes loop=%d", cmd.count, (int)cmd.loop);

                bool interrupted = false;

                do {
                    for(int i = 0; i < cmd.count && !interrupted; ++i) {
                        note_t const &n = cmd.notes[i];

                        if(n.freq_hz <= 0.0f) {
                            // Rest: combined on_ms + gap_ms of silence.
                            interrupted = silence((uint16_t)(n.on_ms + n.gap_ms));
                        } else {
                            interrupted = play_rmt_note(n.freq_hz, n.on_ms, n.attack_ms, n.decay_ms, n.sustain_level, n.release_ms, 1.0f);

                            if(!interrupted) {
                                interrupted = silence(n.gap_ms);
                            }
                        }
                    }
                } while(cmd.loop && !interrupted);
            }

            // Ensure GPIO is low after any playback
            gpio_set_level(BUZZER_GPIO, 0);
            buzzer_playing = false;
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void buzzer_init()
{
    init_cosine_ramp();

    // Configure RMT TX channel with DMA
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = BUZZER_GPIO;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = RMT_MEM_SYMBOLS;
    tx_chan_config.trans_queue_depth = 4;
    tx_chan_config.flags.with_dma = true;
    ESP_LOG_ERR(rmt_new_tx_channel(&tx_chan_config, &rmt_channel));

    // Create custom encoder
    buzzer_enc = (buzzer_encoder_t *)heap_caps_calloc(1, sizeof(buzzer_encoder_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ASSERT(buzzer_enc != nullptr);

    buzzer_enc->base.encode = buzzer_encode;
    buzzer_enc->base.reset = buzzer_encoder_reset;
    buzzer_enc->base.del = buzzer_encoder_del;

    rmt_copy_encoder_config_t copy_config = {};
    ESP_LOG_ERR(rmt_new_copy_encoder(&copy_config, &buzzer_enc->copy_encoder));

    // Create command queue + task (task blocks on queue, safe to create before rmt_enable)
    cmd_queue = xQueueCreate(4, sizeof(cmd_t));
    ASSERT(cmd_queue != nullptr);

    xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 3072, nullptr, 5, &task_handle, 0);

    // Register done callback (must happen before rmt_enable)
    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = on_trans_done;
    ESP_LOG_ERR(rmt_tx_register_event_callbacks(rmt_channel, &callbacks, (void *)task_handle));

    ESP_LOG_ERR(rmt_enable(rmt_channel));

    LOG_INFO("init: RMT+DMA, %lu Hz resolution, %u symbol buffer", (unsigned long)RMT_RESOLUTION_HZ, (unsigned)RMT_MEM_SYMBOLS);
}

//////////////////////////////////////////////////////////////////////

void buzzer_play_tone(float freq_hz, float amplitude, uint16_t duration_ms, uint16_t attack_ms, uint16_t decay_ms, float sustain_level,
                      uint16_t release_ms)
{
    cmd_t cmd{};
    cmd.type = CMD_TONE;
    cmd.freq_hz = freq_hz;
    cmd.amplitude = amplitude;
    cmd.duration_ms = duration_ms;
    cmd.attack_ms = attack_ms;
    cmd.decay_ms = decay_ms;
    cmd.sustain_level = sustain_level;
    cmd.release_ms = release_ms;
    xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
}

void buzzer_play_melody(note_t const *notes, int count, bool loop)
{
    cmd_t cmd{};
    cmd.type = CMD_MELODY;
    cmd.notes = notes;
    cmd.count = count;
    cmd.loop = loop;
    xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
}

void buzzer_stop()
{
    cmd_t cmd{};
    cmd.type = CMD_STOP;
    xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
}

bool buzzer_is_playing()
{
    return buzzer_playing;
}

//////////////////////////////////////////////////////////////////////

namespace
{
    struct : console_command_t<"tone", "test buzzer", "<freq> [amp% [dur [atk dec sus% rel]]] | melody <n> | stop">
    {
        void on_command(int argc, char **argv) override
        {
            if(argc < 2 || strcmp(argv[1], "stop") == 0) {
                buzzer_stop();
                return;
            }
            if(strcmp(argv[1], "melody") == 0) {
                int idx = (argc >= 3) ? atoi(argv[2]) : 0;
                if(idx < 0 || idx >= (int)melodies::count) {
                    printf("melody index out of range (0..%d)\n", (int)melodies::count - 1);
                    return;
                }
                auto const &m = melodies::table[idx];
                buzzer_play_melody(m.notes, m.count, false);
                return;
            }
            float freq = (float)atof(argv[1]);
            if(freq < 100.0f || freq > 10000.0f) {
                printf("freq must be 100–10000 Hz\n");
                return;
            }
            float amp = (argc >= 3) ? (float)atof(argv[2]) : 100.0f;
            amp = std::clamp(amp, 1.0f, 100.0f) * 0.01f;
            uint16_t dur = (argc >= 4) ? (uint16_t)atoi(argv[3]) : 0;
            uint16_t atk = (argc >= 5) ? (uint16_t)atoi(argv[4]) : 15;
            uint16_t dec = (argc >= 6) ? (uint16_t)atoi(argv[5]) : 0;
            float sus = (argc >= 7) ? std::clamp((float)atof(argv[6]), 0.0f, 100.0f) * 0.01f : 1.0f;
            uint16_t rel = (argc >= 8) ? (uint16_t)atoi(argv[7]) : 30;
            buzzer_play_tone(freq, amp, dur, atk, dec, sus, rel);
        }
    } tone_cmd;
}    // namespace
