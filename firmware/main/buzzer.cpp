//////////////////////////////////////////////////////////////////////
//
// Buzzer driver — RMT DMA double-buffered sine synthesis
//                 with melody sequencer and volume envelope
//
// Signal path
// ───────────
//  ESP32-S3 GPIO ──► R4 (1k5) ──► BSS138 gate ──► BSS138 drain ──► buzzer ──► +5V
//
//  The MOSFET is switched at ~300 kHz (RMT carrier).  Each carrier
//  cycle's duty cycle is modulated to trace a sine wave at the desired
//  tone frequency.  The buzzer coil low-pass filters the 300 kHz
//  switching, recovering the audio-frequency sine envelope.
//
// Double-buffer / DMA
// ───────────────────
//  Two DMA-capable buffers (each ≈ 5 ms of carrier cycles) are
//  alternated.  One is streamed to the GPIO by RMT/DMA without CPU
//  involvement; the other is being refilled by buzzer_task.
//  on_trans_done (ISR) increments a counting FreeRTOS notification;
//  the task decrements it one-for-one (pdFALSE mode) so that two
//  rapid completions after a WiFi preemption are each serviced
//  individually and no refill is missed.
//
// WiFi resilience (Core 0)
// ────────────────────────
//  DMA runs independently of the CPU.  The 5 ms buffer window and
//  a pre-queued second buffer (trans_queue_depth=2) give 10 ms of
//  headroom — larger than any typical WiFi CPU burst (~1–3 ms).
//  The task runs at priority 5; resilience comes from buffer depth,
//  not task priority.
//
// Volume envelope
// ───────────────
//  Each note carries attack_ms and release_ms durations.  The
//  amplitude applied to fill_buffer follows a raised-cosine curve
//  (zero derivative at both endpoints), computed from elapsed time
//  at each buffer-fill step (~5 ms granularity — imperceptible for
//  envelopes of 20 ms or more).
//
// Melody sequencer
// ────────────────
//  A FreeRTOS queue carries CMD_TONE / CMD_MELODY / CMD_STOP.
//  Any queued command preempts the current playback (detected via
//  check_stop() at each buffer-fill cycle).  The silence() helper
//  doubles as a gap timer that also responds to incoming commands.
//
//////////////////////////////////////////////////////////////////////

#include <cmath>
#include <cstdlib>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
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
    // RMT / carrier

    static constexpr uint32_t RMT_RESOLUTION_HZ = 80'000'000;    // APB clock
    static constexpr uint16_t CARRIER_PERIOD_TICKS = 267;        // 80e6/267 ≈ 299,625 Hz
    static constexpr float CARRIER_FREQ_HZ = (float)RMT_RESOLUTION_HZ / CARRIER_PERIOD_TICKS;

    // Keep duration0 and duration1 both ≥ 1; 0 is the RMT end-of-sequence marker.
    static constexpr int DUTY_MIN = 1;
    static constexpr int DUTY_MAX = CARRIER_PERIOD_TICKS - 1;    // 266
    static constexpr float DUTY_CENTRE = CARRIER_PERIOD_TICKS * 0.5f;
    static constexpr float DUTY_AMPLITUDE = DUTY_CENTRE - DUTY_MIN - 0.5f;    // 132.0

    //////////////////////////////////////////////////////////////////////
    // Sine LUT — unit values in [-1.0, 1.0]; amplitude is applied at fill time
    // so a single table serves all volume levels without regeneration.

    static constexpr int SINE_LUT_SIZE = 256;
    static float sine_lut[SINE_LUT_SIZE];

    //////////////////////////////////////////////////////////////////////
    // Double buffers
    //
    // 1500 items × (1 / 299,625 Hz) ≈ 5.0 ms per buffer.
    // At 2 kHz that is 10 full sine cycles — comfortably more than any
    // WiFi-induced CPU gap on Core 0.

    static constexpr size_t ITEMS_PER_BUFFER = 1500;
    static rmt_symbol_word_t *buffers[2];    // allocated from internal DMA DRAM
    static int fill_idx;                     // which buffer to refill next

    //////////////////////////////////////////////////////////////////////
    // Phase accumulator: Q16.16 fixed-point
    //   upper 8 bits  → sine_lut index [0, 255]
    //   lower 16 bits → sub-sample fraction

    static constexpr uint32_t PHASE_SCALE = (uint32_t)SINE_LUT_SIZE << 16;    // 0x01000000
    static uint32_t phase_acc;
    static uint32_t phase_step;

    //////////////////////////////////////////////////////////////////////
    // RMT handles

    static rmt_channel_handle_t tx_chan = nullptr;
    static rmt_encoder_handle_t encoder = nullptr;
    static rmt_transmit_config_t tx_config{};

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
        float freq_hz;          // CMD_TONE
        note_t const *notes;    // CMD_MELODY
        int count;              // CMD_MELODY
        bool loop;              // CMD_MELODY
    };

    static QueueHandle_t cmd_queue = nullptr;
    static TaskHandle_t task_handle = nullptr;

    //////////////////////////////////////////////////////////////////////
    // fill_buffer — write ITEMS_PER_BUFFER carrier cycles with
    // amplitude-scaled sine.  Phase accumulator advances and wraps
    // so tone is phase-continuous across buffer boundaries.

    static void fill_buffer(rmt_symbol_word_t *buf, float amplitude)
    {
        float const scale = amplitude * DUTY_AMPLITUDE;
        for(size_t i = 0; i < ITEMS_PER_BUFFER; ++i) {
            uint8_t idx = (uint8_t)(phase_acc >> 16);
            int duty = (int)(DUTY_CENTRE + scale * sine_lut[idx] + 0.5f);
            duty = std::clamp(duty, DUTY_MIN, DUTY_MAX);
            buf[i].duration0 = (uint16_t)duty;
            buf[i].level0 = 1;
            buf[i].duration1 = CARRIER_PERIOD_TICKS - (uint16_t)duty;
            buf[i].level1 = 0;
            phase_acc += phase_step;
            if(phase_acc >= PHASE_SCALE) {
                phase_acc -= PHASE_SCALE;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////
    // envelope — raised-cosine amplitude [0.0, 1.0] for elapsed_ms into note.
    //
    // Timeline:
    //   [0 .. attack_ms)              → ramp up   (1 - cos(πt/T)) / 2
    //   [attack_ms .. on_ms-release_ms) → sustain  1.0
    //   [on_ms-release_ms .. on_ms)   → ramp down (1 + cos(πt/T)) / 2
    //
    // If attack+release > on_ms they are clamped to on_ms/2 each.

    static float envelope(uint32_t elapsed_ms, note_t const &n)
    {
        uint32_t atk = n.attack_ms;
        uint32_t rel = n.release_ms;
        uint32_t on = n.on_ms;

        if(atk + rel > on) {
            atk = on / 2;
            rel = on - atk;
        }

        if(atk > 0 && elapsed_ms < atk) {
            float t = (float)elapsed_ms / (float)atk;
            return (1.0f - cosf((float)M_PI * t)) * 0.5f;
        }

        uint32_t rel_start = on - rel;
        if(rel > 0 && elapsed_ms >= rel_start) {
            if(elapsed_ms >= on) {
                return 0.0f;
            }
            float t = (float)(elapsed_ms - rel_start) / (float)rel;
            return (1.0f + cosf((float)M_PI * t)) * 0.5f;
        }

        return 1.0f;    // sustain
    }

    //////////////////////////////////////////////////////////////////////
    // transmit — concise wrapper used throughout

    static inline void transmit(rmt_symbol_word_t *buf)
    {
        ESP_LOG_ERR(rmt_transmit(tx_chan, encoder, buf, ITEMS_PER_BUFFER * sizeof(rmt_symbol_word_t), &tx_config));
    }

    //////////////////////////////////////////////////////////////////////
    // on_trans_done ISR — counting-mode notify.
    // Each completion increments the counter by 1; ulTaskNotifyTake with
    // pdFALSE decrements by 1, so two rapid completions after a WiFi burst
    // result in two separate wake-ups and two buffer refills.

    IRAM_ATTR static bool on_trans_done(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *)
    {
        BaseType_t hp = pdFALSE;
        vTaskNotifyGiveFromISR(task_handle, &hp);
        return hp == pdTRUE;
    }

    //////////////////////////////////////////////////////////////////////
    // drain_tail — wait for all queued buffers to finish between melody
    // notes.  RMT stays enabled; eot_level=0 pulls GPIO low automatically.

    static void drain_tail()
    {
        rmt_tx_wait_all_done(tx_chan, pdMS_TO_TICKS(50));
        xTaskNotifyStateClear(nullptr);
    }

    //////////////////////////////////////////////////////////////////////
    // stop_rmt — drain, disable, force GPIO low, clear stale notifications.
    // Must be called once after every rmt_enable().

    static void stop_rmt()
    {
        drain_tail();
        rmt_disable(tx_chan);
        gpio_set_level(BUZZER_GPIO, 0);
    }

    //////////////////////////////////////////////////////////////////////
    // check_stop — non-blocking peek at the command queue.
    // If any command is pending, puts it back at the front and returns true.
    // Any pending command (CMD_STOP, CMD_MELODY, CMD_TONE) interrupts
    // current playback so the outer loop can process it.

    static bool check_stop()
    {
        cmd_t cmd;
        if(xQueueReceive(cmd_queue, &cmd, 0) == pdPASS) {
            xQueueSendToFront(cmd_queue, &cmd, 0);
            return true;
        }
        return false;
    }

    //////////////////////////////////////////////////////////////////////
    // silence — wait up to gap_ms doing nothing (RMT idle, GPIO low).
    // Returns true and re-queues the command if one arrives early.
    // Used both for inter-note gaps and for rest notes.

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
    // play_note — stream sine buffers for one note (or indefinitely).
    //
    // note == nullptr  → full amplitude, effectively no time limit
    //                    (caller uses a synthetically large on_ms)
    // note != nullptr  → raised-cosine envelope, exits at note->on_ms
    //
    // Returns true  if a command arrived (playback interrupted).
    // Returns false if the note completed its full on_ms normally.
    //
    // Phase accumulator and phase_step must be set by the caller before
    // calling play_note.

    static bool play_note(note_t const *note)
    {
        TickType_t const note_start = xTaskGetTickCount();

        // Prime both buffers before entering the steady-state loop so
        // RMT always has something queued (no gap at note start).
        static constexpr uint32_t BUF1_MS = (uint32_t)(1000.0f * ITEMS_PER_BUFFER / CARRIER_FREQ_HZ + 0.5f);    // ≈ 5

        float amp0 = note ? envelope(0, *note) : 1.0f;
        float amp1 = note ? envelope(BUF1_MS, *note) : 1.0f;

        fill_buffer(buffers[0], amp0);
        transmit(buffers[0]);
        fill_buffer(buffers[1], amp1);
        transmit(buffers[1]);
        fill_idx = 0;

        while(true) {

            if(check_stop()) {
                return true;
            }

            // Wait for a buffer to be consumed.  10 ms timeout is a safety
            // net against an RMT stall; in normal operation trans_done fires
            // every ~5 ms.
            uint32_t got = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(10));

            if(check_stop()) {
                return true;
            }

            // Genuine timeout (got == 0): no buffer was consumed, so do not
            // queue a new one — just retry the wait.
            if(got == 0) {
                continue;
            }

            uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - note_start) * portTICK_PERIOD_MS);

            if(note && elapsed_ms >= note->on_ms) {
                break;
            }

            float amp = note ? envelope(elapsed_ms, *note) : 1.0f;
            fill_buffer(buffers[fill_idx], amp);
            transmit(buffers[fill_idx]);
            fill_idx ^= 1;
        }

        return false;    // note completed normally
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
                continue;    // nothing is playing, nothing to do
            }

            ESP_LOG_ERR(rmt_enable(tx_chan));

            if(cmd.type == CMD_TONE) {
                LOG_INFO("tone %.1f Hz", (double)cmd.freq_hz);

                phase_acc = 0;
                phase_step = (uint32_t)(cmd.freq_hz / CARRIER_FREQ_HZ * PHASE_SCALE + 0.5f);

                // Synthetic note: 15 ms attack prevents click at tone start;
                // on_ms = 65535 is effectively infinite for an alarm clock.
                note_t const tn = { cmd.freq_hz, 65535, 15, 0, 0 };
                play_note(&tn);

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
                            phase_acc = 0;
                            phase_step = (uint32_t)(n.freq_hz / CARRIER_FREQ_HZ * PHASE_SCALE + 0.5f);

                            interrupted = play_note(&n);

                            if(!interrupted) {
                                // Note completed: drain tail, then inter-note gap.
                                drain_tail();
                                interrupted = silence(n.gap_ms);
                            }
                        }
                    }
                } while(cmd.loop && !interrupted);
            }

            stop_rmt();
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void buzzer_init()
{
    // Unit sine LUT in [-1.0, 1.0]; amplitude applied per-buffer in fill_buffer.
    for(int i = 0; i < SINE_LUT_SIZE; ++i) {
        sine_lut[i] = sinf(2.0f * (float)M_PI * i / SINE_LUT_SIZE);
    }

    // DMA-capable buffers must live in internal SRAM (GDMA cannot reach PSRAM).
    for(int i = 0; i < 2; ++i) {
        buffers[i] =
            (rmt_symbol_word_t *)heap_caps_malloc(ITEMS_PER_BUFFER * sizeof(rmt_symbol_word_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ASSERT(buffers[i] != nullptr);
    }


    tx_config.loop_count = 0;
    tx_config.flags.eot_level = 0;    // GPIO low when idle → MOSFET off

    rmt_tx_channel_config_t chan_cfg{};
    chan_cfg.clk_src = RMT_CLK_SRC_APB;    // 80 MHz
    chan_cfg.gpio_num = BUZZER_GPIO;
    chan_cfg.mem_block_symbols = 64;    // internal FIFO depth (min 48 for DMA)
    chan_cfg.resolution_hz = RMT_RESOLUTION_HZ;
    chan_cfg.trans_queue_depth = 2;    // one in-flight + one pre-queued
    chan_cfg.flags.with_dma = 1;
    ESP_LOG_ERR(rmt_new_tx_channel(&chan_cfg, &tx_chan));

    rmt_copy_encoder_config_t enc_cfg{};
    ESP_LOG_ERR(rmt_new_copy_encoder(&enc_cfg, &encoder));

    rmt_tx_event_callbacks_t cbs{};
    cbs.on_trans_done = on_trans_done;
    ESP_LOG_ERR(rmt_tx_register_event_callbacks(tx_chan, &cbs, nullptr));

    gpio_set_level(BUZZER_GPIO, 0);    // MOSFET off while RMT disabled

    cmd_queue = xQueueCreate(4, sizeof(cmd_t));
    ASSERT(cmd_queue != nullptr);

    // Core 0, priority 5: above idle/lux, below WiFi (23) and display (20).
    // Resilience against WiFi preemption comes from buffer depth, not priority.
    xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 3072, nullptr, 5, &task_handle, 0);

    LOG_INFO("init: carrier=%.0f Hz, buf=%zu items (%.1f ms), 2×%zu B DRAM", (double)CARRIER_FREQ_HZ, ITEMS_PER_BUFFER,
             1000.0 * ITEMS_PER_BUFFER / CARRIER_FREQ_HZ, ITEMS_PER_BUFFER * sizeof(rmt_symbol_word_t));
}

//////////////////////////////////////////////////////////////////////

void buzzer_play_tone(float freq_hz)
{
    cmd_t cmd{};
    cmd.type = CMD_TONE;
    cmd.freq_hz = freq_hz;
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

//////////////////////////////////////////////////////////////////////

namespace
{
    struct : console_command_t<"buzzer", "test buzzer", "[freq_hz | melody <n> | stop]">
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
                buzzer_play_melody(m.notes, m.count, true);
                return;
            }
            float freq = (float)atof(argv[1]);
            if(freq < 100.0f || freq > 10000.0f) {
                printf("freq must be 100–10000 Hz\n");
                return;
            }
            buzzer_play_tone(freq);
        }
    } buzzer_cmd;
}    // namespace
