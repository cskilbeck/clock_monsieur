//////////////////////////////////////////////////////////////////////
//
// Buzzer driver — LEDC square-wave drive
//                 with melody sequencer and volume envelope
//
// Signal path
// ───────────
//  ESP32-S3 GPIO (LEDC) ──► R4 (1k5) ──► BSS138 gate ──► BSS138 drain ──► buzzer ──► +5V
//
//  The LEDC peripheral generates a square wave at the desired tone
//  frequency.  The MOSFET switches the buzzer coil on/off at that
//  rate; the coil's inductance and the diaphragm's mechanical
//  response naturally smooth the harmonics.
//
//  MLT-7525 buzzer: rated 3.6 V (range 2.5–4.5 V), 95 mA, 2.7 kHz.
//  50% duty square wave at 5 V supply ≈ 93 mA RMS — within rating.
//
// Volume envelope
// ───────────────
//  Each note carries attack_ms and release_ms durations.  The
//  LEDC duty cycle is modulated every ~5 ms following a
//  raised-cosine curve (zero derivative at both endpoints).
//
// Melody sequencer
// ────────────────
//  A FreeRTOS queue carries CMD_TONE / CMD_MELODY / CMD_STOP.
//  Any queued command preempts the current playback (detected via
//  check_stop() at each envelope-update cycle).  The silence()
//  helper doubles as a gap timer that also responds to incoming
//  commands.
//
//////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/ledc.h"

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
    // LEDC configuration

    static constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_t LEDC_TMR = LEDC_TIMER_0;
    static constexpr ledc_channel_t LEDC_CHAN = LEDC_CHANNEL_0;
    static constexpr ledc_timer_bit_t DUTY_RES = LEDC_TIMER_10_BIT;
    static constexpr uint32_t DUTY_HALF = (1u << 10) / 2;    // 512 = 50% duty

    //////////////////////////////////////////////////////////////////////
    // Envelope update interval

    static constexpr TickType_t ENVELOPE_TICKS = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;

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
        note_t const *notes;     // CMD_MELODY
        int count;               // CMD_MELODY
        bool loop;               // CMD_MELODY
    };

    static QueueHandle_t cmd_queue = nullptr;
    static TaskHandle_t task_handle = nullptr;

    //////////////////////////////////////////////////////////////////////
    // LEDC helpers

    static void set_freq(float freq_hz)
    {
        ledc_timer_pause(LEDC_MODE, LEDC_TMR);
        ledc_set_freq(LEDC_MODE, LEDC_TMR, (uint32_t)(freq_hz + 0.5f));
        ledc_timer_rst(LEDC_MODE, LEDC_TMR);
        ledc_timer_resume(LEDC_MODE, LEDC_TMR);
    }

    static void set_amplitude(float amplitude)
    {
        if(amplitude < 0.002f) {
            // Disconnect output to avoid duty=0 / hpoint=0 glitch pulses
            ledc_stop(LEDC_MODE, LEDC_CHAN, 0);
            return;
        }
        uint32_t duty = (uint32_t)(amplitude * DUTY_HALF + 0.5f);
        if(duty == 0) duty = 1;
        ledc_set_duty(LEDC_MODE, LEDC_CHAN, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHAN);
    }

    static void stop_output()
    {
        ledc_stop(LEDC_MODE, LEDC_CHAN, 0);
    }

    //////////////////////////////////////////////////////////////////////
    // envelope — raised-cosine amplitude [0.0, 1.0] for elapsed_ms into note.
    //
    // Timeline:
    //   [0 .. attack_ms)                → ramp up   (1 - cos(πt/T)) / 2
    //   [attack_ms .. on_ms-release_ms) → sustain  1.0
    //   [on_ms-release_ms .. on_ms)     → ramp down (1 + cos(πt/T)) / 2
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
    // silence — wait up to gap_ms doing nothing (output off).
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
    // play_note — modulate LEDC duty for one note (or indefinitely).
    //
    // note == nullptr  → full amplitude, effectively no time limit
    //                    (caller uses a synthetically large on_ms)
    // note != nullptr  → raised-cosine envelope, exits at note->on_ms
    //
    // Returns true  if a command arrived (playback interrupted).
    // Returns false if the note completed its full on_ms normally.

    static bool play_note(note_t const *note, float volume = 1.0f)
    {
        TickType_t const note_start = xTaskGetTickCount();

        while(true) {

            if(check_stop()) {
                return true;
            }

            uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - note_start) * portTICK_PERIOD_MS);

            if(note && elapsed_ms >= note->on_ms) {
                break;
            }

            float amp = note ? envelope(elapsed_ms, *note) : 1.0f;
            set_amplitude(amp * volume);

            vTaskDelay(ENVELOPE_TICKS);
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

            if(cmd.type == CMD_TONE) {
                uint16_t dur = cmd.duration_ms > 0 ? cmd.duration_ms : 65535;
                LOG_INFO("tone %.1f Hz  amp %.0f%%  dur %u ms", (double)cmd.freq_hz, (double)(cmd.amplitude * 100.0f), (unsigned)dur);

                set_freq(cmd.freq_hz);

                // 15 ms attack prevents click at tone start.
                // 30 ms release softens tone end (needs ≥3 tick periods
                // at configTICK_RATE_HZ=100 to ramp down meaningfully).
                uint16_t rel = cmd.duration_ms > 0 ? 30 : 0;
                note_t const tn = { cmd.freq_hz, dur, 15, rel, 0 };
                play_note(&tn, cmd.amplitude);

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
                            set_freq(n.freq_hz);

                            interrupted = play_note(&n);

                            if(!interrupted) {
                                stop_output();
                                interrupted = silence(n.gap_ms);
                            }
                        }
                    }
                } while(cmd.loop && !interrupted);
            }

            stop_output();
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void buzzer_init()
{
    ledc_timer_config_t timer_cfg{};
    timer_cfg.speed_mode = LEDC_MODE;
    timer_cfg.duty_resolution = DUTY_RES;
    timer_cfg.timer_num = LEDC_TMR;
    timer_cfg.freq_hz = 2700;    // default; changed per-note
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_LOG_ERR(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t chan_cfg{};
    chan_cfg.gpio_num = BUZZER_GPIO;
    chan_cfg.speed_mode = LEDC_MODE;
    chan_cfg.channel = LEDC_CHAN;
    chan_cfg.timer_sel = LEDC_TMR;
    chan_cfg.duty = 0;
    chan_cfg.hpoint = 0;
    ESP_LOG_ERR(ledc_channel_config(&chan_cfg));

    cmd_queue = xQueueCreate(4, sizeof(cmd_t));
    ASSERT(cmd_queue != nullptr);

    xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 3072, nullptr, 5, &task_handle, 0);

    LOG_INFO("init: LEDC square wave, %d-bit duty", (int)DUTY_RES);
}

//////////////////////////////////////////////////////////////////////

void buzzer_play_tone(float freq_hz, float amplitude, uint16_t duration_ms)
{
    cmd_t cmd{};
    cmd.type = CMD_TONE;
    cmd.freq_hz = freq_hz;
    cmd.amplitude = amplitude;
    cmd.duration_ms = duration_ms;
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
    struct : console_command_t<"tone", "test buzzer", "<freq> [amp dur_ms] | melody <n> | stop">
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
            float amp = (argc >= 3) ? (float)atof(argv[2]) : 100.0f;
            amp = std::clamp(amp, 1.0f, 100.0f) * 0.01f;
            uint16_t dur = (argc >= 4) ? (uint16_t)atoi(argv[3]) : 0;
            buzzer_play_tone(freq, amp, dur);
        }
    } tone_cmd;
}    // namespace
