//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

struct note_t
{
    float freq_hz;          // tone frequency in Hz; 0 = rest (silence)
    uint16_t on_ms;         // total sounding time, including attack + release
    uint16_t attack_ms;     // raised-cosine ramp-up at note start
    uint16_t release_ms;    // raised-cosine ramp-down at note end
    uint16_t gap_ms;        // silence after the note (inter-note articulation)
    // ADSR extensions (C++17 default member initializers)
    uint16_t decay_ms = 0;      // ramp from peak to sustain after attack
    float sustain_level = 1.0f; // held level between decay and release (0.0–1.0)
};

//////////////////////////////////////////////////////////////////////

void buzzer_init();
void buzzer_play_tone(float freq_hz, float amplitude = 1.0f, uint16_t duration_ms = 0,
                      uint16_t attack_ms = 15, uint16_t decay_ms = 0,
                      float sustain_level = 1.0f, uint16_t release_ms = 30);
void buzzer_play_melody(note_t const *notes, int count, bool loop);
void buzzer_stop();
