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
};

//////////////////////////////////////////////////////////////////////

void buzzer_init();
void buzzer_play_tone(float freq_hz);
void buzzer_play_melody(note_t const *notes, int count, bool loop);
void buzzer_stop();
