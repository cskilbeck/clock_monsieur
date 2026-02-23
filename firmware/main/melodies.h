//////////////////////////////////////////////////////////////////////
//
// Melody definitions for buzzer_play_melody().
//
// Each melody is a constexpr array of note_t.  Fields:
//   freq_hz    — pitch in Hz; 0 = rest
//   on_ms      — total sounding time including attack + release
//   attack_ms  — raised-cosine ramp up
//   release_ms — raised-cosine ramp down
//   gap_ms     — silence after the note (inter-note articulation)
//
// Usage:
//   buzzer_play_melody(melodies::GENTLE, melodies::GENTLE_COUNT, true);
//   // or via the table:
//   auto &m = melodies::table[0];
//   buzzer_play_melody(m.notes, m.count, true);
//
//////////////////////////////////////////////////////////////////////

#pragma once

#include "buzzer.h"

//////////////////////////////////////////////////////////////////////
// Convenience macro — keeps columns aligned in the arrays below.
// N(freq, on, attack, release, gap)

#define N(f, on, atk, rel, gap) \
    note_t { (float)(f), (uint16_t)(on), (uint16_t)(atk), (uint16_t)(rel), (uint16_t)(gap) }

#define REST(ms) \
    note_t { 0.0f, (uint16_t)(ms), 0, 0, 0 }

//////////////////////////////////////////////////////////////////////

namespace melodies
{
    //////////////////////////////////////////////////////////////////////
    // GENTLE — soft ascending arpeggio, pleasant morning wakeup.
    //   C5  E5  G5  C6 (held), pause, repeat.

    static constexpr note_t GENTLE[] = {
        N(523.25f,  180, 20, 30,  15), // C5
        N(659.25f,  180, 15, 30,  15), // E5
        N(783.99f,  180, 15, 30,  15), // G5
        N(1046.50f, 400, 15, 60, 400), // C6 held
    };
    static constexpr int GENTLE_COUNT = sizeof(GENTLE) / sizeof(GENTLE[0]);

    //////////////////////////////////////////////////////////////////////
    // STANDARD — classic double-beep alarm.
    //   Two short high beeps, pause, repeat.

    static constexpr note_t STANDARD[] = {
        N(1760.0f, 160, 10, 20,  30), // A6 beep 1
        N(1760.0f, 160, 10, 20, 350), // A6 beep 2, long gap before repeat
    };
    static constexpr int STANDARD_COUNT = sizeof(STANDARD) / sizeof(STANDARD[0]);

    //////////////////////////////////////////////////////////////////////
    // URGENT — rapid alternating pitches, for a snooze timeout or
    //          second-alarm escalation.

    static constexpr note_t URGENT[] = {
        N(1318.51f, 100,  5, 15, 10), // E6
        N(659.25f,  100,  5, 15, 10), // E5
        N(1318.51f, 100,  5, 15, 10),
        N(659.25f,  100,  5, 15, 10),
        N(1318.51f, 100,  5, 15, 10),
        N(659.25f,  100,  5, 15, 10),
        REST(300),
    };
    static constexpr int URGENT_COUNT = sizeof(URGENT) / sizeof(URGENT[0]);

    //////////////////////////////////////////////////////////////////////
    // CUCKOO — two-note descending cuckoo call.

    static constexpr note_t CUCKOO[] = {
        N(830.61f, 220, 20, 40,  30), // G#5
        N(659.25f, 350, 20, 60, 500), // E5 held, long pause
    };
    static constexpr int CUCKOO_COUNT = sizeof(CUCKOO) / sizeof(CUCKOO[0]);

    //////////////////////////////////////////////////////////////////////
    // Indexed table for console access ("buzzer melody 0", etc.)

    struct entry_t
    {
        char const *name;
        note_t const *notes;
        int           count;
    };

    static constexpr entry_t table[] = {
        { "gentle",   GENTLE,   GENTLE_COUNT   },
        { "standard", STANDARD, STANDARD_COUNT },
        { "urgent",   URGENT,   URGENT_COUNT   },
        { "cuckoo",   CUCKOO,   CUCKOO_COUNT   },
    };
    static constexpr int count = sizeof(table) / sizeof(table[0]);

} // namespace melodies

//////////////////////////////////////////////////////////////////////

#undef N
#undef REST
