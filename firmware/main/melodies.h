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

#define N(f, on, atk, rel, gap)                                                       \
    note_t                                                                            \
    {                                                                                 \
        (float)(f), (uint16_t)(on), (uint16_t)(atk), (uint16_t)(rel), (uint16_t)(gap) \
    }

// NA — note with full ADSR: N + decay_ms + sustain_level (0.0–1.0)
#define NA(f, on, atk, rel, gap, dec, sus)                                                                           \
    note_t                                                                                                           \
    {                                                                                                                \
        (float)(f), (uint16_t)(on), (uint16_t)(atk), (uint16_t)(rel), (uint16_t)(gap), (uint16_t)(dec), (float)(sus) \
    }

#define REST(ms) note_t{ 0.0f, (uint16_t)(ms), 0, 0, 0 }

//////////////////////////////////////////////////////////////////////

namespace melodies
{
    //////////////////////////////////////////////////////////////////////
    // GENTLE — soft ascending arpeggio, pleasant morning wakeup.
    //   C5  E5  G5  C6 (held), pause, repeat.

    static constexpr note_t GENTLE[] = {
        N(523.25f, 180, 20, 30, 15),      // C5
        N(659.25f, 180, 15, 30, 15),      // E5
        N(783.99f, 180, 15, 30, 15),      // G5
        N(1046.50f, 400, 15, 60, 400),    // C6 held
    };
    static constexpr int GENTLE_COUNT = sizeof(GENTLE) / sizeof(GENTLE[0]);

    //////////////////////////////////////////////////////////////////////
    // STANDARD — classic double-beep alarm.
    //   Two short high beeps, pause, repeat.

    static constexpr note_t STANDARD[] = {
        N(1760.0f, 160, 10, 20, 30),     // A6 beep 1
        N(1760.0f, 160, 10, 20, 350),    // A6 beep 2, long gap before repeat
    };
    static constexpr int STANDARD_COUNT = sizeof(STANDARD) / sizeof(STANDARD[0]);

    //////////////////////////////////////////////////////////////////////
    // URGENT — rapid alternating pitches, for a snooze timeout or
    //          second-alarm escalation.

    static constexpr note_t URGENT[] = {
        N(1318.51f, 100, 5, 15, 10),    // E6
        N(659.25f, 100, 5, 15, 10),     // E5
        N(1318.51f, 100, 5, 15, 10), N(659.25f, 100, 5, 15, 10), N(1318.51f, 100, 5, 15, 10), N(659.25f, 100, 5, 15, 10), REST(300),
    };
    static constexpr int URGENT_COUNT = sizeof(URGENT) / sizeof(URGENT[0]);

    //////////////////////////////////////////////////////////////////////
    // CUCKOO — two-note descending cuckoo call.

    static constexpr note_t CUCKOO[] = {
        N(830.61f, 220, 20, 40, 30),     // G#5
        N(659.25f, 350, 20, 60, 500),    // E5 held, long pause
    };
    static constexpr int CUCKOO_COUNT = sizeof(CUCKOO) / sizeof(CUCKOO[0]);

    //////////////////////////////////////////////////////////////////////
    // CHARGE — classic sports-arena bugle fanfare.
    //   C5 E5 G5 C6 (ascending)  G5 (quick)  C6 (held with decay)

    static constexpr note_t CHARGE[] = {
        N(523.25f, 120, 5, 10, 5),                  // C5
        N(659.25f, 120, 5, 10, 5),                  // E5
        N(783.99f, 120, 5, 10, 5),                  // G5
        NA(1046.50f, 200, 5, 15, 5, 40, 0.85f),     // C6 accent, slight decay
        N(783.99f, 90, 5, 10, 5),                   // G5 quick
        NA(1046.50f, 600, 5, 80, 0, 120, 0.50f),    // C6 held, decay to 50%
        REST(500),
    };
    static constexpr int CHARGE_COUNT = sizeof(CHARGE) / sizeof(CHARGE[0]);

    //////////////////////////////////////////////////////////////////////
    // HAPPY_BIRTHDAY — the classic song, key of C.
    //   Four phrases with held endings that decay naturally.

    static constexpr note_t HAPPY_BIRTHDAY[] = {
        // "Hap-py birth-day to you"
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),     // G4  Hap-
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),     // G4  -py
        NA(440.00f, 300, 5, 25, 10, 80, 0.0f),     // A4  birth-
        NA(392.00f, 300, 5, 25, 10, 80, 0.0f),     // G4  -day
        NA(523.25f, 300, 5, 25, 10, 80, 0.0f),     // C5  to
        NA(493.88f, 550, 5, 40, 50, 100, 0.0f),    // B4  you   (rings out)

        // "Hap-py birth-day to you"
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),     // G4  Hap-
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),     // G4  -py
        NA(440.00f, 300, 5, 25, 10, 80, 0.0f),     // A4  birth-
        NA(392.00f, 300, 5, 25, 10, 80, 0.0f),     // G4  -day
        NA(587.33f, 300, 5, 25, 10, 80, 0.0f),     // D5  to
        NA(523.25f, 550, 5, 40, 50, 100, 0.0f),    // C5  you   (rings out)

        // "Hap-py birth-day dear ___"
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),    // G4  Hap-
        NA(392.00f, 150, 5, 15, 10, 50, 0.0f),    // G4  -py
        NA(783.99f, 300, 5, 25, 10, 80, 0.0f),    // G5  birth- (peak!)
        NA(659.25f, 300, 5, 25, 10, 80, 0.0f),    // E5  -day
        NA(523.25f, 300, 5, 25, 10, 80, 0.0f),    // C5  dear
        NA(493.88f, 300, 5, 25, 10, 80, 0.0f),    // B4  [name
        NA(440.00f, 400, 5, 30, 40, 90, 0.0f),    // A4  ___]  (rings out)

        // "Hap-py birth-day to you!"
        NA(698.46f, 150, 5, 15, 10, 50, 0.0f),    // F5  Hap-
        NA(698.46f, 150, 5, 15, 10, 50, 0.0f),    // F5  -py
        NA(659.25f, 300, 5, 25, 10, 80, 0.0f),    // E5  birth-
        NA(523.25f, 300, 5, 25, 10, 80, 0.0f),    // C5  -day
        NA(587.33f, 300, 5, 25, 10, 80, 0.0f),    // D5  to
        NA(523.25f, 900, 5, 60, 0, 150, 0.0f),    // C5  you!  (final strike, long ring)
        REST(600),
    };
    static constexpr int HAPPY_BIRTHDAY_COUNT = sizeof(HAPPY_BIRTHDAY) / sizeof(HAPPY_BIRTHDAY[0]);

    //////////////////////////////////////////////////////////////////////
    // Indexed table for console access ("buzzer melody 0", etc.)

    struct entry_t
    {
        char const *name;
        note_t const *notes;
        int count;
    };

    static constexpr entry_t table[] = {
        { "gentle", GENTLE, GENTLE_COUNT }, { "standard", STANDARD, STANDARD_COUNT }, { "urgent", URGENT, URGENT_COUNT },
        { "cuckoo", CUCKOO, CUCKOO_COUNT }, { "charge", CHARGE, CHARGE_COUNT },       { "birthday", HAPPY_BIRTHDAY, HAPPY_BIRTHDAY_COUNT },
    };
    static constexpr int count = sizeof(table) / sizeof(table[0]);

}    // namespace melodies

//////////////////////////////////////////////////////////////////////

#undef N
#undef NA
#undef REST
