// ============================================================
//  testing/test_lead_screws.cpp
//
//  PURPOSE
//  -------
//  Bring-up test for four NEMA 17 lead-screw steppers wired
//  through individual TB6600 stepper drivers (STEP + DIR only).
//
//  SETUP — fill in PinMap.h first
//  --------------------------------
//  In include/pins/PinMap.h, under namespace LeadScrews, set:
//
//    DEVICE1_SCREW1_STEP / _DIR   — Teensy digital pins
//    DEVICE1_SCREW2_STEP / _DIR
//    DEVICE2_SCREW1_STEP / _DIR
//    DEVICE2_SCREW2_STEP / _DIR
//
//  TB6600 wiring per driver:
//    PUL+ / PUL- — STEP pin (use 3.3 V logic; connect PUL- to GND)
//    DIR+ / DIR- — DIR  pin (same)
//    ENA+ / ENA- — leave disconnected or tie ENA- to GND with ENA+
//                  left floating to keep drivers always enabled
//    Power: 12–24 V motor supply, common GND with Teensy
//
//  MICROSTEPPING — set per driver below
//  --------------------------------------
//  Each TB6600 has its own SW1-SW3 DIP switches that set
//  microstepping independently. Match the values below to your
//  physical switch positions before running any motion commands.
//
//    TB6600 SW1 SW2 SW3  microstep
//    --------------------------------
//    Full      OFF OFF OFF   1
//    Half      ON  OFF OFF   2
//    1/4       OFF ON  OFF   4
//    1/8       ON  ON  OFF   8
//    1/16      OFF OFF ON    16
//    1/32      ON  OFF ON    32
//
//  DIRECTION CONVENTION
//  --------------------
//  "Up" means the platform rises (leadscrew rotates to extend).
//  If a motor moves down when you press U, set its DIR_UP value
//  to LOW instead of HIGH in the config section below.
//
//  COMMANDS (single keypress, no newline needed)
//  ---------------------------------------------
//    1 / 2 / 3 / 4  — Select motor to jog (default: all)
//    A               — Select ALL motors
//    U               — Move selected motor(s) UP by target distance
//    D               — Move selected motor(s) DOWN by target distance
//    + / -           — Increase / decrease target distance (1 mm steps)
//    T               — Auto-test: all motors up → pause → all down
//    ?               — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testLeadScrews_setup();
//    extern void testLeadScrews_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  USER CONFIGURATION  —  edit before first run
// ============================================================

// ---- Microstepping per driver ------------------------------
// Match to your TB6600 DIP switch positions.
// Valid values: 1, 2, 4, 8, 16, 32
static constexpr int MS_SCREW1 = 1;   // Device 1, Screw 1 — TODO: match SW1-SW3
static constexpr int MS_SCREW2 = 1;   // Device 1, Screw 2 — TODO: match SW1-SW3
static constexpr int MS_SCREW3 = 1;   // Device 2, Screw 1 — TODO: match SW1-SW3
static constexpr int MS_SCREW4 = 1;   // Device 2, Screw 2 — TODO: match SW1-SW3

// ---- Direction convention ----------------------------------
// HIGH = platform rises (UP), LOW = platform falls (DOWN).
// If a screw moves the wrong way, flip its entry to LOW / HIGH.
static constexpr bool DIR_UP_SCREW1 = HIGH;
static constexpr bool DIR_UP_SCREW2 = HIGH;
static constexpr bool DIR_UP_SCREW3 = HIGH;
static constexpr bool DIR_UP_SCREW4 = HIGH;

// ---- Default travel distance ------------------------------
static constexpr float DEFAULT_DIST_MM = 15.0f;   // mm to travel per U/D press
static constexpr float DIST_STEP_MM    =  1.0f;   // mm per +/- keypress
static constexpr float DIST_MAX_MM     = 50.0f;   // upper safety cap

// ---- Speed ------------------------------------------------
// Platform travel speed in mm/s.
// T8 leadscrew + NEMA 17 + TB6600: 5–15 mm/s is safe open-loop.
// Reduce if the motor stalls or makes grinding noises.
static constexpr float SPEED_MM_S = 5.0f;

// ============================================================
//  DERIVED CONSTANTS
// ============================================================
// T8 leadscrew: 8 mm/rev (from Constants::LeadScrews::LEAD_MM_PER_REV)
// NEMA 17:     200 full steps/rev
static constexpr float LEAD_MM_PER_REV   = Constants::LeadScrews::LEAD_MM_PER_REV;
static constexpr int   FULL_STEPS_PER_REV = Constants::LeadScrews::MOTOR_STEPS_PER_REV;

// Steps per mm for each screw (microstepping × full-steps / lead)
static constexpr float SPM1 = (FULL_STEPS_PER_REV * MS_SCREW1) / LEAD_MM_PER_REV;
static constexpr float SPM2 = (FULL_STEPS_PER_REV * MS_SCREW2) / LEAD_MM_PER_REV;
static constexpr float SPM3 = (FULL_STEPS_PER_REV * MS_SCREW3) / LEAD_MM_PER_REV;
static constexpr float SPM4 = (FULL_STEPS_PER_REV * MS_SCREW4) / LEAD_MM_PER_REV;

// Step pulse high time (µs) — TB6600 needs ≥ 2.2 µs
static constexpr uint32_t PULSE_US = 5;

// ============================================================
//  MOTOR TABLE
//  Index 0–3 maps to Screws 1–4.
// ============================================================
struct ScrewDef {
    int   stepPin;
    int   dirPin;
    float stepsPerMm;
    bool  dirUp;      // DIR level that means platform-up
    const char* name;
};

static const ScrewDef SCREWS[4] = {
    { Pins::LeadScrews::DEVICE1_SCREW1_STEP,
      Pins::LeadScrews::DEVICE1_SCREW1_DIR,
      SPM1, DIR_UP_SCREW1, "D1-S1" },

    { Pins::LeadScrews::DEVICE1_SCREW2_STEP,
      Pins::LeadScrews::DEVICE1_SCREW2_DIR,
      SPM2, DIR_UP_SCREW2, "D1-S2" },

    { Pins::LeadScrews::DEVICE2_SCREW1_STEP,
      Pins::LeadScrews::DEVICE2_SCREW1_DIR,
      SPM3, DIR_UP_SCREW3, "D2-S1" },

    { Pins::LeadScrews::DEVICE2_SCREW2_STEP,
      Pins::LeadScrews::DEVICE2_SCREW2_DIR,
      SPM4, DIR_UP_SCREW4, "D2-S2" },
};

// ============================================================
//  RUNTIME STATE
// ============================================================
// Bitmask: bit 0 = Screw 1, bit 1 = Screw 2, etc.
// 0b1111 = all four selected.
static uint8_t g_sel      = 0b1111;   // default: all motors
static float   g_distMm   = DEFAULT_DIST_MM;

// ============================================================
//  HELPERS
// ============================================================
static bool screwWired(uint8_t idx) {
    return (SCREWS[idx].stepPin >= 0 && SCREWS[idx].dirPin >= 0);
}

static bool anyWired() {
    for (uint8_t i = 0; i < 4; i++) if (screwWired(i)) return true;
    return false;
}

// Move one screw by `steps` microsteps at `delayBetweenStepsUs` per step.
// Fires STEP pulses via tight loop — blocking for the duration.
static void moveSteps(uint8_t idx, uint32_t steps, bool up,
                      uint32_t delayBetweenStepsUs) {
    if (!screwWired(idx)) return;

    bool dirLevel = up ? SCREWS[idx].dirUp : !SCREWS[idx].dirUp;
    digitalWrite(SCREWS[idx].dirPin, dirLevel ? HIGH : LOW);
    delayMicroseconds(2);   // DIR setup time before first pulse

    for (uint32_t s = 0; s < steps; s++) {
        digitalWrite(SCREWS[idx].stepPin, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(SCREWS[idx].stepPin, LOW);
        if (delayBetweenStepsUs > PULSE_US)
            delayMicroseconds(delayBetweenStepsUs - PULSE_US);
    }
}

// Move all selected screws simultaneously by g_distMm.
// All motors step in lock-step so they travel together.
// Uses the slowest steps/mm motor to set the shared step rate,
// and fires each motor's step pin only when its own step is due.
static void moveSelected(bool up) {
    if (!anyWired()) {
        Serial.println(F("# No screws wired — set STEP/DIR pins in PinMap.h."));
        return;
    }

    // Build list of screws to move
    uint8_t active[4];
    uint8_t activeCount = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if ((g_sel & (1 << i)) && screwWired(i)) {
            active[activeCount++] = i;
        }
    }

    if (activeCount == 0) {
        Serial.println(F("# Selected motor(s) not wired — check pins and selection."));
        return;
    }

    // Print intent
    Serial.print(F("# Moving "));
    for (uint8_t k = 0; k < activeCount; k++) {
        Serial.print(SCREWS[active[k]].name);
        if (k < activeCount - 1) Serial.print(',');
    }
    Serial.print(up ? F(" UP ") : F(" DOWN "));
    Serial.print(g_distMm, 1);
    Serial.println(F(" mm"));

    // Set DIR for all motors first
    for (uint8_t k = 0; k < activeCount; k++) {
        uint8_t i = active[k];
        bool dirLevel = up ? SCREWS[i].dirUp : !SCREWS[i].dirUp;
        digitalWrite(SCREWS[i].dirPin, dirLevel ? HIGH : LOW);
    }
    delayMicroseconds(2);

    // Each motor gets its own step budget and accumulator.
    // We use Bresenham-style step scheduling so motors with
    // different steps/mm still all travel the same physical mm.
    uint32_t totalSteps[4] = {0};
    uint32_t stepsLeft[4]  = {0};
    uint32_t maxTotal = 0;

    for (uint8_t k = 0; k < activeCount; k++) {
        uint8_t i = active[k];
        uint32_t n = static_cast<uint32_t>(g_distMm * SCREWS[i].stepsPerMm + 0.5f);
        totalSteps[k] = n;
        stepsLeft[k]  = n;
        if (n > maxTotal) maxTotal = n;
    }

    if (maxTotal == 0) return;

    // Inter-step delay based on the motor with the MOST steps
    // (fastest stepping motor sets the timing loop rate).
    uint32_t delayUs = static_cast<uint32_t>(
        1000000.0f / (SPEED_MM_S * static_cast<float>(maxTotal) / g_distMm) + 0.5f);
    if (delayUs < PULSE_US) delayUs = PULSE_US;

    // Bresenham accumulators: each motor fires a step when its
    // accumulator ≥ half of maxTotal.
    uint32_t accum[4]  = {0};

    for (uint32_t tick = 0; tick < maxTotal; tick++) {
        for (uint8_t k = 0; k < activeCount; k++) {
            if (stepsLeft[k] == 0) continue;
            accum[k] += totalSteps[k];
            if (accum[k] >= maxTotal) {
                accum[k] -= maxTotal;
                digitalWrite(SCREWS[active[k]].stepPin, HIGH);
                // Step pins are cleared together below
            }
        }
        delayMicroseconds(PULSE_US);
        // Clear all step pins
        for (uint8_t k = 0; k < activeCount; k++) {
            if (stepsLeft[k] > 0) {
                digitalWrite(SCREWS[active[k]].stepPin, LOW);
                // Decrement remaining for motors that actually stepped
            }
        }
        // Count steps fired this tick
        for (uint8_t k = 0; k < activeCount; k++) {
            if (stepsLeft[k] > 0 && accum[k] == 0) {
                // Just fired — this approach is slightly imprecise for counting.
                // Safe to just decrement all equally since each motor's budget
                // is managed by its accumulator ratio.
                stepsLeft[k]--;
            }
        }
        if (delayUs > PULSE_US)
            delayMicroseconds(delayUs - PULSE_US);
    }

    // Final step count confirmation
    for (uint8_t k = 0; k < activeCount; k++) {
        uint8_t i = active[k];
        Serial.print(F("#   ")); Serial.print(SCREWS[i].name);
        Serial.print(F(":  ")); Serial.print(totalSteps[k]);
        Serial.print(F(" steps  @ ")); Serial.print(SCREWS[i].stepsPerMm, 1);
        Serial.println(F(" steps/mm"));
    }
    Serial.println(F("# Done."));
}

// ============================================================
//  AUTO-TEST
//  Moves all wired screws up 15mm, pauses 1 s, then back down.
// ============================================================
static void runAutoTest() {
    if (!anyWired()) {
        Serial.println(F("# SKIP: no screws wired — set STEP/DIR pins in PinMap.h."));
        return;
    }

    Serial.println(F("# === Auto-test start ==="));

    uint8_t savedSel = g_sel;
    float   savedDist = g_distMm;

    g_sel    = 0b1111;
    g_distMm = DEFAULT_DIST_MM;

    // Test each screw individually first
    for (uint8_t i = 0; i < 4; i++) {
        if (!screwWired(i)) {
            Serial.print(F("# Screw ")); Serial.print(i + 1);
            Serial.println(F(" — skipped (pin not set)"));
            continue;
        }
        g_sel = (1 << i);

        Serial.print(F("# ["));
        Serial.print(SCREWS[i].name);
        Serial.print(F("] UP "));
        Serial.print(g_distMm, 1);
        Serial.println(F(" mm"));
        moveSelected(true);
        delay(800);

        Serial.print(F("# ["));
        Serial.print(SCREWS[i].name);
        Serial.print(F("] DOWN "));
        Serial.print(g_distMm, 1);
        Serial.println(F(" mm"));
        moveSelected(false);
        delay(800);
    }

    // Then move all together
    g_sel = 0b1111;
    Serial.println(F("# [ALL] UP together"));
    moveSelected(true);
    delay(1000);
    Serial.println(F("# [ALL] DOWN together"));
    moveSelected(false);

    g_sel    = savedSel;
    g_distMm = savedDist;

    Serial.println(F("# === Auto-test complete ==="));
    Serial.println(F("#"));
    Serial.println(F("# Motor didn't move?  Check 12–24 V supply, DIR+/PUL+ wiring,"));
    Serial.println(F("#   and STEP/DIR pin values in PinMap.h."));
    Serial.println(F("# Motor moves wrong direction?  Flip its DIR_UP_SCREWx"));
    Serial.println(F("#   to LOW in the config section at the top of this file."));
    Serial.println(F("# Motor stalls?  Reduce SPEED_MM_S or increase TB6600 current."));
    Serial.println(F("# Distance wrong?  Check MS_SCREWx matches your TB6600 DIP switches."));
}

// ============================================================
//  HELP / STATUS
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# Lead-Screw NEMA 17 Test  (TB6600 STEP/DIR)"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  1/2/3/4 — Select individual motor"));
    Serial.println(F("#  A       — Select ALL motors"));
    Serial.println(F("#  U       — Move selected UP  by target dist"));
    Serial.println(F("#  D       — Move selected DOWN by target dist"));
    Serial.println(F("#  +/-     — Change target distance (1 mm step)"));
    Serial.println(F("#  T       — Auto-test (each alone, then all)"));
    Serial.println(F("#  ?       — This help"));
    Serial.println(F("# ---------------------------------------------"));

    Serial.print(F("# Target dist: ")); Serial.print(g_distMm, 1); Serial.println(F(" mm"));
    Serial.print(F("# Speed:       ")); Serial.print(SPEED_MM_S, 1); Serial.println(F(" mm/s"));
    Serial.print(F("# Selection:   "));
    if (g_sel == 0b1111) {
        Serial.println(F("ALL"));
    } else {
        for (uint8_t i = 0; i < 4; i++)
            if (g_sel & (1 << i)) { Serial.print(SCREWS[i].name); Serial.print(' '); }
        Serial.println();
    }
    Serial.println(F("# ---------------------------------------------"));

    Serial.println(F("# Screw   STEP  DIR   MS    spm   Wired?"));
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("# ")); Serial.print(SCREWS[i].name);
        Serial.print(F("  ")); Serial.print(SCREWS[i].stepPin);
        Serial.print(F("     ")); Serial.print(SCREWS[i].dirPin);
        // microstepping
        const int msArr[4] = {MS_SCREW1, MS_SCREW2, MS_SCREW3, MS_SCREW4};
        Serial.print(F("     1/")); Serial.print(msArr[i]);
        Serial.print(F("   ")); Serial.print(SCREWS[i].stepsPerMm, 1);
        Serial.print(F("  ")); Serial.println(screwWired(i) ? F("YES") : F("NO (pin=-1)"));
    }
    Serial.print(F("# Leadscrew: ")); Serial.print(LEAD_MM_PER_REV, 1);
    Serial.print(F(" mm/rev  ")); Serial.print(FULL_STEPS_PER_REV);
    Serial.println(F(" full-steps/rev"));
    Serial.println(F("# ============================================="));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testLeadScrews_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    for (uint8_t i = 0; i < 4; i++) {
        if (SCREWS[i].stepPin >= 0) {
            pinMode(SCREWS[i].stepPin, OUTPUT);
            digitalWrite(SCREWS[i].stepPin, LOW);
        }
        if (SCREWS[i].dirPin >= 0) {
            pinMode(SCREWS[i].dirPin, OUTPUT);
        }
    }

    printHelp();
}

void testLeadScrews_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {

        // ---- Motor selection ----
        case '1': case '2': case '3': case '4': {
            uint8_t idx = static_cast<uint8_t>(c - '1');
            g_sel = (1 << idx);
            Serial.print(F("# Selected: ")); Serial.println(SCREWS[idx].name);
            if (!screwWired(idx))
                Serial.println(F("# (not wired — set STEP/DIR pin in PinMap.h)"));
            break;
        }
        case 'a': case 'A':
            g_sel = 0b1111;
            Serial.println(F("# Selected: ALL"));
            break;

        // ---- Motion ----
        case 'u': case 'U':
            moveSelected(true);
            break;
        case 'd': case 'D':
            moveSelected(false);
            break;

        // ---- Distance adjustment ----
        case '+': case '=':
            if (g_distMm + DIST_STEP_MM <= DIST_MAX_MM) g_distMm += DIST_STEP_MM;
            else g_distMm = DIST_MAX_MM;
            Serial.print(F("# Target dist: ")); Serial.print(g_distMm, 1); Serial.println(F(" mm"));
            break;
        case '-': case '_':
            if (g_distMm - DIST_STEP_MM >= DIST_STEP_MM) g_distMm -= DIST_STEP_MM;
            else g_distMm = DIST_STEP_MM;
            Serial.print(F("# Target dist: ")); Serial.print(g_distMm, 1); Serial.println(F(" mm"));
            break;

        case 't': case 'T': runAutoTest(); break;
        case '?':           printHelp();   break;

        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

#endif // TEST_MODE
