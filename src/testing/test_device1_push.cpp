// ============================================================
//  testing/test_device1_push.cpp
//
//  PURPOSE
//  -------
//  Coordinated bring-up test for the Device 1 push mechanism:
//
//    1. LIFT  — Device 1's two lead screws raise the platform
//               by LIFT_MM at LIFT_SPEED_MM_S.
//    2. SETTLE — Wait SETTLE_MS for the platform to stop vibrating.
//    3. PUSH   — Selected MG996R servo(s) rotate from REST_DEG to
//                PUSH_DEG to sweep a linear bar and push the petri
//                dish to its destination.
//
//  The reverse sequence (RESET) pulls the bar back then lowers
//  the platform, leaving everything ready for the next cycle.
//
//  SETUP — fill in PinMap.h and the config section below
//  -------------------------------------------------------
//  Pins::LeadScrews::DEVICE1_SCREW1_STEP / _DIR
//  Pins::LeadScrews::DEVICE1_SCREW2_STEP / _DIR
//  Pins::Servos::MG996R_1 … MG996R_4  (whichever you use)
//
//  Then set the values in the USER CONFIGURATION section below.
//
//  COMMANDS (single keypress, no newline needed)
//  ---------------------------------------------
//    R  — Run full sequence  (lift → settle → push)
//    H  — Reset / home       (pull back → lower)
//    L  — Lift only  (lead screws up LIFT_MM)
//    D  — Lower only (lead screws down LIFT_MM)
//    P  — Push only  (servo(s) → PUSH_DEG)
//    B  — Pull back  (servo(s) → REST_DEG)
//    ?  — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testDevice1Push_setup();
//    extern void testDevice1Push_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include <Servo.h>
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  USER CONFIGURATION  —  edit these before first run
// ============================================================

// ---- Lead-screw lift distance & speed ----------------------
// How far the platform rises before the push occurs.
static constexpr float LIFT_MM          = 15.0f;  // mm to raise platform
static constexpr float LIFT_SPEED_MM_S  =  5.0f;  // mm/s (keep ≤ 15 to avoid stall)

// ---- Microstepping — match your TB6600 DIP switches --------
// SW1-SW3 positions → 1, 2, 4, 8, 16, or 32
static constexpr int MS_D1S1 = 1;   // Device 1, Screw 1 — TODO: match DIP switch
static constexpr int MS_D1S2 = 1;   // Device 1, Screw 2 — TODO: match DIP switch

// ---- Lead-screw direction sense ----------------------------
// HIGH = platform rises.  Flip to LOW if platform moves down on 'L'.
static constexpr bool DIR_UP_D1S1 = HIGH;
static constexpr bool DIR_UP_D1S2 = HIGH;

// ---- Settle delay between lift and push --------------------
static constexpr uint32_t SETTLE_MS = 500;   // ms to wait after lifting

// ---- Servo push configuration ------------------------------
// Which MG996R servo(s) to use — set bits 0-3 for servos 1-4.
// Examples:  0b0001 = servo 1 only
//            0b0011 = servos 1 and 2 together
//            0b1111 = all four
static constexpr uint8_t SERVO_MASK = 0b0001;  // TODO: set which servo(s) drive the bar

// Servo angle at rest (bar retracted, clear of petri dish).
static constexpr float REST_DEG = 90.0f;       // TODO: set your mechanism's rest angle

// Servo angle at full push (bar extended, petri dish moved).
static constexpr float PUSH_DEG = 45.0f;       // TODO: set your mechanism's push angle

// Time for the servo to travel from REST_DEG to PUSH_DEG.
// The move is swept in small steps over this duration so the
// bar accelerates gently rather than slamming the dish.
static constexpr uint32_t PUSH_TIME_MS  = 300; // ms for full push stroke
static constexpr uint32_t PULL_TIME_MS  = 300; // ms for full pull-back stroke

// MG996R pulse width limits — match your servo batch.
static constexpr int SERVO_MIN_PULSE_US =
    static_cast<int>(Constants::Servos::MG996R_MIN_PULSE_US);
static constexpr int SERVO_MAX_PULSE_US =
    static_cast<int>(Constants::Servos::MG996R_MAX_PULSE_US);

// ============================================================
//  DERIVED CONSTANTS  —  do not edit
// ============================================================
static constexpr float LEAD_MM_PER_REV    = Constants::LeadScrews::LEAD_MM_PER_REV;
static constexpr int   FULL_STEPS_PER_REV = Constants::LeadScrews::MOTOR_STEPS_PER_REV;

static constexpr float SPM_D1S1 =
    static_cast<float>(FULL_STEPS_PER_REV * MS_D1S1) / LEAD_MM_PER_REV;
static constexpr float SPM_D1S2 =
    static_cast<float>(FULL_STEPS_PER_REV * MS_D1S2) / LEAD_MM_PER_REV;

static constexpr uint32_t STEPS_D1S1 =
    static_cast<uint32_t>(LIFT_MM * SPM_D1S1 + 0.5f);
static constexpr uint32_t STEPS_D1S2 =
    static_cast<uint32_t>(LIFT_MM * SPM_D1S2 + 0.5f);

// Step pulse high time (µs); TB6600 needs ≥ 2.2 µs
static constexpr uint32_t PULSE_US = 5;

// ============================================================
//  SERVO INSTANCES
// ============================================================
static Servo  g_servo[4];
static bool   g_attached[4] = {false, false, false, false};
static float  g_servoAngle[4];   // last commanded angle per servo

static int servoPin(uint8_t idx) {
    switch (idx) {
        case 0: return Pins::Servos::MG996R_1;
        case 1: return Pins::Servos::MG996R_2;
        case 2: return Pins::Servos::MG996R_3;
        case 3: return Pins::Servos::MG996R_4;
        default: return -1;
    }
}

// ============================================================
//  PIN GUARDS
// ============================================================
static bool screwD1Wired() {
    return (Pins::LeadScrews::DEVICE1_SCREW1_STEP >= 0 &&
            Pins::LeadScrews::DEVICE1_SCREW1_DIR  >= 0 &&
            Pins::LeadScrews::DEVICE1_SCREW2_STEP >= 0 &&
            Pins::LeadScrews::DEVICE1_SCREW2_DIR  >= 0);
}

static bool anyServoAttached() {
    for (uint8_t i = 0; i < 4; i++)
        if ((SERVO_MASK & (1 << i)) && g_attached[i]) return true;
    return false;
}

// ============================================================
//  SERVO SWEEP  —  gradual move from `fromDeg` to `toDeg`
//  over `durationMs`.  Steps every ~20 ms (50 Hz cadence).
// ============================================================
static void servoSweep(float fromDeg, float toDeg, uint32_t durationMs) {
    constexpr uint32_t STEP_MS = 20;
    uint32_t steps = durationMs / STEP_MS;
    if (steps == 0) steps = 1;

    for (uint32_t s = 0; s <= steps; s++) {
        float t   = static_cast<float>(s) / static_cast<float>(steps);
        float deg = fromDeg + t * (toDeg - fromDeg);

        // Clamp to MG996R range
        if (deg < Constants::Servos::MG996R_MIN_ANGLE_DEG)
            deg = Constants::Servos::MG996R_MIN_ANGLE_DEG;
        if (deg > Constants::Servos::MG996R_MAX_ANGLE_DEG)
            deg = Constants::Servos::MG996R_MAX_ANGLE_DEG;

        for (uint8_t i = 0; i < 4; i++) {
            if (!(SERVO_MASK & (1 << i))) continue;
            if (!g_attached[i]) continue;
            g_servo[i].write(static_cast<int>(deg));
            g_servoAngle[i] = deg;
        }
        delay(STEP_MS);
    }
}

// ============================================================
//  LEAD-SCREW LIFT/LOWER  —  Both Device 1 screws move together.
//
//  Each screw may have a different microstepping setting, so they
//  can have different step counts for the same physical distance.
//  Both step pins fire each tick; each motor's step is gated by a
//  Bresenham accumulator so physical travel stays identical.
// ============================================================
static void liftD1(bool up) {
    if (!screwD1Wired()) {
        Serial.println(F("# SKIP: Device 1 STEP/DIR pins not set in PinMap.h."));
        return;
    }

    // Direction
    bool dirLevelS1 = up ? DIR_UP_D1S1 : !DIR_UP_D1S1;
    bool dirLevelS2 = up ? DIR_UP_D1S2 : !DIR_UP_D1S2;
    digitalWrite(Pins::LeadScrews::DEVICE1_SCREW1_DIR, dirLevelS1 ? HIGH : LOW);
    digitalWrite(Pins::LeadScrews::DEVICE1_SCREW2_DIR, dirLevelS2 ? HIGH : LOW);
    delayMicroseconds(2);   // DIR setup time

    // Timing: based on the motor that fires the most pulses
    uint32_t maxSteps = max(STEPS_D1S1, STEPS_D1S2);
    if (maxSteps == 0) return;

    uint32_t delayUs = static_cast<uint32_t>(
        1000000.0f / (LIFT_SPEED_MM_S * static_cast<float>(maxSteps) / LIFT_MM) + 0.5f);
    if (delayUs < PULSE_US) delayUs = PULSE_US;

    // Bresenham accumulators
    uint32_t accS1 = 0, accS2 = 0;

    for (uint32_t tick = 0; tick < maxSteps; tick++) {
        accS1 += STEPS_D1S1;
        accS2 += STEPS_D1S2;

        bool fireS1 = (accS1 >= maxSteps);
        bool fireS2 = (accS2 >= maxSteps);

        if (fireS1) { accS1 -= maxSteps; digitalWrite(Pins::LeadScrews::DEVICE1_SCREW1_STEP, HIGH); }
        if (fireS2) { accS2 -= maxSteps; digitalWrite(Pins::LeadScrews::DEVICE1_SCREW2_STEP, HIGH); }

        delayMicroseconds(PULSE_US);

        if (fireS1) digitalWrite(Pins::LeadScrews::DEVICE1_SCREW1_STEP, LOW);
        if (fireS2) digitalWrite(Pins::LeadScrews::DEVICE1_SCREW2_STEP, LOW);

        if (delayUs > PULSE_US) delayMicroseconds(delayUs - PULSE_US);
    }
}

// ============================================================
//  FULL SEQUENCE — lift → settle → push
// ============================================================
static void runSequence() {
    if (!screwD1Wired() && !anyServoAttached()) {
        Serial.println(F("# ERROR: No screws or servos wired. Check PinMap.h."));
        return;
    }

    Serial.println(F("# -------- Run Sequence --------"));

    // 1. LIFT
    if (screwD1Wired()) {
        Serial.print(F("# [1/3] Lifting platform ")); Serial.print(LIFT_MM, 1);
        Serial.print(F(" mm at ")); Serial.print(LIFT_SPEED_MM_S, 1); Serial.println(F(" mm/s..."));
        liftD1(true);
        Serial.println(F("# Lift complete."));
    } else {
        Serial.println(F("# [1/3] SKIP lift — Device 1 pins not wired."));
    }

    // 2. SETTLE
    Serial.print(F("# [2/3] Settling ")); Serial.print(SETTLE_MS); Serial.println(F(" ms..."));
    delay(SETTLE_MS);

    // 3. PUSH
    if (anyServoAttached()) {
        Serial.print(F("# [3/3] Pushing: servo(s) ")); Serial.print(REST_DEG, 1);
        Serial.print(F(" → ")); Serial.print(PUSH_DEG, 1);
        Serial.print(F(" deg over ")); Serial.print(PUSH_TIME_MS); Serial.println(F(" ms..."));
        servoSweep(REST_DEG, PUSH_DEG, PUSH_TIME_MS);
        Serial.println(F("# Push complete."));
    } else {
        Serial.println(F("# [3/3] SKIP push — no servo(s) attached (check SERVO_MASK + PinMap.h)."));
    }

    Serial.println(F("# Sequence done. Press H to reset."));
}

// ============================================================
//  RESET SEQUENCE — pull back → lower
// ============================================================
static void runReset() {
    Serial.println(F("# -------- Reset Sequence --------"));

    // 1. PULL BACK
    if (anyServoAttached()) {
        Serial.print(F("# [1/2] Pulling back: servo(s) ")); Serial.print(PUSH_DEG, 1);
        Serial.print(F(" → ")); Serial.print(REST_DEG, 1);
        Serial.print(F(" deg over ")); Serial.print(PULL_TIME_MS); Serial.println(F(" ms..."));
        servoSweep(PUSH_DEG, REST_DEG, PULL_TIME_MS);
        Serial.println(F("# Pull-back complete."));
    } else {
        Serial.println(F("# [1/2] SKIP pull-back — no servo(s) attached."));
    }

    delay(SETTLE_MS);

    // 2. LOWER
    if (screwD1Wired()) {
        Serial.print(F("# [2/2] Lowering platform ")); Serial.print(LIFT_MM, 1);
        Serial.println(F(" mm..."));
        liftD1(false);
        Serial.println(F("# Lower complete."));
    } else {
        Serial.println(F("# [2/2] SKIP lower — Device 1 pins not wired."));
    }

    Serial.println(F("# Reset done. Ready for next cycle."));
}

// ============================================================
//  HELP / STATUS
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# Device 1  —  Lift + Push Sequence Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  R  — Full sequence (lift → settle → push)"));
    Serial.println(F("#  H  — Reset / home (pull back → lower)"));
    Serial.println(F("#  L  — Lift only (screws up)"));
    Serial.println(F("#  D  — Lower only (screws down)"));
    Serial.println(F("#  P  — Push only (servo → PUSH_DEG)"));
    Serial.println(F("#  B  — Pull back (servo → REST_DEG)"));
    Serial.println(F("#  ?  — This help"));
    Serial.println(F("# ---------------------------------------------"));

    // Lead-screw summary
    Serial.print(F("# Lift dist:   ")); Serial.print(LIFT_MM, 1);    Serial.println(F(" mm"));
    Serial.print(F("# Lift speed:  ")); Serial.print(LIFT_SPEED_MM_S, 1); Serial.println(F(" mm/s"));
    Serial.print(F("# Settle:      ")); Serial.print(SETTLE_MS);     Serial.println(F(" ms"));
    Serial.print(F("# D1-S1 pin STEP/DIR: "));
    Serial.print(Pins::LeadScrews::DEVICE1_SCREW1_STEP); Serial.print(F(" / "));
    Serial.print(Pins::LeadScrews::DEVICE1_SCREW1_DIR);
    Serial.print(F("  MS=1/")); Serial.print(MS_D1S1);
    Serial.print(F("  ")); Serial.print(SPM_D1S1, 1); Serial.println(F(" steps/mm"));
    Serial.print(F("# D1-S2 pin STEP/DIR: "));
    Serial.print(Pins::LeadScrews::DEVICE1_SCREW2_STEP); Serial.print(F(" / "));
    Serial.print(Pins::LeadScrews::DEVICE1_SCREW2_DIR);
    Serial.print(F("  MS=1/")); Serial.print(MS_D1S2);
    Serial.print(F("  ")); Serial.print(SPM_D1S2, 1); Serial.println(F(" steps/mm"));

    if (!screwD1Wired())
        Serial.println(F("# WARNING: Device 1 STEP/DIR pins not set in PinMap.h."));

    // Servo summary
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Servo mask:  0b"));
    for (int b = 3; b >= 0; b--) Serial.print((SERVO_MASK >> b) & 1);
    Serial.println();
    Serial.print(F("# Rest angle:  ")); Serial.print(REST_DEG, 1);   Serial.println(F(" deg"));
    Serial.print(F("# Push angle:  ")); Serial.print(PUSH_DEG, 1);   Serial.println(F(" deg"));
    Serial.print(F("# Push time:   ")); Serial.print(PUSH_TIME_MS);  Serial.println(F(" ms"));

    for (uint8_t i = 0; i < 4; i++) {
        if (!(SERVO_MASK & (1 << i))) continue;
        Serial.print(F("# MG996R_")); Serial.print(i + 1);
        Serial.print(F("  pin=")); Serial.print(servoPin(i));
        Serial.print(F("  attached=")); Serial.println(g_attached[i] ? F("YES") : F("NO (pin=-1)"));
    }

    if (!anyServoAttached())
        Serial.println(F("# WARNING: No servo(s) in SERVO_MASK are wired. Check PinMap.h."));

    Serial.println(F("# ============================================="));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testDevice1Push_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Configure lead-screw pins
    if (Pins::LeadScrews::DEVICE1_SCREW1_STEP >= 0) {
        pinMode(Pins::LeadScrews::DEVICE1_SCREW1_STEP, OUTPUT);
        digitalWrite(Pins::LeadScrews::DEVICE1_SCREW1_STEP, LOW);
    }
    if (Pins::LeadScrews::DEVICE1_SCREW1_DIR >= 0)
        pinMode(Pins::LeadScrews::DEVICE1_SCREW1_DIR, OUTPUT);

    if (Pins::LeadScrews::DEVICE1_SCREW2_STEP >= 0) {
        pinMode(Pins::LeadScrews::DEVICE1_SCREW2_STEP, OUTPUT);
        digitalWrite(Pins::LeadScrews::DEVICE1_SCREW2_STEP, LOW);
    }
    if (Pins::LeadScrews::DEVICE1_SCREW2_DIR >= 0)
        pinMode(Pins::LeadScrews::DEVICE1_SCREW2_DIR, OUTPUT);

    // Attach servo(s) and drive to rest position
    for (uint8_t i = 0; i < 4; i++) {
        g_servoAngle[i] = REST_DEG;
        if (!(SERVO_MASK & (1 << i))) continue;
        int pin = servoPin(i);
        if (pin < 0) continue;
        g_servo[i].attach(pin, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        g_servo[i].write(static_cast<int>(REST_DEG));
        g_attached[i] = true;
    }

    printHelp();
}

void testDevice1Push_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {

        case 'r': case 'R':
            runSequence();
            break;

        case 'h': case 'H':
            runReset();
            break;

        case 'l': case 'L':
            if (!screwD1Wired()) {
                Serial.println(F("# Device 1 STEP/DIR pins not set in PinMap.h."));
                break;
            }
            Serial.print(F("# Lifting ")); Serial.print(LIFT_MM, 1); Serial.println(F(" mm..."));
            liftD1(true);
            Serial.println(F("# Done."));
            break;

        case 'd': case 'D':
            if (!screwD1Wired()) {
                Serial.println(F("# Device 1 STEP/DIR pins not set in PinMap.h."));
                break;
            }
            Serial.print(F("# Lowering ")); Serial.print(LIFT_MM, 1); Serial.println(F(" mm..."));
            liftD1(false);
            Serial.println(F("# Done."));
            break;

        case 'p': case 'P':
            if (!anyServoAttached()) {
                Serial.println(F("# No servo(s) attached — check SERVO_MASK and PinMap.h."));
                break;
            }
            Serial.print(F("# Push: → ")); Serial.print(PUSH_DEG, 1); Serial.println(F(" deg"));
            servoSweep(REST_DEG, PUSH_DEG, PUSH_TIME_MS);
            Serial.println(F("# Done."));
            break;

        case 'b': case 'B':
            if (!anyServoAttached()) {
                Serial.println(F("# No servo(s) attached — check SERVO_MASK and PinMap.h."));
                break;
            }
            Serial.print(F("# Pull back: → ")); Serial.print(REST_DEG, 1); Serial.println(F(" deg"));
            servoSweep(PUSH_DEG, REST_DEG, PULL_TIME_MS);
            Serial.println(F("# Done."));
            break;

        case '?':
            printHelp();
            break;

        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

#endif // TEST_MODE
