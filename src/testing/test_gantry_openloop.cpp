// ============================================================
//  testing/test_gantry_openloop.cpp
//
//  PURPOSE
//  -------
//  Move the CoreXY gantry via direct STEP/DIR GPIO — no
//  ClosedLoopStepper, no MotorController, no Kalman/LQR,
//  no MachineState, no TeensyThreads.
//
//  Use this to verify wiring, direction sense, and step
//  generation in complete isolation before enabling the
//  closed-loop control stack.
//
//  SETUP
//  -----
//  Only PinMap.h is required.  Confirm the following are set:
//    Pins::Gantry::MOTOR_A_STEP / MOTOR_A_DIR
//    Pins::Gantry::MOTOR_B_STEP / MOTOR_B_DIR
//
//  CONTROLS (single keypress, no newline needed)
//  ----------------------------------------------
//    W / S  — Jog +Y / -Y
//    A / D  — Jog -X / +X
//    Q / E  — Step Motor A only (forward / backward)
//    Z / C  — Step Motor B only (forward / backward)
//    + / -  — Double / halve jog step count
//    F / G  — Increase / decrease step speed (inter-step delay)
//    P      — Print current step counts (A and B)
//    R      — Reset step counters to zero
//    T      — Auto-test: step A fwd, A rev, B fwd, B rev
//    ?      — Print this help
//
//  COREXY DIRECTION CONVENTION
//  ----------------------------
//  Motor A drives belt anchored at left  (contributes X + Y)
//  Motor B drives belt anchored at right (contributes X - Y)
//
//    Pure +Y: A forward,  B backward
//    Pure -Y: A backward, B forward
//    Pure +X: A forward,  B forward
//    Pure -X: A backward, B backward
//
//  If W jogs the wrong direction, flip DIR_A_FORWARD or
//  DIR_B_FORWARD below.  If A/D are wrong, flip only one of them.
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testGantryOpenLoop_setup();
//    extern void testGantryOpenLoop_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"

// ============================================================
//  USER CONFIGURATION
// ============================================================

// Physical DIR pin level that moves each motor "forward".
// Flip to LOW if a motor runs the wrong direction.
static constexpr bool DIR_A_FORWARD = LOW;   // flip to LOW if Motor A reversed
static constexpr bool DIR_B_FORWARD = LOW;   // flip to LOW if Motor B reversed

// Starting inter-step delay (µs).  Lower = faster.
// TB6600 can typically handle down to ~100 µs (5000 steps/s at full step).
// Start conservative and decrease with F key.
static constexpr uint32_t DELAY_US_DEFAULT = 1000;   // 1 ms = ~1000 steps/s
static constexpr uint32_t DELAY_US_MIN     = 100;    // 100 µs = ~10 000 steps/s
static constexpr uint32_t DELAY_US_MAX     = 10000;  // 10 ms (very slow)

// Step pulse high time (µs) — TB6600 datasheet minimum is 2.2 µs
static constexpr uint32_t PULSE_US = 5;

// Default and limits for jog step count
static constexpr uint32_t JOG_STEPS_DEFAULT = 200;   // 1 mm at STEPS_PER_MM=200
static constexpr uint32_t JOG_STEPS_MIN     = 10;
static constexpr uint32_t JOG_STEPS_MAX     = 10000;

// ============================================================
//  RUNTIME STATE
// ============================================================
static uint32_t g_delayUs   = DELAY_US_DEFAULT;
static uint32_t g_jogSteps  = JOG_STEPS_DEFAULT;

// Cumulative step counters (signed)
static int32_t g_stepsA = 0;
static int32_t g_stepsB = 0;

// ============================================================
//  STEP PRIMITIVE
//  Issues `n` pulses on `stepPin` with direction set on `dirPin`.
//  forward=true → DIR_x_FORWARD level; false → opposite.
//  Returns the signed step count (+n or -n) for accumulation.
// ============================================================
static int32_t stepMotor(int stepPin, int dirPin,
                         bool forwardConst, bool forward,
                         uint32_t n)
{
    // Set direction
    bool pinLevel = forward ? forwardConst : !forwardConst;
    digitalWrite(dirPin, pinLevel ? HIGH : LOW);
    delayMicroseconds(5);   // DIR setup time before first pulse

    for (uint32_t i = 0; i < n; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(g_delayUs);   // inter-step hold
    }

    return forward ? static_cast<int32_t>(n) : -static_cast<int32_t>(n);
}

// ============================================================
//  SIMULTANEOUS TWO-MOTOR STEP
//  Both STEP pins are toggled on the same pulse so the motors
//  move together and the carriage travels in a straight line.
//  forwardA / forwardB: true = forward sense for that motor.
// ============================================================
static void stepBoth(bool forwardA, bool forwardB, uint32_t n)
{
    bool levA = forwardA ? DIR_A_FORWARD : !DIR_A_FORWARD;
    bool levB = forwardB ? DIR_B_FORWARD : !DIR_B_FORWARD;
    digitalWrite(Pins::Gantry::MOTOR_A_DIR, levA ? HIGH : LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_DIR, levB ? HIGH : LOW);
    delayMicroseconds(5);   // DIR setup time

    for (uint32_t i = 0; i < n; i++) {
        digitalWrite(Pins::Gantry::MOTOR_A_STEP, HIGH);
        digitalWrite(Pins::Gantry::MOTOR_B_STEP, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
        digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);
        delayMicroseconds(g_delayUs);
    }

    g_stepsA += forwardA ? static_cast<int32_t>(n) : -static_cast<int32_t>(n);
    g_stepsB += forwardB ? static_cast<int32_t>(n) : -static_cast<int32_t>(n);
}

// ============================================================
//  JOG HELPERS
// ============================================================
static void jogPosY() {
    // +Y: A forward, B backward — both step simultaneously
    Serial.print(F("# +Y  ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
    stepBoth(true, false, g_jogSteps);
}

static void jogNegY() {
    // -Y: A backward, B forward — both step simultaneously
    Serial.print(F("# -Y  ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
    stepBoth(false, true, g_jogSteps);
}

static void jogPosX() {
    // +X: A forward, B forward — both step simultaneously
    Serial.print(F("# +X  ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
    stepBoth(true, true, g_jogSteps);
}

static void jogNegX() {
    // -X: A backward, B backward
    Serial.print(F("# -X  ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
    stepBoth(false, false, g_jogSteps);
}

// ============================================================
//  PRINT HELPERS
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# CoreXY Open-Loop Gantry Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  W / S  — Jog +Y / -Y"));
    Serial.println(F("#  A / D  — Jog -X / +X"));
    Serial.println(F("#  Q / E  — Motor A only: forward / backward"));
    Serial.println(F("#  Z / C  — Motor B only: forward / backward"));
    Serial.println(F("#  + / -  — Double / halve jog step count"));
    Serial.println(F("#  F / G  — Faster / slower step speed"));
    Serial.println(F("#  P      — Print step counters"));
    Serial.println(F("#  R      — Reset step counters"));
    Serial.println(F("#  T      — Auto-test sequence"));
    Serial.println(F("#  ?      — This help"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Jog steps:  ")); Serial.println(g_jogSteps);
    Serial.print(F("# Step delay: ")); Serial.print(g_delayUs); Serial.println(F(" µs"));
    Serial.print(F("# A STEP/DIR: ")); Serial.print(Pins::Gantry::MOTOR_A_STEP);
    Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_A_DIR);
    Serial.print(F("# B STEP/DIR: ")); Serial.print(Pins::Gantry::MOTOR_B_STEP);
    Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_B_DIR);
    Serial.println(F("# ============================================="));
}

static void printCounters() {
    Serial.print(F("# Steps A: ")); Serial.print(g_stepsA);
    Serial.print(F("  Steps B: ")); Serial.println(g_stepsB);
}

// ============================================================
//  AUTO-TEST
//  Steps each motor forward then back independently so you can
//  visually confirm direction sense before jogging diagonally.
// ============================================================
static void autoTest() {
    const uint32_t TEST_STEPS = 400;

    Serial.println(F("# Auto-test: Motor A forward..."));
    g_stepsA += stepMotor(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                          DIR_A_FORWARD, true, TEST_STEPS);
    delay(300);

    Serial.println(F("# Auto-test: Motor A backward..."));
    g_stepsA += stepMotor(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                          DIR_A_FORWARD, false, TEST_STEPS);
    delay(300);

    Serial.println(F("# Auto-test: Motor B forward..."));
    g_stepsB += stepMotor(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                          DIR_B_FORWARD, true, TEST_STEPS);
    delay(300);

    Serial.println(F("# Auto-test: Motor B backward..."));
    g_stepsB += stepMotor(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                          DIR_B_FORWARD, false, TEST_STEPS);
    delay(300);

    Serial.println(F("# Auto-test done."));
    Serial.println(F("#   Observe:"));
    Serial.println(F("#     A fwd + B bwd = +Y  (W key)"));
    Serial.println(F("#     A bwd + B fwd = -Y  (S key)"));
    Serial.println(F("#     A fwd + B fwd = +X  (D key)"));
    Serial.println(F("#     A bwd + B bwd = -X  (A key)"));
    Serial.println(F("#   If any direction is wrong, flip DIR_A_FORWARD or"));
    Serial.println(F("#   DIR_B_FORWARD at the top of test_gantry_openloop.cpp"));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testGantryOpenLoop_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(500);

    // Check pins are assigned
    if (Pins::Gantry::MOTOR_A_STEP < 0 || Pins::Gantry::MOTOR_A_DIR < 0 ||
        Pins::Gantry::MOTOR_B_STEP < 0 || Pins::Gantry::MOTOR_B_DIR < 0) {
        Serial.println(F("# ERROR: Gantry STEP/DIR pins not set in PinMap.h. Halting."));
        while (true) {}
    }

    pinMode(Pins::Gantry::MOTOR_A_STEP, OUTPUT);
    pinMode(Pins::Gantry::MOTOR_A_DIR,  OUTPUT);
    pinMode(Pins::Gantry::MOTOR_B_STEP, OUTPUT);
    pinMode(Pins::Gantry::MOTOR_B_DIR,  OUTPUT);

    digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);

    printHelp();
}

void testGantryOpenLoop_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {
        case 'w': case 'W': jogPosY(); break;
        case 's': case 'S': jogNegY(); break;
        case 'a': case 'A': jogNegX(); break;
        case 'd': case 'D': jogPosX(); break;

        case 'q': case 'Q':
            Serial.print(F("# Motor A fwd ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
            g_stepsA += stepMotor(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                                  DIR_A_FORWARD, true, g_jogSteps);
            break;
        case 'e': case 'E':
            Serial.print(F("# Motor A rev ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
            g_stepsA += stepMotor(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                                  DIR_A_FORWARD, false, g_jogSteps);
            break;
        case 'z': case 'Z':
            Serial.print(F("# Motor B fwd ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
            g_stepsB += stepMotor(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                                  DIR_B_FORWARD, true, g_jogSteps);
            break;
        case 'c': case 'C':
            Serial.print(F("# Motor B rev ")); Serial.print(g_jogSteps); Serial.println(F(" steps"));
            g_stepsB += stepMotor(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                                  DIR_B_FORWARD, false, g_jogSteps);
            break;

        case '+': case '=':
            g_jogSteps = min(g_jogSteps * 2, JOG_STEPS_MAX);
            Serial.print(F("# Jog steps: ")); Serial.println(g_jogSteps);
            break;
        case '-': case '_':
            g_jogSteps = max(g_jogSteps / 2, JOG_STEPS_MIN);
            Serial.print(F("# Jog steps: ")); Serial.println(g_jogSteps);
            break;

        case 'f': case 'F':
            g_delayUs = max(g_delayUs / 2, DELAY_US_MIN);
            Serial.print(F("# Step delay: ")); Serial.print(g_delayUs); Serial.println(F(" µs (faster)"));
            break;
        case 'g': case 'G':
            g_delayUs = min(g_delayUs * 2, DELAY_US_MAX);
            Serial.print(F("# Step delay: ")); Serial.print(g_delayUs); Serial.println(F(" µs (slower)"));
            break;

        case 'p': case 'P':
            printCounters();
            break;
        case 'r': case 'R':
            g_stepsA = 0; g_stepsB = 0;
            Serial.println(F("# Step counters reset."));
            break;
        case 't': case 'T':
            autoTest();
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
