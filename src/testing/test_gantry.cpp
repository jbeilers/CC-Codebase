// ============================================================
//  testing/test_gantry.cpp
//
//  PURPOSE
//  -------
//  Unified gantry bring-up test.  Supports two runtime modes
//  switchable with the 'M' key:
//
//    OPEN-LOOP  — Direct STEP/DIR GPIO.  No Kalman, no LQR,
//                 no e-stops.  Use this first to confirm wiring
//                 and physical direction before enabling feedback.
//
//    CLOSED-LOOP — Full MotorController pipeline (Trajectory +
//                  FeedForward + Kalman + LQR).  Use this once
//                  open-loop motion looks correct.
//
//  Encoder position is always displayed regardless of mode.
//
//  CONTROLS
//  ---------
//    W / S      Jog +Y / -Y
//    A / D      Jog -X / +X
//    M          Toggle open-loop ↔ closed-loop
//    P          Print full position status
//    R          Reset encoder counters and Kalman to zero
//    + / -      More / fewer steps (OL) or larger / smaller mm (CL)
//    F / G      Faster / slower step speed  (OL only)
//    Q / E      Motor A forward / backward  (always raw GPIO)
//    Z / C      Motor B forward / backward  (always raw GPIO)
//    T          Auto-test each motor individually (always raw GPIO)
//    0          Move to origin (CL only)
//    ?          Print this help
//
//  SETUP
//  -----
//  Set the four constants in the USER CONFIGURATION section below.
//  PinMap.h must have Gantry STEP/DIR/ENC pins assigned.
//  Encoders are optional in open-loop mode but give richer output.
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testGantry_setup();
//    extern void testGantry_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"
#include "math/Constants.h"
#include "devices/ClosedLoopStepper.h"
#include "control/MotorController.h"
#include "state/MachineState.h"

// ============================================================
//  USER CONFIGURATION
// ============================================================

// Encoder counts per revolution for your motor.
// 17HS24-2004D-E1K: confirm from datasheet (commonly 1000 or 4000).
static constexpr int32_t ENCODER_CPR = 4000;

// Set true if a motor runs backwards in closed-loop mode.
// These flip the DIR pin level sent to the TB6600.
// Derive from open-loop test: if DIR_x_FORWARD = LOW, set INVERT = true.
static constexpr bool INVERT_A = true;
static constexpr bool INVERT_B = true;

// Open-loop jog defaults
static constexpr uint32_t OL_STEPS_DEFAULT = 200;
static constexpr uint32_t OL_STEPS_MIN     = 10;
static constexpr uint32_t OL_STEPS_MAX     = 10000;
static constexpr uint32_t OL_DELAY_DEFAULT = 1000;  // µs between steps
static constexpr uint32_t OL_DELAY_MIN     = 100;
static constexpr uint32_t OL_DELAY_MAX     = 10000;
static constexpr uint32_t OL_PULSE_US      = 5;     // STEP high time

// Closed-loop jog defaults (mm)
static constexpr float CL_JOG_MM_DEFAULT = 5.0f;
static constexpr float CL_JOG_MM_MIN     = 0.5f;
static constexpr float CL_JOG_MM_MAX     = 50.0f;

// Control loop rate limit (must match Kalman/LQR design timestep)
static constexpr uint32_t CONTROL_PERIOD_US = 1000;  // 1 kHz

// ============================================================
//  RUNTIME STATE
// ============================================================
enum class GantryMode { OPEN_LOOP, CLOSED_LOOP };
static GantryMode g_mode = GantryMode::OPEN_LOOP;

// ClosedLoopStepper constructed inside setup() to avoid
// attachInterrupt() running before Arduino hardware init.
static ClosedLoopStepper* g_motorA = nullptr;
static ClosedLoopStepper* g_motorB = nullptr;
static MotorController*   g_mc     = nullptr;
static bool               g_encodersAvailable = false;

// Open-loop state
static uint32_t g_olSteps = OL_STEPS_DEFAULT;
static uint32_t g_olDelay = OL_DELAY_DEFAULT;

// Closed-loop state
static float g_clJogMM = CL_JOG_MM_DEFAULT;

// Rate-limit timestamp for closed-loop update()
static uint32_t g_lastControlUs = 0;

// E-stop print guard
static bool g_ePrinted = false;

// Live encoder stream
static bool     g_encStream   = false;
static uint32_t g_encStreamMs = 0;
static constexpr uint32_t ENC_STREAM_INTERVAL_MS = 100;

// Derived DIR pin levels for open-loop (consistent with INVERT flags)
static const int OL_DIR_A_FWD = INVERT_A ? LOW : HIGH;
static const int OL_DIR_B_FWD = INVERT_B ? LOW : HIGH;

// ============================================================
//  HELPERS — encoder position
// ============================================================
static float encAMM() {
    return g_encodersAvailable ? g_motorA->getEncoderPositionMM() : 0.0f;
}
static float encBMM() {
    return g_encodersAvailable ? g_motorB->getEncoderPositionMM() : 0.0f;
}
// CoreXY forward kinematics from encoder mm values
static float encXMM() { return (encAMM() + encBMM()) * 0.5f; }
static float encYMM() { return (encAMM() - encBMM()) * 0.5f; }

// ============================================================
//  OPEN-LOOP STEP PRIMITIVES  (raw GPIO, no ClosedLoopStepper)
// ============================================================
static void olStep(int stepPin, int dirPin, int fwdLevel,
                   bool forward, uint32_t n)
{
    digitalWrite(dirPin, forward ? fwdLevel : !fwdLevel);
    delayMicroseconds(5);
    for (uint32_t i = 0; i < n; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(OL_PULSE_US);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(g_olDelay);
    }
}

// Step both motors simultaneously (straight-line motion)
static void olStepBoth(bool fwdA, bool fwdB, uint32_t n)
{
    digitalWrite(Pins::Gantry::MOTOR_A_DIR, fwdA ? OL_DIR_A_FWD : !OL_DIR_A_FWD);
    digitalWrite(Pins::Gantry::MOTOR_B_DIR, fwdB ? OL_DIR_B_FWD : !OL_DIR_B_FWD);
    delayMicroseconds(5);
    for (uint32_t i = 0; i < n; i++) {
        digitalWrite(Pins::Gantry::MOTOR_A_STEP, HIGH);
        digitalWrite(Pins::Gantry::MOTOR_B_STEP, HIGH);
        delayMicroseconds(OL_PULSE_US);
        digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
        digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);
        delayMicroseconds(g_olDelay);
    }
}

// ============================================================
//  JOG — open-loop
// ============================================================
static void olJog(bool fwdA, bool fwdB, const __FlashStringHelper* label)
{
    Serial.print(F("# [OL] ")); Serial.print(label);
    Serial.print(F("  ")); Serial.print(g_olSteps); Serial.println(F(" steps"));
    olStepBoth(fwdA, fwdB, g_olSteps);
}

// ============================================================
//  JOG — closed-loop
// ============================================================
static void clJog(float dx, float dy, const __FlashStringHelper* label)
{
    if (!g_mc) return;
    if (g_mc->isEStopped()) {
        Serial.println(F("# [CL] E-stopped — press R to reset, or M to switch to open-loop."));
        return;
    }
    float nx = g_mc->getX() + dx;
    float ny = g_mc->getY() + dy;
    Serial.print(F("# [CL] ")); Serial.print(label);
    Serial.print(F(" → X=")); Serial.print(nx, 2);
    Serial.print(F(" Y=")); Serial.println(ny, 2);
    g_mc->moveTo(nx, ny);
}

// ============================================================
//  MODE SWITCH
// ============================================================
static void switchMode(GantryMode newMode)
{
    if (newMode == g_mode) return;

    if (newMode == GantryMode::CLOSED_LOOP) {
        if (!g_mc) {
            Serial.println(F("# Cannot switch: MotorController not initialised (encoder pins missing?)."));
            return;
        }
        // Sync Kalman to current encoder-derived position before handing over
        float x = encXMM();
        float y = encYMM();
        g_mc->resetPosition(x, y);
        g_ePrinted = false;
        Serial.println(F("# Mode → CLOSED-LOOP"));
        Serial.print(F("#   Kalman seeded at X=")); Serial.print(x, 2);
        Serial.print(F(" Y=")); Serial.println(y, 2);
    } else {
        if (g_mc) g_mc->stop();
        Serial.println(F("# Mode → OPEN-LOOP"));
    }
    g_mode = newMode;
}

// ============================================================
//  RESET
// ============================================================
static void doReset()
{
    if (g_encodersAvailable) {
        g_motorA->zeroPosition();
        g_motorB->zeroPosition();
    }
    if (g_mc) {
        g_mc->resetPosition(0.0f, 0.0f);
        g_ePrinted = false;
    }
    Serial.println(F("# Encoders and Kalman zeroed."));
}

// ============================================================
//  POSITION PRINT
// ============================================================
static void printPosition()
{
    Serial.println(F("# ---- Position --------------------------------"));

    if (g_encodersAvailable) {
        Serial.print(F("#  Enc A raw:  ")); Serial.println(g_motorA->getEncoderCount());
        Serial.print(F("#  Enc B raw:  ")); Serial.println(g_motorB->getEncoderCount());
        Serial.print(F("#  Enc A mm:   ")); Serial.println(encAMM(), 3);
        Serial.print(F("#  Enc B mm:   ")); Serial.println(encBMM(), 3);
        Serial.print(F("#  Enc X mm:   ")); Serial.println(encXMM(), 3);
        Serial.print(F("#  Enc Y mm:   ")); Serial.println(encYMM(), 3);
    } else {
        Serial.println(F("#  (encoders not available)"));
    }

    if (g_mode == GantryMode::CLOSED_LOOP && g_mc) {
        Serial.print(F("#  Kalman X:   ")); Serial.println(g_mc->getX(), 3);
        Serial.print(F("#  Kalman Y:   ")); Serial.println(g_mc->getY(), 3);
        Serial.print(F("#  Moving:     ")); Serial.println(g_mc->isMoving() ? F("yes") : F("no"));
        Serial.print(F("#  E-stopped:  ")); Serial.println(g_mc->isEStopped() ? F("YES") : F("no"));
    }

    Serial.print(F("#  Mode:        "));
    Serial.println(g_mode == GantryMode::OPEN_LOOP ? F("OPEN-LOOP") : F("CLOSED-LOOP"));
    Serial.print(F("#  OL steps:    ")); Serial.println(g_olSteps);
    Serial.print(F("#  OL delay:    ")); Serial.print(g_olDelay); Serial.println(F(" µs"));
    Serial.print(F("#  CL jog:      ")); Serial.print(g_clJogMM, 1); Serial.println(F(" mm"));
    Serial.println(F("# -----------------------------------------------"));
}

// ============================================================
//  AUTO-TEST  (always raw GPIO — tests physical wiring)
// ============================================================
static void autoTest()
{
    const uint32_t N = 400;
    Serial.println(F("# Auto-test: Motor A forward (raw GPIO)..."));
    olStep(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
           OL_DIR_A_FWD, true, N);
    delay(300);
    Serial.println(F("# Auto-test: Motor A backward..."));
    olStep(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
           OL_DIR_A_FWD, false, N);
    delay(300);
    Serial.println(F("# Auto-test: Motor B forward..."));
    olStep(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
           OL_DIR_B_FWD, true, N);
    delay(300);
    Serial.println(F("# Auto-test: Motor B backward..."));
    olStep(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
           OL_DIR_B_FWD, false, N);
    delay(300);
    Serial.println(F("# Auto-test done.  A fwd + B bwd = +Y (W).  A fwd + B fwd = +X (D)."));
}

// ============================================================
//  HELP
// ============================================================
static void printHelp()
{
    Serial.println(F("# ============================================"));
    Serial.println(F("# Gantry Test  —  open-loop & closed-loop"));
    Serial.println(F("# --------------------------------------------"));
    Serial.println(F("#  W / S    Jog +Y / -Y"));
    Serial.println(F("#  A / D    Jog -X / +X"));
    Serial.println(F("#  M        Toggle open-loop ↔ closed-loop"));
    Serial.println(F("#  P        Print position status"));
    Serial.println(F("#  R        Reset encoders and Kalman to zero"));
    Serial.println(F("#  + / -    More/fewer steps (OL) or mm (CL)"));
    Serial.println(F("#  F / G    Faster / slower  (open-loop only)"));
    Serial.println(F("#  Q / E    Motor A fwd / bwd  (raw GPIO)"));
    Serial.println(F("#  Z / C    Motor B fwd / bwd  (raw GPIO)"));
    Serial.println(F("#  T        Auto-test each motor (raw GPIO)"));
    Serial.println(F("#  I        Toggle live encoder stream (100 ms)"));
    Serial.println(F("#  0        Move to origin  (closed-loop only)"));
    Serial.println(F("#  ?        This help"));
    Serial.println(F("# --------------------------------------------"));
    Serial.print(F("#  Mode:    "));
    Serial.println(g_mode == GantryMode::OPEN_LOOP ? F("OPEN-LOOP") : F("CLOSED-LOOP"));
    Serial.print(F("#  OL steps: ")); Serial.println(g_olSteps);
    Serial.print(F("#  OL delay: ")); Serial.print(g_olDelay); Serial.println(F(" µs"));
    Serial.print(F("#  CL jog:   ")); Serial.print(g_clJogMM, 1); Serial.println(F(" mm"));
    Serial.println(F("# ============================================"));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testGantry_setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(500);

    // Always configure STEP/DIR pins so open-loop works immediately
    if (Pins::Gantry::MOTOR_A_STEP < 0 || Pins::Gantry::MOTOR_A_DIR < 0 ||
        Pins::Gantry::MOTOR_B_STEP < 0 || Pins::Gantry::MOTOR_B_DIR < 0) {
        Serial.println(F("# ERROR: Gantry STEP/DIR pins not set in PinMap.h. Halting."));
        while (true) {}
    }
    pinMode(Pins::Gantry::MOTOR_A_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    pinMode(Pins::Gantry::MOTOR_A_DIR,  OUTPUT);
    pinMode(Pins::Gantry::MOTOR_B_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);
    pinMode(Pins::Gantry::MOTOR_B_DIR,  OUTPUT);

    // Attempt to construct closed-loop objects if encoder pins are assigned.
    // Construction MUST happen inside setup() — Encoder::attachInterrupt() must
    // run after Arduino hardware init or it crashes.
    bool encPinsOk = (Pins::Gantry::MOTOR_A_ENC_A >= 0 &&
                      Pins::Gantry::MOTOR_A_ENC_B >= 0 &&
                      Pins::Gantry::MOTOR_B_ENC_A >= 0 &&
                      Pins::Gantry::MOTOR_B_ENC_B >= 0);

    if (encPinsOk) {
        static ClosedLoopStepper motorA(
            Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
            Pins::Gantry::MOTOR_A_ENC_A, Pins::Gantry::MOTOR_A_ENC_B,
            Constants::Gantry::STEPS_PER_MM, 0, ENCODER_CPR, INVERT_A);
        static ClosedLoopStepper motorB(
            Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
            Pins::Gantry::MOTOR_B_ENC_A, Pins::Gantry::MOTOR_B_ENC_B,
            Constants::Gantry::STEPS_PER_MM, 1, ENCODER_CPR, INVERT_B);
        static MotorController mc;

        motorA.begin();
        motorB.begin();
        mc.begin(motorA, motorB);

        g_motorA = &motorA;
        g_motorB = &motorB;
        g_mc     = &mc;
        g_encodersAvailable = true;

        MachineState::instance().setStatus(SystemStatus::IDLE);
        Serial.println(F("# Encoders and MotorController ready."));
        Serial.println(F("# Start in OPEN-LOOP mode. Press M to switch to closed-loop."));
    } else {
        Serial.println(F("# WARNING: Encoder pins not set — running open-loop only."));
        Serial.println(F("#   Set Gantry::MOTOR_x_ENC_A/B in PinMap.h to enable closed-loop."));
    }

    printHelp();
}

void testGantry_loop()
{
    // Closed-loop control tick (rate-limited to 1 kHz)
    if (g_mode == GantryMode::CLOSED_LOOP && g_mc) {
        uint32_t now = micros();
        if (now - g_lastControlUs >= CONTROL_PERIOD_US) {
            g_lastControlUs = now;
            g_mc->update();
        }
        if (g_mc->isEStopped() && !g_ePrinted) {
            g_ePrinted = true;
            Serial.println(F("# E-STOP: Kalman position exceeded workspace boundary."));
            Serial.println(F("#   Likely cause: encoder phases (ENC_A/ENC_B) wired backwards"));
            Serial.println(F("#   on one motor — swap those two pins in PinMap.h."));
            Serial.println(F("#   Press R to reset, or M to switch back to open-loop."));
        }
        if (!g_mc->isEStopped()) g_ePrinted = false;
    }

    // Live encoder stream
    if (g_encStream && g_encodersAvailable) {
        uint32_t nowMs = millis();
        if (nowMs - g_encStreamMs >= ENC_STREAM_INTERVAL_MS) {
            g_encStreamMs = nowMs;
            Serial.print(F("ENC A="));
            Serial.print(g_motorA->getEncoderCount());
            Serial.print(F("  B="));
            Serial.print(g_motorB->getEncoderCount());
            Serial.print(F("  X="));
            Serial.print(encXMM(), 2);
            Serial.print(F("mm  Y="));
            Serial.print(encYMM(), 2);
            Serial.println(F("mm"));
        }
    }

    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());

    switch (c) {
        // ---- Jog (mode-aware) ----
        case 'w': case 'W':
            if (g_mode == GantryMode::OPEN_LOOP)
                olJog(true, false, F("+Y"));
            else
                clJog(0.0f, g_clJogMM, F("+Y"));
            break;
        case 's': case 'S':
            if (g_mode == GantryMode::OPEN_LOOP)
                olJog(false, true, F("-Y"));
            else
                clJog(0.0f, -g_clJogMM, F("-Y"));
            break;
        case 'd': case 'D':
            if (g_mode == GantryMode::OPEN_LOOP)
                olJog(true, true, F("+X"));
            else
                clJog(g_clJogMM, 0.0f, F("+X"));
            break;
        case 'a': case 'A':
            if (g_mode == GantryMode::OPEN_LOOP)
                olJog(false, false, F("-X"));
            else
                clJog(-g_clJogMM, 0.0f, F("-X"));
            break;

        // ---- Single-motor raw GPIO (always open-loop) ----
        case 'q': case 'Q':
            Serial.print(F("# Motor A fwd ")); Serial.print(g_olSteps); Serial.println(F(" steps (raw)"));
            olStep(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                   OL_DIR_A_FWD, true, g_olSteps);
            break;
        case 'e': case 'E':
            Serial.print(F("# Motor A bwd ")); Serial.print(g_olSteps); Serial.println(F(" steps (raw)"));
            olStep(Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
                   OL_DIR_A_FWD, false, g_olSteps);
            break;
        case 'z': case 'Z':
            Serial.print(F("# Motor B fwd ")); Serial.print(g_olSteps); Serial.println(F(" steps (raw)"));
            olStep(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                   OL_DIR_B_FWD, true, g_olSteps);
            break;
        case 'c': case 'C':
            Serial.print(F("# Motor B bwd ")); Serial.print(g_olSteps); Serial.println(F(" steps (raw)"));
            olStep(Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
                   OL_DIR_B_FWD, false, g_olSteps);
            break;

        // ---- Mode toggle ----
        case 'm': case 'M':
            switchMode(g_mode == GantryMode::OPEN_LOOP
                       ? GantryMode::CLOSED_LOOP
                       : GantryMode::OPEN_LOOP);
            break;

        // ---- Step count / jog distance ----
        case '+': case '=':
            if (g_mode == GantryMode::OPEN_LOOP) {
                g_olSteps = min(g_olSteps * 2, OL_STEPS_MAX);
                Serial.print(F("# OL steps: ")); Serial.println(g_olSteps);
            } else {
                g_clJogMM = min(g_clJogMM * 2.0f, CL_JOG_MM_MAX);
                Serial.print(F("# CL jog: ")); Serial.print(g_clJogMM, 1); Serial.println(F(" mm"));
            }
            break;
        case '-': case '_':
            if (g_mode == GantryMode::OPEN_LOOP) {
                g_olSteps = max(g_olSteps / 2, OL_STEPS_MIN);
                Serial.print(F("# OL steps: ")); Serial.println(g_olSteps);
            } else {
                g_clJogMM = max(g_clJogMM * 0.5f, CL_JOG_MM_MIN);
                Serial.print(F("# CL jog: ")); Serial.print(g_clJogMM, 1); Serial.println(F(" mm"));
            }
            break;

        // ---- Speed (open-loop only) ----
        case 'f': case 'F':
            g_olDelay = max(g_olDelay / 2, OL_DELAY_MIN);
            Serial.print(F("# OL delay: ")); Serial.print(g_olDelay); Serial.println(F(" µs (faster)"));
            break;
        case 'g': case 'G':
            g_olDelay = min(g_olDelay * 2, OL_DELAY_MAX);
            Serial.print(F("# OL delay: ")); Serial.print(g_olDelay); Serial.println(F(" µs (slower)"));
            break;

        // ---- Reset ----
        case 'r': case 'R':
            doReset();
            break;

        // ---- Position print ----
        case 'p': case 'P':
            printPosition();
            break;

        // ---- Origin (closed-loop only) ----
        case '0':
            if (g_mode == GantryMode::CLOSED_LOOP && g_mc) {
                if (g_mc->isEStopped()) {
                    Serial.println(F("# E-stopped — press R first."));
                } else {
                    Serial.println(F("# Moving to origin (0, 0)"));
                    g_mc->moveTo(0.0f, 0.0f);
                }
            } else {
                Serial.println(F("# '0' only works in closed-loop mode (press M to switch)."));
            }
            break;

        // ---- Auto-test ----
        case 't': case 'T':
            autoTest();
            break;

        // ---- Live encoder stream ----
        case 'i': case 'I':
            if (!g_encodersAvailable) {
                Serial.println(F("# Encoders not available (check ENC pins in PinMap.h)."));
            } else {
                g_encStream = !g_encStream;
                g_encStreamMs = millis();
                Serial.println(g_encStream ? F("# Encoder stream ON  (press I to stop)")
                                           : F("# Encoder stream OFF"));
            }
            break;

        // ---- Help ----
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
