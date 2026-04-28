// ============================================================
//  testing/test_corexy_jog.cpp
//
//  PURPOSE
//  -------
//  Manually jog the CoreXY gantry via single-character commands
//  sent from a serial terminal (115200 baud, newline not required).
//
//  This test deliberately uses NO TaskQueue, NO ThreadManager,
//  NO operations — only the bare ClosedLoopStepper drivers and
//  MotorController so you can verify wiring, direction, and step
//  generation in isolation before enabling the full control stack.
//
//  SETUP — fill in PinMap.h first
//  --------------------------------
//  In include/pins/PinMap.h, under namespace Gantry, confirm:
//
//    MOTOR_A_STEP / MOTOR_A_DIR    — already set (4, 5)
//    MOTOR_B_STEP / MOTOR_B_DIR    — already set (6, 7)
//    MOTOR_A_ENC_A / MOTOR_A_ENC_B — set encoder pins for Motor A
//    MOTOR_B_ENC_A / MOTOR_B_ENC_B — set encoder pins for Motor B
//
//  In include/math/Constants.h, under namespace Gantry, confirm:
//
//    BELT_PITCH_MM   — GT2 belt = 2.0
//    PULLEY_TEETH    — your pulley (default 20)
//    MICROSTEPPING   — match your TB6600 DIP switches (default 4)
//    MOTOR_STEPS_PER_REV — 200 for NEMA 17
//    → STEPS_PER_MM resolves automatically from these
//
//    ENCODER_CPR below must match your 17HS24-2004D-E1K encoder.
//    Typical value: 1000 CPR.  Confirm from the datasheet.
//
//  DIRECTION INVERSION
//  --------------------
//  If a motor moves the wrong direction, flip INVERT_A or
//  INVERT_B to true. No rewiring needed.
//
//  CONTROLS (single keypress, no newline needed)
//  -----------------------------------------------
//    W / S  — Jog +Y / -Y
//    A / D  — Jog -X / +X
//    + / -  — Double / halve jog step size
//    0      — Move to origin (0, 0)
//    P      — Print current estimated position
//    ?      — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testCoreXYJog_setup();
//    extern void testCoreXYJog_loop();
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

// Encoder counts per revolution for the 17HS24-2004D-E1K.
// Typical value: 1000.  Confirm from the motor's datasheet.
static constexpr int32_t ENCODER_CPR = 4000;   // TODO: confirm CPR

// Direction inversion — flip to true if a motor runs backwards.
//
// HOW TO DETERMINE CORRECT VALUES:
//   Press W (jog +Y).  In a CoreXY system this should make Motor A step
//   forward and Motor B step backward.  The gantry should move away from
//   the home corner in the Y direction.
//
//   If the gantry moves the WRONG way on W:
//     - Try INVERT_A = true, INVERT_B = true  (both DIR pins wired reversed)
//   If W is correct but A/D are swapped (left/right mixed up):
//     - Try INVERT_A = true, INVERT_B = false  (or vice versa)
//
// NOTE: INVERT only flips the DIR pin.  If the encoder counts the wrong
//   direction for a given motor (encoder A/B phases swapped on that motor),
//   swap ENC_A <-> ENC_B for that motor in PinMap.h — not via INVERT.
static constexpr bool INVERT_A = true;    // start true — flip back if wrong
static constexpr bool INVERT_B = true;    // start true — flip back if wrong

// ============================================================
//  JOG PARAMETERS
// ============================================================
static constexpr float JOG_STEP_MIN_MM = 1.0f;
static constexpr float JOG_STEP_MAX_MM = 20.0f;
static float g_jogMM = 5.0f;

// ============================================================
//  DEVICE POINTERS — set inside setup(), used in loop()
//
//  ClosedLoopStepper MUST be constructed inside setup(), not at
//  file scope. Its Encoder member calls attachInterrupt() in the
//  constructor; at file scope this runs before Arduino hardware
//  initialises and causes a crash / USB disconnect loop.
// ============================================================
static ClosedLoopStepper* g_motorA = nullptr;
static ClosedLoopStepper* g_motorB = nullptr;
static MotorController*   g_mc     = nullptr;

// ============================================================
//  PIN GUARD
// ============================================================
static bool pinsAssigned() {
    return (Pins::Gantry::MOTOR_A_STEP  >= 0 &&
            Pins::Gantry::MOTOR_A_DIR   >= 0 &&
            Pins::Gantry::MOTOR_A_ENC_A >= 0 &&
            Pins::Gantry::MOTOR_A_ENC_B >= 0 &&
            Pins::Gantry::MOTOR_B_STEP  >= 0 &&
            Pins::Gantry::MOTOR_B_DIR   >= 0 &&
            Pins::Gantry::MOTOR_B_ENC_A >= 0 &&
            Pins::Gantry::MOTOR_B_ENC_B >= 0);
}

// ============================================================
//  HELPERS
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# CoreXY Jog Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  W / S  — Jog +Y / -Y"));
    Serial.println(F("#  A / D  — Jog -X / +X"));
    Serial.println(F("#  + / -  — Double / halve jog step"));
    Serial.println(F("#  0      — Move to origin (0, 0)"));
    Serial.println(F("#  P      — Print current position"));
    Serial.println(F("#  ?      — This help"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Jog step:    ")); Serial.print(g_jogMM, 1); Serial.println(F(" mm"));
    Serial.print(F("# Steps/mm:    ")); Serial.println(Constants::Gantry::STEPS_PER_MM, 3);
    Serial.print(F("# Encoder CPR: ")); Serial.println(ENCODER_CPR);
    Serial.println(F("# ---- Pins ----"));
    Serial.print(F("# A STEP/DIR:  ")); Serial.print(Pins::Gantry::MOTOR_A_STEP); Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_A_DIR);
    Serial.print(F("# A ENC A/B:   ")); Serial.print(Pins::Gantry::MOTOR_A_ENC_A); Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_A_ENC_B);
    Serial.print(F("# B STEP/DIR:  ")); Serial.print(Pins::Gantry::MOTOR_B_STEP); Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_B_DIR);
    Serial.print(F("# B ENC A/B:   ")); Serial.print(Pins::Gantry::MOTOR_B_ENC_A); Serial.print(F(" / ")); Serial.println(Pins::Gantry::MOTOR_B_ENC_B);
    if (!pinsAssigned())
        Serial.println(F("# WARNING: One or more encoder pins still -1 in PinMap.h."));
    Serial.println(F("# ============================================="));
}

static void printPosition() {
    if (!g_mc) return;
    Serial.print(F("# X: ")); Serial.print(g_mc->getX(), 3);
    Serial.print(F(" mm   Y: ")); Serial.print(g_mc->getY(), 3);
    Serial.print(F(" mm   moving: ")); Serial.println(g_mc->isMoving() ? F("yes") : F("no"));
}

static void jogTo(float dx, float dy) {
    if (!g_mc) return;
    float nx = g_mc->getX() + dx;
    float ny = g_mc->getY() + dy;
    Serial.print(F("# Jog → X=")); Serial.print(nx, 2);
    Serial.print(F(" Y=")); Serial.println(ny, 2);
    g_mc->moveTo(nx, ny);
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testCoreXYJog_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    // Wait for the serial logger to reconnect after the Teensy
    // enumerates on USB — without this, early prints are lost.
    delay(2000);

    Serial.println(F("# [DBG 1] Serial ready"));
    Serial.flush();

    if (!pinsAssigned()) {
        Serial.println(F("# ERROR: Encoder pins not set in PinMap.h."));
        Serial.println(F("#   Set Gantry::MOTOR_x_ENC_A/B before running this test."));
        Serial.println(F("#   Halting — fix PinMap.h and re-upload."));
        while (true) {}
    }

    // Construct inside setup() — Encoder::attachInterrupt() must
    // run after Arduino hardware init, not at file scope.
    static ClosedLoopStepper motorA(
        Pins::Gantry::MOTOR_A_STEP,
        Pins::Gantry::MOTOR_A_DIR,
        Pins::Gantry::MOTOR_A_ENC_A,
        Pins::Gantry::MOTOR_A_ENC_B,
        Constants::Gantry::STEPS_PER_MM,
        /*stateIndex=*/ 0,
        ENCODER_CPR,
        INVERT_A
    );
    Serial.println(F("# [DBG 2] Motor A constructed"));
    Serial.flush();

    static ClosedLoopStepper motorB(
        Pins::Gantry::MOTOR_B_STEP,
        Pins::Gantry::MOTOR_B_DIR,
        Pins::Gantry::MOTOR_B_ENC_A,
        Pins::Gantry::MOTOR_B_ENC_B,
        Constants::Gantry::STEPS_PER_MM,
        /*stateIndex=*/ 1,
        ENCODER_CPR,
        INVERT_B
    );
    Serial.println(F("# [DBG 3] Motor B constructed"));
    Serial.flush();

    static MotorController mc;

    motorA.begin();
    motorB.begin();
    Serial.println(F("# [DBG 4] Stepper begin() done"));
    Serial.flush();

    mc.begin(motorA, motorB);
    Serial.println(F("# [DBG 5] MotorController begin() done"));
    Serial.flush();

    MachineState::instance().setStatus(SystemStatus::IDLE);

    g_motorA = &motorA;
    g_motorB = &motorB;
    g_mc     = &mc;

    printHelp();
}

void testCoreXYJog_loop() {
    if (!g_mc) return;

    // Rate-limit the control loop to 1 kHz — MotorController's Kalman/LQR
    // pipeline is tuned for a 1 ms timestep.  Calling update() at loop()
    // speed (hundreds of kHz) causes the velocity integrator to accumulate
    // too fast, overshoots the workspace boundary, and triggers the internal
    // e-stop after the very first jog command.
    static uint32_t lastControlUs = 0;
    uint32_t nowUs = micros();
    if (nowUs - lastControlUs >= 1000u) {
        lastControlUs = nowUs;
        g_mc->update();
    }

    // Warn once if the controller has e-stopped (hard-limit tripped)
    static bool ePrinted = false;
    if (g_mc->isEStopped() && !ePrinted) {
        ePrinted = true;
        Serial.println(F("# E-STOP: Kalman position exceeded workspace limit."));
        Serial.println(F("#   This can happen if encoder CPR or MICROSTEPPING mismatch."));
        Serial.println(F("#   Re-upload to reset, then verify Constants.h and ENCODER_CPR."));
    }
    if (!g_mc->isEStopped()) ePrinted = false;   // Clear if recovered

    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {
        case 'w': case 'W': jogTo( 0.0f,  g_jogMM); break;
        case 's': case 'S': jogTo( 0.0f, -g_jogMM); break;
        case 'a': case 'A': jogTo(-g_jogMM,  0.0f); break;
        case 'd': case 'D': jogTo( g_jogMM,  0.0f); break;

        case '+': case '=':
            g_jogMM = min(g_jogMM * 2.0f, JOG_STEP_MAX_MM);
            Serial.print(F("# Jog step: ")); Serial.print(g_jogMM, 1); Serial.println(F(" mm"));
            break;
        case '-': case '_':
            g_jogMM = max(g_jogMM * 0.5f, JOG_STEP_MIN_MM);
            Serial.print(F("# Jog step: ")); Serial.print(g_jogMM, 1); Serial.println(F(" mm"));
            break;

        case '0':
            Serial.println(F("# Moving to origin (0, 0)"));
            g_mc->moveTo(0.0f, 0.0f);
            break;

        case 'p': case 'P':
            printPosition();
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
