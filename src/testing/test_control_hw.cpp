// ============================================================
//  testing/test_control_hw.cpp
//
//  PURPOSE
//  -------
//  Unified hardware test for the CoreXY gantry and the STS3215
//  smart servo.  Subsystems can be exercised independently or
//  together.
//
//  PRE-REQUISITES — GANTRY
//  -----------------------
//  1. Fill in all Pins::Gantry::* values in PinMap.h.
//  2. Set ENCODER_CPR to your motor's counts per revolution.
//  3. Home the gantry ('H') before issuing any gantry move.
//  4. Fill in Constants.h:
//       - LQR::K              (from solve_lqr_gains.m)
//       - Gantry::CARRIAGE_MASS_KG  (measured mass)
//       - Gantry::FRICTION_COEFF    (from constant-vel sweep)
//
//  PRE-REQUISITES — SERVO
//  ----------------------
//  1. Wire STS3215 data line to Serial7 (TX=pin 29, RX=pin 28).
//  2. Set SERVO_HOME_DEG to the fully-lowered angle.
//  3. Measure MM_PER_DEG using the 'C' calibration crawl:
//       command 'C', measure displacement, set MM_PER_DEG = mm/10.
//
//  CONTROLS (single character, no newline)
//  ----------------------------------------
//    H  — home gantry (drives toward endstops, zeros encoder)
//    G  — move gantry to (HW_TARGET_X, HW_TARGET_Y)
//    U  — servo Up: lift to (SERVO_HOME_DEG + LIFT_DEG)
//    D  — servo Down: return servo to SERVO_HOME_DEG
//    B  — Both: move gantry + lift servo simultaneously
//    R  — reset gantry (zero encoder + Kalman; re-home required)
//    C  — servo calibration crawl (+10° slowly; measure mm displaced)
//    S  — stop all (gantry and servo)
//    ?  — print this help
//
//  OUTPUT (CSV over USB Serial)
//  ----------------------------
//  Gantry only ('G'):
//    t_ms, est_x, est_y, est_vx, est_vy,
//    foll_err_A, foll_err_B, err_x, err_y
//
//  Servo only ('U'/'D'):
//    t_ms, target_deg, actual_deg, lift_est_mm
//
//  Both ('B'):
//    t_ms, est_x, est_y, est_vx, est_vy,
//    foll_err_A, foll_err_B, err_x, err_y,
//    target_deg, actual_deg, lift_est_mm
//
//  COMPILE
//  -------
//  platformio run -e teensy41_test
//  (main_test.cpp must forward to testControlHw_setup/loop)
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "control/MotorController.h"
#include "devices/ClosedLoopStepper.h"
#include "devices/Microswitch.h"
#include "devices/ServoController.h"
#include "math/Constants.h"
#include "pins/PinMap.h"

// ============================================================
//  Gantry parameters — edit before first run
// ============================================================

// Target Cartesian position for 'G' and 'B' commands (mm)
static constexpr float HW_TARGET_X = 20.0f;
static constexpr float HW_TARGET_Y = 20.0f;

// Encoder counts per revolution of the motor shaft.
// TODO: Confirm with spec sheet or by rotating one rev and reading encoder_.read().
// Common values for 17HS24-2004D-E1K: 1000 or 4000.
static constexpr int32_t ENCODER_CPR = 4000;

// Following-error fault threshold (mm).  Start loose; tighten once stable.
static constexpr float FAULT_THRESHOLD_MM =
    Constants::Gantry::FOLLOWING_ERROR_FAULT_MM;

// CSV decimation: print every N gantry ticks → 1000/N Hz output
static constexpr int CSV_PRINT_EVERY_N_TICKS = 10;

// ============================================================
//  Servo parameters — edit before first run
// ============================================================

// Mechanical conversion: degrees of servo rotation → mm of linear lift.
// Measure with the 'C' calibration crawl: command 'C', read the mm displaced,
// set MM_PER_DEG = measured_mm / 10.0.
//   Crank/arm:      MM_PER_DEG ≈ arm_radius_mm * PI / 180
//   Rack & pinion:  MM_PER_DEG = (pitch_mm * pinion_teeth) / 360
static constexpr float MM_PER_DEG    = 0.5f;   // TODO: measure your mechanism

// Angle at which the mechanism is fully lowered / at rest (degrees)
static constexpr float SERVO_HOME_DEG = 90.0f;  // TODO: set rest angle

// Lift requirement (used to compute LIFT_SPEED automatically)
static constexpr float LIFT_MM       = 15.0f;
static constexpr float LIFT_TIME_S   = 0.2f;

// Derived servo constants — do not edit
static constexpr float    LIFT_DEG      = LIFT_MM / MM_PER_DEG;
static constexpr float    LIFT_DEG_S    = LIFT_DEG / LIFT_TIME_S;
// STS3215 speed register: counts/s where 4095 counts = 360°.
// speed=0 means maximum speed; clamp computed value to [1, 4095].
static constexpr float    LIFT_SPEED_F  = LIFT_DEG_S * (4095.0f / 360.0f);
static constexpr uint16_t LIFT_SPEED    =
    (LIFT_SPEED_F < 1.0f)    ? uint16_t(1)    :
    (LIFT_SPEED_F > 4095.0f) ? uint16_t(4095) :
    static_cast<uint16_t>(LIFT_SPEED_F);
static constexpr uint16_t SLOW_SPEED    = 200;    // Used for calibration crawl
static constexpr float    STS_MAX_DEG_S = 342.0f; // STS3215 no-load max (7.4 V)

// ============================================================
//  Hardware instances
//
//  motorA and motorB are constructed inside testControlHw_setup()
//  (not at file scope) because the Encoder library calls
//  attachInterrupt() and delayMicroseconds() in its constructor.
//  Both require the Arduino framework to be fully initialised,
//  which has not happened yet when file-scope static constructors run.
//  Constructing them inside setup() avoids the pre-init hard fault.
// ============================================================
static ClosedLoopStepper* motorA = nullptr;
static ClosedLoopStepper* motorB = nullptr;
static MotorController mc;

static Microswitch switchX(Pins::Switches::SWITCH_X, SwitchAxis::X);
static Microswitch switchY(Pins::Switches::SWITCH_Y, SwitchAxis::Y);

// ============================================================
//  Gantry runtime state
// ============================================================
static bool     homed            = false;
static bool     homing           = false;
static bool     running          = false;   // gantry move active
static bool     faulted          = false;
static uint32_t tickCount        = 0;
static uint32_t moveStartUs      = 0;

// ============================================================
//  Servo runtime state
// ============================================================
static bool     servoFound       = false;   // true if ping succeeded at startup
static bool     servoActive      = false;   // servo move active
static float    servoTargetDeg   = SERVO_HOME_DEG;
static uint32_t servoStartMs     = 0;
static float    lastServoAngle   = -1.0f;   // cached from last poll
static uint32_t lastServoPollMs  = 0;

// ============================================================
//  Forward declarations
// ============================================================
static void doReset();
static void doStop();
static void startHoming();
static void servoUp();
static void servoDown();
static void printHelp();

// CSV helpers — three variants for different active subsystem combinations
static void printCsvHeader(bool gantryOn, bool servoOn);
static void printCsvRow(bool gantryOn, bool servoOn,
                        uint32_t t_ms,
                        float est_x,   float est_y,
                        float est_vx,  float est_vy,
                        float ferrA,   float ferrB,
                        float servoActual);

// ============================================================
//  doReset — zeros gantry; servo position is unaffected
// ============================================================
static void doReset() {
    mc.stop();
    mc.resetPosition(0.0f, 0.0f);
    motorA->zeroPosition();
    motorB->zeroPosition();

    homed    = false;
    homing   = false;
    running  = false;
    faulted  = false;
    tickCount = 0;

    Serial.println(F("# RESET — gantry zeroed. Re-home before moving."));
}

// ============================================================
//  doStop — stops gantry and holds servo at current position
// ============================================================
static void doStop() {
    mc.stop();
    running = false;

    if (servoFound && servoActive) {
        // Re-command current angle to hold position
        float cur = ServoController::instance().sts3215GetPosition();
        if (cur >= 0.0f) {
            ServoController::instance().sts3215SetPosition(cur, SLOW_SPEED);
            lastServoAngle = cur;
        }
        servoActive = false;
    }

    Serial.println(F("# STOP"));
}

// ============================================================
//  startHoming
// ============================================================
static void startHoming() {
    if (homing) return;
    mc.stop();
    running = false;
    homing  = true;
    homed   = false;
    Serial.println(F("# HOMING — crawling toward endstops..."));
}

// ============================================================
//  servoUp — lift the servo to HOME + LIFT_DEG
// ============================================================
static void servoUp() {
    if (!servoFound) {
        Serial.println(F("# Servo not found — check wiring and re-power."));
        return;
    }
    float tgt = SERVO_HOME_DEG + LIFT_DEG;
    if (tgt > 360.0f) tgt = 360.0f;
    servoTargetDeg = tgt;
    ServoController::instance().sts3215SetPosition(servoTargetDeg, LIFT_SPEED);
    servoActive  = true;
    servoStartMs = millis();
}

// ============================================================
//  servoDown — return servo to HOME_DEG
// ============================================================
static void servoDown() {
    if (!servoFound) {
        Serial.println(F("# Servo not found — check wiring and re-power."));
        return;
    }
    servoTargetDeg = SERVO_HOME_DEG;
    ServoController::instance().sts3215SetPosition(SERVO_HOME_DEG, LIFT_SPEED);
    servoActive  = true;
    servoStartMs = millis();
}

// ============================================================
//  printHelp
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# Combined Hardware Test (Gantry + STS3215)"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  H — home gantry"));
    Serial.println(F("#  G — move gantry to target (XY only)"));
    Serial.println(F("#  U — servo Up: lift to target angle"));
    Serial.println(F("#  D — servo Down: return to home angle"));
    Serial.println(F("#  B — Both: move gantry + lift servo"));
    Serial.println(F("#  R — reset gantry (re-home required)"));
    Serial.println(F("#  C — servo calibration crawl (+10 deg slowly)"));
    Serial.println(F("#  S — stop all"));
    Serial.println(F("#  ? — this help"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Gantry target: X=")); Serial.print(HW_TARGET_X, 1);
    Serial.print(F(" mm  Y="));             Serial.print(HW_TARGET_Y, 1);
    Serial.println(F(" mm"));
    Serial.print(F("# Homed:         ")); Serial.println(homed ? F("YES") : F("NO"));
    Serial.print(F("# LQR K:         ["));
    Serial.print(Constants::LQR::K[0][0], 3); Serial.print(F(", "));
    Serial.print(Constants::LQR::K[0][1], 3); Serial.print(F(", "));
    Serial.print(Constants::LQR::K[0][2], 3); Serial.println(F("]"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Servo:         ")); Serial.println(servoFound ? F("FOUND") : F("NOT FOUND"));
    Serial.print(F("# Servo home:    ")); Serial.print(SERVO_HOME_DEG, 1); Serial.println(F(" deg"));
    Serial.print(F("# Lift angle:    ")); Serial.print(LIFT_DEG, 1);       Serial.println(F(" deg"));
    Serial.print(F("# Lift dist:     ")); Serial.print(LIFT_MM, 1);        Serial.println(F(" mm"));
    Serial.print(F("# Lift speed:    ")); Serial.println(LIFT_SPEED);
    Serial.print(F("# MM_PER_DEG:    ")); Serial.println(MM_PER_DEG, 4);
    Serial.println(F("# ============================================="));
}

// ============================================================
//  printCsvHeader
// ============================================================
static void printCsvHeader(bool gantryOn, bool servoOn) {
    if (gantryOn) {
        Serial.print(F("t_ms,est_x,est_y,est_vx,est_vy,foll_err_A,foll_err_B,err_x,err_y"));
        if (servoOn) Serial.print(F(",target_deg,actual_deg,lift_est_mm"));
    } else {
        Serial.print(F("t_ms,target_deg,actual_deg,lift_est_mm"));
    }
    Serial.println();
}

// ============================================================
//  printCsvRow
// ============================================================
static void printCsvRow(bool gantryOn, bool servoOn,
                        uint32_t t_ms,
                        float est_x,  float est_y,
                        float est_vx, float est_vy,
                        float ferrA,  float ferrB,
                        float servoActual)
{
    Serial.print(t_ms); Serial.print(',');

    if (gantryOn) {
        Serial.print(est_x,  4); Serial.print(',');
        Serial.print(est_y,  4); Serial.print(',');
        Serial.print(est_vx, 4); Serial.print(',');
        Serial.print(est_vy, 4); Serial.print(',');
        Serial.print(ferrA,  4); Serial.print(',');
        Serial.print(ferrB,  4); Serial.print(',');
        Serial.print(HW_TARGET_X - est_x, 4); Serial.print(',');
        if (!servoOn) {
            Serial.println(HW_TARGET_Y - est_y, 4);
            return;
        }
        Serial.print(HW_TARGET_Y - est_y, 4); Serial.print(',');
    }

    // Servo columns
    Serial.print(servoTargetDeg, 2); Serial.print(',');
    if (servoActual < 0.0f) { Serial.print(F("NaN")); }
    else                    { Serial.print(servoActual, 2); }
    Serial.print(',');
    float liftEst = (servoActual >= 0.0f) ? (servoActual - SERVO_HOME_DEG) * MM_PER_DEG : 0.0f;
    Serial.println(liftEst, 3);
}

// ============================================================
//  testControlHw_setup
// ============================================================
void testControlHw_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    // Wait 2 s so the logger's 1.5 s reconnect delay expires and it is
    // actively reading before we start printing diagnostic output.
    // Without this delay, the host closes its buffer during reconnect
    // and all setup() prints are discarded before they can be read.
    delay(2000);

    Serial.println(F("# [DBG 1] Serial ready"));
    Serial.flush();

    // ---- Gantry pin guard ----
    if (Pins::Gantry::MOTOR_A_STEP == -1 ||
        Pins::Gantry::MOTOR_B_STEP == -1 ||
        Pins::Gantry::MOTOR_A_ENC_A == -1 ||
        Pins::Gantry::MOTOR_B_ENC_A == -1)
    {
        Serial.println(F("# FATAL: Gantry pins not assigned in PinMap.h — halting."));
        while (true) {}
    }

    Serial.println(F("# [DBG 2] Pin guard passed"));
    Serial.flush();

    // Construct here (not at file scope) so the Encoder library's
    // attachInterrupt() runs after Arduino framework init.
    static ClosedLoopStepper _motorA(
        Pins::Gantry::MOTOR_A_STEP,
        Pins::Gantry::MOTOR_A_DIR,
        Pins::Gantry::MOTOR_A_ENC_A,
        Pins::Gantry::MOTOR_A_ENC_B,
        Constants::Gantry::STEPS_PER_MM,
        /*stateIndex=*/0,
        ENCODER_CPR,
        /*invertDir=*/false
    );

    Serial.println(F("# [DBG 3] motorA constructed"));
    Serial.flush();

    static ClosedLoopStepper _motorB(
        Pins::Gantry::MOTOR_B_STEP,
        Pins::Gantry::MOTOR_B_DIR,
        Pins::Gantry::MOTOR_B_ENC_A,
        Pins::Gantry::MOTOR_B_ENC_B,
        Constants::Gantry::STEPS_PER_MM,
        /*stateIndex=*/1,
        ENCODER_CPR,
        /*invertDir=*/false
    );
    motorA = &_motorA;
    motorB = &_motorB;

    Serial.println(F("# [DBG 4] motorB constructed"));
    Serial.flush();

    motorA->begin();
    motorB->begin();

    Serial.println(F("# [DBG 5] stepper begin() done"));
    Serial.flush();

    switchX.begin();
    switchY.begin();
    mc.begin(*motorA, *motorB);

    Serial.println(F("# [DBG 6] mc.begin() done"));
    Serial.flush();

    // ---- Servo init (non-fatal if absent) ----
    ServoController::instance().begin();

    Serial.println(F("# [DBG 7] ServoController.begin() done"));
    Serial.flush();

    servoFound = ServoController::instance().sts3215Ping();

    Serial.println(F("# [DBG 8] sts3215Ping() done"));
    Serial.flush();

    if (servoFound) {
        ServoController::instance().sts3215SetPosition(SERVO_HOME_DEG, SLOW_SPEED);
        if (LIFT_DEG_S > STS_MAX_DEG_S) {
            Serial.print(F("# WARN: Required servo speed (")); Serial.print(LIFT_DEG_S, 1);
            Serial.print(F(" deg/s) exceeds STS3215 max (~")); Serial.print(STS_MAX_DEG_S, 0);
            Serial.println(F(" deg/s). Increase MM_PER_DEG or LIFT_TIME_S."));
        }
    } else {
        Serial.println(F("# Servo not found — gantry-only commands still work."));
    }

    printHelp();
    Serial.println(F("# Send 'H' to home gantry, then 'G'/'U'/'B' to move."));
}

// ============================================================
//  testControlHw_loop — called at ~1 kHz by loop()
// ============================================================
void testControlHw_loop() {

    // ---- Heartbeat while idle ----
    if (!running && !homing && !servoActive) {
        static uint32_t lastHbMs = 0;
        uint32_t now = millis();
        if (now - lastHbMs >= 3000) {
            lastHbMs = now;
            Serial.print(F("# alive — homed="));
            Serial.print(homed ? F("YES") : F("NO"));
            Serial.println(F("  send H/G/U/D/B/R/C/S/?"));
        }
    }

    // ---- Serial command handler ----
    if (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        switch (c) {

            // ---- Gantry: home ----
            case 'h': case 'H':
                startHoming();
                break;

            // ---- Gantry only: move to target ----
            case 'g': case 'G':
                if (!homed)   { Serial.println(F("# Not homed — send 'H' first."));   break; }
                if (faulted)  { Serial.println(F("# Faulted — send 'R' to reset."));  break; }
                mc.moveTo(HW_TARGET_X, HW_TARGET_Y);
                running     = true;
                tickCount   = 0;
                moveStartUs = micros();
                printCsvHeader(/*gantry=*/true, /*servo=*/false);
                Serial.print(F("# GO gantry → X=")); Serial.print(HW_TARGET_X, 1);
                Serial.print(F(" Y="));               Serial.println(HW_TARGET_Y, 1);
                break;

            // ---- Servo only: lift up ----
            case 'u': case 'U':
                servoUp();
                if (servoFound) {
                    printCsvHeader(/*gantry=*/false, /*servo=*/true);
                    Serial.print(F("# LIFT → ")); Serial.print(servoTargetDeg, 1);
                    Serial.print(F(" deg @ speed ")); Serial.println(LIFT_SPEED);
                }
                break;

            // ---- Servo only: return home ----
            case 'd': case 'D':
                servoDown();
                if (servoFound) {
                    printCsvHeader(/*gantry=*/false, /*servo=*/true);
                    Serial.print(F("# LOWER → ")); Serial.print(SERVO_HOME_DEG, 1);
                    Serial.println(F(" deg"));
                }
                break;

            // ---- Both: gantry move + servo lift ----
            case 'b': case 'B':
                if (!homed)   { Serial.println(F("# Not homed — send 'H' first."));   break; }
                if (faulted)  { Serial.println(F("# Faulted — send 'R' to reset."));  break; }
                mc.moveTo(HW_TARGET_X, HW_TARGET_Y);
                running     = true;
                tickCount   = 0;
                moveStartUs = micros();
                servoUp();
                printCsvHeader(/*gantry=*/true, /*servo=*/servoFound);
                Serial.print(F("# BOTH — gantry to X=")); Serial.print(HW_TARGET_X, 1);
                Serial.print(F(" Y="));                   Serial.print(HW_TARGET_Y, 1);
                if (servoFound) {
                    Serial.print(F("  servo → ")); Serial.print(servoTargetDeg, 1);
                    Serial.print(F(" deg @ ")); Serial.print(LIFT_SPEED);
                }
                Serial.println();
                break;

            // ---- Gantry: reset ----
            case 'r': case 'R':
                doReset();
                break;

            // ---- Servo: calibration crawl ----
            case 'c': case 'C':
                if (!servoFound) { Serial.println(F("# Servo not found.")); break; }
                {
                    float calTgt = SERVO_HOME_DEG + 10.0f;
                    if (calTgt > 360.0f) calTgt = 360.0f;
                    ServoController::instance().sts3215SetPosition(calTgt, SLOW_SPEED);
                    Serial.print(F("# CAL: moving +10 deg to ")); Serial.print(calTgt, 1);
                    Serial.println(F(" deg slowly."));
                    Serial.println(F("# Measure linear displacement (mm), then set: MM_PER_DEG = mm / 10.0"));
                }
                break;

            // ---- Stop all ----
            case 's': case 'S':
                doStop();
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

    // ---- Homing state machine ----
    if (homing) {
        switchX.update();
        switchY.update();
        bool done = mc.homingStep(switchX.isTripped(), switchY.isTripped());
        if (done) {
            homing = false;
            homed  = true;
            Serial.println(F("# HOMED — ready. Send 'G', 'U', or 'B'."));
        }
        delayMicroseconds(static_cast<unsigned int>(
            Constants::Timing::CONTROL_LOOP_DT * 1e6f));
        return;
    }

    // ---- Nothing active ----
    if (!running && !servoActive) {
        delayMicroseconds(static_cast<unsigned int>(
            Constants::Timing::CONTROL_LOOP_DT * 1e6f));
        return;
    }

    // ============================================================
    //  Servo poll — runs at ~100 Hz regardless of gantry state.
    //  The read takes < 1 ms at 1 Mbaud; a failed read takes up
    //  to READ_TIMEOUT_MS (5 ms) but is rare.
    // ============================================================
    uint32_t nowMs = millis();
    if (servoActive && (nowMs - lastServoPollMs >= 10)) {
        lastServoPollMs = nowMs;
        lastServoAngle  = ServoController::instance().sts3215GetPosition();

        // Check servo completion
        if (lastServoAngle >= 0.0f &&
            fabsf(lastServoAngle - servoTargetDeg) < 1.5f)
        {
            uint32_t elapsed = nowMs - servoStartMs;
            float finalLift  = (lastServoAngle - SERVO_HOME_DEG) * MM_PER_DEG;
            Serial.print(F("# Servo reached target in ")); Serial.print(elapsed); Serial.println(F(" ms"));
            Serial.print(F("# Est. lift: ")); Serial.print(finalLift, 2); Serial.println(F(" mm"));
            if (elapsed > 0 && finalLift > 0.0f) {
                float mmPerS = finalLift / (elapsed / 1000.0f);
                Serial.print(F("# Avg speed: ")); Serial.print(mmPerS, 1); Serial.println(F(" mm/s"));
                if (mmPerS >= LIFT_MM / LIFT_TIME_S)
                    Serial.println(F("# Servo PASS — meets lift requirement."));
                else
                    Serial.println(F("# Servo FAIL — too slow. Check load and MM_PER_DEG."));
            }
            servoActive = false;
        }

        // Servo timeout
        if (servoActive && (nowMs - servoStartMs > 2000)) {
            Serial.print(F("# Servo TIMEOUT at "));
            Serial.print(lastServoAngle, 1);
            Serial.println(F(" deg. Check power and torque limit (reg 0x23)."));
            servoActive = false;
        }

        // If only servo was running (no gantry), print servo CSV row
        if (!running && servoActive) {
            uint32_t t_ms = nowMs - servoStartMs;
            printCsvRow(/*gantry=*/false, /*servo=*/true,
                        t_ms, 0, 0, 0, 0, 0, 0, lastServoAngle);
        }
    }

    // ---- Gantry pipeline ----
    if (!running) {
        delayMicroseconds(static_cast<unsigned int>(
            Constants::Timing::CONTROL_LOOP_DT * 1e6f));
        return;
    }

    // 1. Following-error fault check
    float ferrA = motorA->getFollowingErrorMM();
    float ferrB = motorB->getFollowingErrorMM();
    if (fabsf(ferrA) > FAULT_THRESHOLD_MM ||
        fabsf(ferrB) > FAULT_THRESHOLD_MM)
    {
        mc.eStop();
        running = false;
        faulted = true;
        Serial.print(F("# FAULT — following error: A="));
        Serial.print(ferrA, 3);
        Serial.print(F(" mm  B="));
        Serial.print(ferrB, 3);
        Serial.println(F(" mm  Send 'R' to reset."));
        return;
    }

    // 2. Run control pipeline
    bool gantryDone = mc.update();

    // 3. Read Kalman state
    float est_x  = mc.getX();
    float est_y  = mc.getY();
    float est_vx = mc.getVx();
    float est_vy = mc.getVy();

    // 4. CSV output (decimated)
    if (tickCount % CSV_PRINT_EVERY_N_TICKS == 0) {
        uint32_t t_ms = (micros() - moveStartUs) / 1000UL;
        printCsvRow(/*gantry=*/true, /*servo=*/servoActive || (lastServoAngle >= 0.0f && servoFound),
                    t_ms,
                    est_x, est_y, est_vx, est_vy,
                    ferrA, ferrB,
                    lastServoAngle);
    }

    // 5. Gantry completion: run 1 extra second of settling data
    if (gantryDone) {
        static uint32_t doneAt = 0;
        if (doneAt == 0) {
            doneAt = tickCount;
            Serial.println(F("# Gantry trajectory complete — printing 1 s settling data..."));
        }
        if (tickCount - doneAt >= 1000) {
            running = false;
            doneAt  = 0;
            Serial.print(F("# Gantry done. Final error: X="));
            Serial.print(HW_TARGET_X - mc.getX(), 3);
            Serial.print(F(" mm  Y="));
            Serial.print(HW_TARGET_Y - mc.getY(), 3);
            Serial.println(F(" mm"));
            Serial.println(F("# Send 'R'+'G' to repeat, or 'S' to hold."));
        }
    }

    tickCount++;

    delayMicroseconds(static_cast<unsigned int>(
        Constants::Timing::CONTROL_LOOP_DT * 1e6f));
}

#endif // TEST_MODE
