// ============================================================
//  testing/test_servo_sts.cpp
//
//  PURPOSE
//  -------
//  Validates STS3215 smart servo motion on real hardware.
//  Commands a single lift move and logs position feedback
//  as CSV so you can verify the 15 mm / 0.2 s requirement.
//
//  PRE-REQUISITES
//  --------------
//  1. Wire STS3215 data line to Teensy Serial7 (TX=29, RX=28).
//     For half-duplex: connect TX and RX through a 1 kΩ resistor
//     to a shared DATA line, or use a 74HC126 with STS3215_DIR.
//  2. Power the servo separately (7.4 V recommended).
//  3. Confirm servo ID = Constants::Servos::STS3215_SERVO_ID (default 1).
//  4. Set HOME_DEG to the fully-lowered angle of your mechanism.
//  5. Measure MM_PER_DEG for your specific linkage (see below).
//
//  MEASURING MM_PER_DEG
//  --------------------
//  Send 'C': the servo moves from HOME_DEG to HOME_DEG+10° slowly.
//  Measure the resulting linear displacement with a ruler.
//  MM_PER_DEG = measured_mm / 10.0
//
//  SPEED UNITS
//  -----------
//  The STS3215 goal-speed register is in position counts per second,
//  where 4095 counts = 360° (verify with your servo's datasheet —
//  some STS3215 variants use 300°; update COUNTS_PER_DEG if so).
//  speed = 0        → maximum speed (no limit)
//  speed = 1..4095  → controlled speed in counts/s
//  STS3215 no-load max ≈ 57 RPM ≈ 342 deg/s ≈ 3892 counts/s @ 7.4 V.
//
//  OUTPUT (CSV, 100 Hz)
//  --------------------
//  t_ms, target_deg, actual_deg, lift_est_mm
//
//  CONTROLS
//  --------
//    G  — lift to (HOME_DEG + LIFT_DEG) at computed speed
//    R  — return to HOME_DEG at same speed
//    C  — calibration crawl: move +10° slowly; measure mm displaced
//    S  — stop (hold current position by re-commanding it)
//    ?  — print this help
//
//  COMPILE
//  -------
//  platformio run -e teensy41_test
//  (main_test.cpp must forward to testServoSts_setup/loop)
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "devices/ServoController.h"
#include "math/Constants.h"

// ============================================================
//  User configuration — fill in before first run
// ============================================================

// Mechanical conversion: degrees of servo rotation → mm of linear lift.
// Common linkages:
//   Crank/arm:      MM_PER_DEG ≈ arm_radius_mm * PI / 180  (small-angle approx)
//   Rack & pinion:  MM_PER_DEG = (pitch_mm * pinion_teeth) / 360
// Use the 'C' calibration crawl to measure this experimentally.
static constexpr float MM_PER_DEG  = 0.5f;    // TODO: measure your mechanism

// Home angle — fully lowered / resting position of the mechanism (degrees).
static constexpr float HOME_DEG    = 90.0f;   // TODO: set your mechanism's rest angle

// Lift requirement
static constexpr float LIFT_MM     = 15.0f;   // Minimum required lift (mm)
static constexpr float LIFT_TIME_S = 0.2f;    // Required time (s)

// ============================================================
//  Derived — do not edit
// ============================================================
static constexpr float    LIFT_DEG   = LIFT_MM / MM_PER_DEG;
static constexpr float    LIFT_DEG_S = LIFT_DEG / LIFT_TIME_S;   // deg/s needed

// Speed in STS counts/s, clamped to [1, 4095].
// speed=0 means "no limit" (max speed), so floor at 1 to keep it controlled.
static constexpr float    LIFT_SPEED_F = LIFT_DEG_S * (4095.0f / 360.0f);
static constexpr uint16_t LIFT_SPEED   =
    (LIFT_SPEED_F < 1.0f)    ? uint16_t(1)    :
    (LIFT_SPEED_F > 4095.0f) ? uint16_t(4095) :
    static_cast<uint16_t>(LIFT_SPEED_F);

// STS3215 physical max (no-load, 7.4 V): ~342 deg/s
static constexpr float STS3215_MAX_DEG_S = 342.0f;

// Slow speed used for homing and calibration crawl
static constexpr uint16_t SLOW_SPEED = 200;

// ============================================================
//  Runtime state
// ============================================================
static bool     running     = false;
static bool     returning   = false;
static uint32_t moveStartMs = 0;
static float    targetDeg   = HOME_DEG;

// ============================================================
//  Forward declarations
// ============================================================
static void printHelp();
static void printCsvHeader();

// ============================================================
//  testServoSts_setup
// ============================================================
void testServoSts_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    ServoController::instance().begin();

    // Verify servo is reachable before doing anything else
    if (!ServoController::instance().sts3215Ping()) {
        Serial.println(F("# FATAL: STS3215 not responding."));
        Serial.println(F("#   Check: power supply, wiring (TX=29/RX=28), servo ID, baud rate."));
        while (true) {}
    }
    Serial.println(F("# STS3215 found."));

    // Drive to home position slowly on startup
    ServoController::instance().sts3215SetPosition(HOME_DEG, SLOW_SPEED);
    delay(1500);

    // Warn if the required speed exceeds what the servo can physically do
    if (LIFT_DEG_S > STS3215_MAX_DEG_S) {
        Serial.print(F("# WARNING: required "));
        Serial.print(LIFT_DEG_S, 1);
        Serial.print(F(" deg/s exceeds STS3215 max (~"));
        Serial.print(STS3215_MAX_DEG_S, 0);
        Serial.println(F(" deg/s)."));
        Serial.println(F("#   Increase MM_PER_DEG (change mechanism) or increase LIFT_TIME_S."));
    }

    printHelp();
    Serial.print(F("# Lift deg needed: ")); Serial.print(LIFT_DEG, 1);  Serial.println(F(" deg"));
    Serial.print(F("# Speed reg value: ")); Serial.println(LIFT_SPEED);
    Serial.print(F("# Target deg/s:    ")); Serial.print(LIFT_DEG_S, 1); Serial.println(F(" deg/s"));
    Serial.println(F("# Send 'G' to lift, 'R' to return, 'C' to calibrate, '?' for help."));
}

// ============================================================
//  testServoSts_loop
// ============================================================
void testServoSts_loop() {

    // ---- Heartbeat while idle ----
    if (!running && !returning) {
        static uint32_t lastHbMs = 0;
        if (millis() - lastHbMs >= 3000) {
            lastHbMs = millis();
            Serial.println(F("# alive — send G/R/C/S/?"));
        }
    }

    // ---- Serial commands ----
    if (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        switch (c) {

            case 'g': case 'G': {
                float tgt = HOME_DEG + LIFT_DEG;
                if (tgt > 360.0f) tgt = 360.0f;
                targetDeg = tgt;
                ServoController::instance().sts3215SetPosition(targetDeg, LIFT_SPEED);
                running     = true;
                returning   = false;
                moveStartMs = millis();
                printCsvHeader();
                Serial.print(F("# LIFT → ")); Serial.print(targetDeg, 1);
                Serial.print(F(" deg  speed=")); Serial.println(LIFT_SPEED);
                break;
            }

            case 'r': case 'R':
                targetDeg = HOME_DEG;
                ServoController::instance().sts3215SetPosition(HOME_DEG, LIFT_SPEED);
                returning   = true;
                running     = false;
                moveStartMs = millis();
                printCsvHeader();
                Serial.println(F("# RETURN → home"));
                break;

            case 'c': case 'C': {
                // Calibration crawl: move +10° slowly, user measures linear displacement
                float calTarget = HOME_DEG + 10.0f;
                if (calTarget > 360.0f) calTarget = 360.0f;
                ServoController::instance().sts3215SetPosition(calTarget, SLOW_SPEED);
                Serial.print(F("# CAL: moving +10 deg to "));
                Serial.print(calTarget, 1);
                Serial.println(F(" deg at slow speed."));
                Serial.println(F("# Measure linear displacement (mm), then set MM_PER_DEG = measured / 10.0"));
                break;
            }

            case 's': case 'S': {
                // Hold position by re-commanding current position
                float cur = ServoController::instance().sts3215GetPosition();
                if (cur < 0.0f) cur = HOME_DEG;
                targetDeg = cur;
                ServoController::instance().sts3215SetPosition(cur, SLOW_SPEED);
                running   = false;
                returning = false;
                Serial.print(F("# STOP — holding ")); Serial.print(cur, 1); Serial.println(F(" deg"));
                break;
            }

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

    if (!running && !returning) return;

    // ---- Log position at ~100 Hz ----
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs < 10) return;
    lastPrintMs = nowMs;

    float actual  = ServoController::instance().sts3215GetPosition();
    uint32_t t_ms = nowMs - moveStartMs;
    float liftEst = (actual >= 0.0f) ? (actual - HOME_DEG) * MM_PER_DEG : 0.0f;

    Serial.print(t_ms);          Serial.print(',');
    Serial.print(targetDeg, 2);  Serial.print(',');
    if (actual < 0.0f) { Serial.print(F("NaN")); } else { Serial.print(actual, 2); }
    Serial.print(',');
    Serial.println(liftEst, 3);

    // ---- Move completion: within 1.5° of target ----
    if (actual >= 0.0f && fabsf(actual - targetDeg) < 1.5f) {
        float finalLift = (actual - HOME_DEG) * MM_PER_DEG;
        Serial.print(F("# Reached target in ")); Serial.print(t_ms); Serial.println(F(" ms"));
        Serial.print(F("# Est. lift:    ")); Serial.print(finalLift, 2);   Serial.println(F(" mm"));
        if (t_ms > 0 && running) {
            float mmPerS = fabsf(finalLift) / (t_ms / 1000.0f);
            Serial.print(F("# Avg speed:    ")); Serial.print(mmPerS, 1); Serial.println(F(" mm/s"));
            if (mmPerS >= LIFT_MM / LIFT_TIME_S) {
                Serial.println(F("# PASS — lift speed meets requirement."));
            } else {
                Serial.println(F("# FAIL — lift too slow. Decrease MM_PER_DEG or check mechanism load."));
            }
        }
        running   = false;
        returning = false;
    }

    // ---- 2 s timeout ----
    if (t_ms > 2000) {
        Serial.print(F("# TIMEOUT — stalled at "));
        Serial.print(actual >= 0.0f ? actual : -1.0f, 1);
        Serial.println(F(" deg. Check power and torque limit (register 0x23)."));
        running   = false;
        returning = false;
    }
}

// ============================================================
//  printHelp
// ============================================================
static void printHelp() {
    Serial.println(F("# =============================="));
    Serial.println(F("# STS3215 Servo Lift Test"));
    Serial.println(F("# ------------------------------"));
    Serial.println(F("#  G — lift to target position"));
    Serial.println(F("#  R — return to home position"));
    Serial.println(F("#  C — calibration crawl (+10 deg slowly)"));
    Serial.println(F("#  S — stop and hold"));
    Serial.println(F("#  ? — this help"));
    Serial.println(F("# ------------------------------"));
    Serial.print(F("# Home:        ")); Serial.print(HOME_DEG, 1);  Serial.println(F(" deg"));
    Serial.print(F("# Lift angle:  ")); Serial.print(LIFT_DEG, 1);  Serial.println(F(" deg"));
    Serial.print(F("# Lift dist:   ")); Serial.print(LIFT_MM, 1);   Serial.println(F(" mm"));
    Serial.print(F("# Lift time:   ")); Serial.print(LIFT_TIME_S, 2); Serial.println(F(" s"));
    Serial.print(F("# MM_PER_DEG:  ")); Serial.println(MM_PER_DEG, 4);
    Serial.println(F("# =============================="));
}

// ============================================================
//  printCsvHeader
// ============================================================
static void printCsvHeader() {
    Serial.println(F("t_ms,target_deg,actual_deg,lift_est_mm"));
}

#endif // TEST_MODE
