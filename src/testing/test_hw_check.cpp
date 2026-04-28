// ============================================================
//  testing/test_hw_check.cpp
//
//  PURPOSE
//  -------
//  Pre-flight hardware validation for the CoreXY transfer robot.
//  Run this after any wiring change or before first powered test.
//  Results are printed as a PASS / FAIL / WARN / SKIP table.
//
//  CHECKS (auto-run, no motion)
//  ----------------------------
//  1. Pin assignment audit  — reports which pins are still -1
//  2. STS3215 ping + position read
//  3. Limit switch readback — reports OPEN / TRIPPED
//  4. TSL1401 line sensor ADC read
//  5. Encoder stability     — reads for 500 ms; flags drift
//
//  CHECKS (interactive — press Y to run each one)
//  -----------------------------------------------
//  6. Gantry motion + encoder response
//       Pulses both motors forward ~2 mm, verifies encoder moved,
//       then reverses to origin.  Reports if encoder is backwards,
//       missing, or has excessive following error.
//  7. MG996R servo sweep  (if any pins are assigned)
//       Moves each assigned servo to 45° then back to 90°.
//       No feedback — WARN is normal; listen/watch for motion.
//
//  COMPILE / RUN
//  -------------
//  1. In main_test.cpp, activate:
//       extern void testHwCheck_setup();
//       extern void testHwCheck_loop();
//     and call them from setup() / loop().
//  2.  platformio run -e teensy41_test -t upload
//  3.  python serial_logger.py
//
//  SAFE-TO-USE BEFORE HOMING
//  -------------------------
//  The motion test jogs only JOG_STEPS steps (~2 mm at default
//  STEPS_PER_MM) at a slow rate.  Keep the gantry at least 5 mm
//  away from limit switches before running it.
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "devices/ClosedLoopStepper.h"
#include "devices/ServoController.h"
#include "math/Constants.h"
#include "pins/PinMap.h"

// ============================================================
//  Configuration — match to your setup
// ============================================================

// Encoder counts per revolution of the motor shaft.
// Must match the value in test_control_hw.cpp.
static constexpr int32_t CHECK_ENCODER_CPR = 4000;

// Number of steps to jog during the motion test.
// At STEPS_PER_MM ≈ 200, this is ~2 mm.
static constexpr int32_t JOG_STEPS = 400;

// Step pulse interval for the motion test (µs).
// 500 µs → 2 kHz, which is slow and safe for a diagnostic.
static constexpr uint32_t JOG_PULSE_US = 500;

// Minimum encoder displacement (mm) required to call a motor PASS.
static constexpr float ENC_MIN_MOVE_MM = 0.3f;

// Maximum allowed return error (mm) — encoder should be near 0 after jogging back.
static constexpr float ENC_MAX_RETURN_ERR_MM = 1.0f;

// Acceptable encoder drift over 500 ms while stationary (ticks).
static constexpr int32_t ENC_MAX_STATIC_DRIFT = 5;

// ============================================================
//  Result enum and printer
// ============================================================
enum class R : uint8_t { PASS, FAIL, WARN, SKIP };

static uint8_t g_fails = 0;

static void result(const __FlashStringHelper* label, R r, const char* detail = nullptr) {
    Serial.print(F("  "));
    switch (r) {
        case R::PASS: Serial.print(F("[PASS]  ")); break;
        case R::FAIL: Serial.print(F("[FAIL]  ")); g_fails++; break;
        case R::WARN: Serial.print(F("[WARN]  ")); break;
        case R::SKIP: Serial.print(F("[SKIP]  ")); break;
    }
    Serial.print(label);
    if (detail && detail[0]) { Serial.print(F(" — ")); Serial.print(detail); }
    Serial.println();
    Serial.flush();
}

// Tiny helper to build a short string on the stack
#define MKBUF(buf, ...) do { snprintf(buf, sizeof(buf), __VA_ARGS__); } while (0)

// ============================================================
//  Wait for a keypress with timeout.
//  Returns the character pressed, or '\0' on timeout / no input.
// ============================================================
static char waitKey(uint32_t timeoutMs = 15000) {
    while (Serial.available()) Serial.read();   // flush RX
    uint32_t t0 = millis();
    while (!Serial.available() && millis() - t0 < timeoutMs) {}
    if (!Serial.available()) return '\0';
    char c = static_cast<char>(Serial.read());
    Serial.println(c);  // echo
    return c;
}

// ============================================================
//  Stage 1 — Pin assignment audit
// ============================================================
static bool stagePinAudit() {
    Serial.println(F("\n[1] Pin assignment audit"));

    struct { const __FlashStringHelper* name; int pin; } pins[] = {
        { F("Gantry: MotorA STEP"),  Pins::Gantry::MOTOR_A_STEP  },
        { F("Gantry: MotorA DIR"),   Pins::Gantry::MOTOR_A_DIR   },
        { F("Gantry: MotorA ENC_A"), Pins::Gantry::MOTOR_A_ENC_A },
        { F("Gantry: MotorA ENC_B"), Pins::Gantry::MOTOR_A_ENC_B },
        { F("Gantry: MotorB STEP"),  Pins::Gantry::MOTOR_B_STEP  },
        { F("Gantry: MotorB DIR"),   Pins::Gantry::MOTOR_B_DIR   },
        { F("Gantry: MotorB ENC_A"), Pins::Gantry::MOTOR_B_ENC_A },
        { F("Gantry: MotorB ENC_B"), Pins::Gantry::MOTOR_B_ENC_B },
        { F("Servo:  STS3215 TX"),   Pins::Servos::STS3215_TX    },
        { F("Servo:  MG996R_1"),     Pins::Servos::MG996R_1      },
        { F("Servo:  MG996R_2"),     Pins::Servos::MG996R_2      },
        { F("Servo:  MG996R_3"),     Pins::Servos::MG996R_3      },
        { F("Servo:  MG996R_4"),     Pins::Servos::MG996R_4      },
        { F("Switch: X"),            Pins::Switches::SWITCH_X    },
        { F("Switch: Y"),            Pins::Switches::SWITCH_Y    },
        { F("Switch: Z"),            Pins::Switches::SWITCH_Z    },
        { F("Sensor: TSL1401 AO"),   Pins::LaserSensor::AO       },
        { F("Sensor: TSL1401 SI"),   Pins::LaserSensor::SI       },
        { F("Sensor: TSL1401 CLK"),  Pins::LaserSensor::CLK      },
        { F("Laser:  Line enable"),  Pins::LineLaser::ENABLE     },
    };

    bool gantryOK = true;
    char buf[12];
    for (auto& p : pins) {
        if (p.pin < 0) {
            result(p.name, R::SKIP, "unassigned (-1)");
        } else {
            MKBUF(buf, "pin %d", p.pin);
            result(p.name, R::PASS, buf);
        }
    }

    // Gantry is critical — flag if any core pin is missing
    if (Pins::Gantry::MOTOR_A_STEP < 0 || Pins::Gantry::MOTOR_B_STEP < 0 ||
        Pins::Gantry::MOTOR_A_ENC_A < 0 || Pins::Gantry::MOTOR_B_ENC_A < 0)
    {
        result(F("Gantry critical pins"), R::FAIL,
               "one or more STEP/ENC_A pins unassigned — motion tests skipped");
        gantryOK = false;
    }

    return gantryOK;
}

// ============================================================
//  Stage 2 — STS3215 smart servo
// ============================================================
static void stageSTS3215() {
    Serial.println(F("\n[2] STS3215 smart servo"));

    ServoController& sc = ServoController::instance();
    sc.begin();

    if (!sc.sts3215Ping()) {
        result(F("STS3215 ping"), R::FAIL,
               "no response — check 5V power, data wire on pin 29, servo ID=1");
        return;
    }
    result(F("STS3215 ping"), R::PASS);

    float pos = sc.sts3215GetPosition();
    if (pos < 0.0f) {
        result(F("STS3215 position read"), R::WARN, "ping OK but read failed");
    } else {
        char buf[32];
        MKBUF(buf, "%.1f deg", pos);
        result(F("STS3215 position read"), R::PASS, buf);
    }
}

// ============================================================
//  Stage 3 — Limit switches
// ============================================================
static void stageSwitches() {
    Serial.println(F("\n[3] Limit switches  (normally-open, INPUT_PULLUP)"));

    struct { const __FlashStringHelper* name; int pin; } sw[] = {
        { F("Switch X"), Pins::Switches::SWITCH_X },
        { F("Switch Y"), Pins::Switches::SWITCH_Y },
        { F("Switch Z"), Pins::Switches::SWITCH_Z },
    };

    for (auto& s : sw) {
        if (s.pin < 0) {
            result(s.name, R::SKIP, "unassigned");
            continue;
        }
        pinMode(s.pin, INPUT_PULLUP);
        delayMicroseconds(50);
        bool tripped = (digitalRead(s.pin) == LOW);
        result(s.name,
               tripped ? R::WARN : R::PASS,
               tripped ? "TRIPPED (LOW) — is gantry on this switch?"
                       : "OPEN (HIGH) — nominal");
    }
}

// ============================================================
//  Stage 4 — TSL1401 line sensor ADC
// ============================================================
static void stageLineSensor() {
    Serial.println(F("\n[4] TSL1401 line sensor"));

    if (Pins::LaserSensor::AO < 0) {
        result(F("Sensor AO"), R::SKIP, "pin unassigned");
        return;
    }

    pinMode(Pins::LaserSensor::AO, INPUT);
    // Discard first read (ADC channel switching artifact on Teensy 4.x)
    analogRead(Pins::LaserSensor::AO);
    delayMicroseconds(100);
    int raw = analogRead(Pins::LaserSensor::AO);

    char buf[32];
    MKBUF(buf, "ADC = %d / 1023", raw);

    if (raw < 10) {
        result(F("Sensor AO"), R::WARN, "near 0 — possible short to GND or no VCC");
    } else if (raw > 1013) {
        result(F("Sensor AO"), R::WARN, "near max — possible short to 3.3V or SI/CLK floating");
    } else {
        result(F("Sensor AO"), R::PASS, buf);
    }

    if (Pins::LaserSensor::SI < 0 || Pins::LaserSensor::CLK < 0) {
        result(F("Sensor SI/CLK"), R::SKIP, "one or both control pins unassigned");
    }
}

// ============================================================
//  Stage 5 — Encoder static stability
// ============================================================
static void stageEncoderStatic(ClosedLoopStepper* mA, ClosedLoopStepper* mB) {
    Serial.println(F("\n[5] Encoder static stability  (500 ms)"));

    if (!mA || !mB) {
        result(F("Encoders"), R::SKIP, "motors not constructed (gantry pins unassigned)");
        return;
    }

    int32_t a0 = mA->getEncoderCount();
    int32_t b0 = mB->getEncoderCount();
    delay(500);
    int32_t dA = abs(mA->getEncoderCount() - a0);
    int32_t dB = abs(mB->getEncoderCount() - b0);

    char bufA[40], bufB[40];
    MKBUF(bufA, "drift = %ld ticks / 500 ms", (long)dA);
    MKBUF(bufB, "drift = %ld ticks / 500 ms", (long)dB);

    result(F("Encoder A static"), dA <= ENC_MAX_STATIC_DRIFT ? R::PASS : R::WARN, bufA);
    result(F("Encoder B static"), dB <= ENC_MAX_STATIC_DRIFT ? R::PASS : R::WARN, bufB);

    if (dA > ENC_MAX_STATIC_DRIFT || dB > ENC_MAX_STATIC_DRIFT) {
        Serial.println(F("    High drift: check ENC_A/B wiring for noise pickup,"));
        Serial.println(F("    or gently hold gantry still to rule out vibration."));
    }
}

// ============================================================
//  Stage 6 — Gantry motion + encoder response  (interactive)
// ============================================================
static void stageGantryMotion(ClosedLoopStepper* mA, ClosedLoopStepper* mB) {
    Serial.println(F("\n[6] Gantry motion + encoder response  (interactive)"));

    if (!mA || !mB) {
        result(F("Gantry motion"), R::SKIP, "motors not constructed");
        return;
    }

    Serial.println(F("    Pulses both motors forward ~2 mm then back."));
    Serial.println(F("    Keep gantry at least 5 mm from limit switches."));
    Serial.print(F("    Press Y to run, any other key to skip: "));
    Serial.flush();

    char c = waitKey(20000);
    if (c != 'y' && c != 'Y') {
        result(F("Gantry motion"), R::SKIP, "user skipped");
        return;
    }

    // Derived encoder counts per mm
    float encPerMM = static_cast<float>(CHECK_ENCODER_CPR)
                   / (Constants::Gantry::BELT_PITCH_MM
                      * Constants::Gantry::PULLEY_TEETH);
    float jogMM    = static_cast<float>(JOG_STEPS) / Constants::Gantry::STEPS_PER_MM;

    // Zero before test
    mA->zeroPosition();
    mB->zeroPosition();

    // ---- Jog forward ----
    mA->setDirection(true);
    mB->setDirection(true);
    delayMicroseconds(10);
    for (int32_t i = 0; i < JOG_STEPS; i++) {
        mA->step();
        mB->step();
        delayMicroseconds(JOG_PULSE_US);
    }
    delay(100);  // settle before reading

    int32_t encA_fwd = mA->getEncoderCount();
    int32_t encB_fwd = mB->getEncoderCount();

    // ---- Jog back ----
    mA->setDirection(false);
    mB->setDirection(false);
    delayMicroseconds(10);
    for (int32_t i = 0; i < JOG_STEPS; i++) {
        mA->step();
        mB->step();
        delayMicroseconds(JOG_PULSE_US);
    }
    delay(100);

    int32_t encA_ret = mA->getEncoderCount();
    int32_t encB_ret = mB->getEncoderCount();

    // Reset step/encoder counters
    mA->zeroPosition();
    mB->zeroPosition();

    // ---- Evaluate ----
    float minFwdTicks = ENC_MIN_MOVE_MM  * encPerMM;
    float maxRetTicks = ENC_MAX_RETURN_ERR_MM * encPerMM;

    // Check motor A
    {
        bool moved    = (abs(encA_fwd) >= static_cast<int32_t>(minFwdTicks));
        bool returned = (abs(encA_ret) <= static_cast<int32_t>(maxRetTicks));
        bool reversed = moved && (encA_fwd < 0);  // encoder went wrong way

        char buf[64];
        MKBUF(buf, "fwd=%ld  ret=%ld  exp_fwd~%.0f ticks",
              (long)encA_fwd, (long)encA_ret, jogMM * encPerMM);

        if (!moved) {
            result(F("MotorA + Encoder A"), R::FAIL,
                   "encoder did not move — check motor wiring and encoder wiring");
        } else if (reversed) {
            result(F("MotorA + Encoder A"), R::WARN,
                   "encoder moved BACKWARD — swap ENC_A/ENC_B wires or set invertDir");
        } else if (!returned) {
            result(F("MotorA + Encoder A"), R::WARN,
                   "did not return to origin (following error) — check belt tension");
        } else {
            result(F("MotorA + Encoder A"), R::PASS, buf);
        }
    }

    // Check motor B
    {
        bool moved    = (abs(encB_fwd) >= static_cast<int32_t>(minFwdTicks));
        bool returned = (abs(encB_ret) <= static_cast<int32_t>(maxRetTicks));
        bool reversed = moved && (encB_fwd < 0);

        char buf[64];
        MKBUF(buf, "fwd=%ld  ret=%ld  exp_fwd~%.0f ticks",
              (long)encB_fwd, (long)encB_ret, jogMM * encPerMM);

        if (!moved) {
            result(F("MotorB + Encoder B"), R::FAIL,
                   "encoder did not move — check motor wiring and encoder wiring");
        } else if (reversed) {
            result(F("MotorB + Encoder B"), R::WARN,
                   "encoder moved BACKWARD — swap ENC_A/ENC_B wires or set invertDir");
        } else if (!returned) {
            result(F("MotorB + Encoder B"), R::WARN,
                   "did not return to origin (following error) — check belt tension");
        } else {
            result(F("MotorB + Encoder B"), R::PASS, buf);
        }
    }
}

// ============================================================
//  Stage 7 — MG996R servo sweep  (interactive)
// ============================================================
static void stageMG996R() {
    Serial.println(F("\n[7] MG996R PWM servos"));

    const int pins[4] = {
        Pins::Servos::MG996R_1, Pins::Servos::MG996R_2,
        Pins::Servos::MG996R_3, Pins::Servos::MG996R_4,
    };
    bool anyAssigned = false;
    for (int p : pins) { if (p >= 0) { anyAssigned = true; break; } }

    if (!anyAssigned) {
        result(F("MG996R (all)"), R::SKIP, "all pins unassigned");
        return;
    }

    Serial.println(F("    Moves each assigned servo to 45 deg then back to 90 deg."));
    Serial.println(F("    Listen for servo motion (no position feedback available)."));
    Serial.print(F("    Press Y to run, any other key to skip: "));
    Serial.flush();

    char c = waitKey(15000);
    if (c != 'y' && c != 'Y') {
        result(F("MG996R"), R::SKIP, "user skipped");
        return;
    }

    ServoController& sc = ServoController::instance();
    const __FlashStringHelper* labels[4] = {
        F("MG996R_1"), F("MG996R_2"), F("MG996R_3"), F("MG996R_4")
    };

    for (uint8_t i = 0; i < 4; i++) {
        if (pins[i] < 0) {
            result(labels[i], R::SKIP, "unassigned");
            continue;
        }
        sc.mg996rSetAngle(i, 45.0f);
        delay(600);
        sc.mg996rSetAngle(i, 90.0f);
        delay(600);
        // No feedback — WARN is the best we can do without a potentiometer or encoder
        char buf[16]; MKBUF(buf, "pin %d", pins[i]);
        result(labels[i], R::WARN,
               "commanded OK — verify motion by sight/sound (no feedback)");
    }
}

// ============================================================
//  Summary
// ============================================================
static void printSummary() {
    Serial.println(F("\n=========================================="));
    if (g_fails == 0) {
        Serial.println(F("  RESULT:  ALL CHECKS PASSED / SKIPPED"));
        Serial.println(F("  System appears ready for powered tests."));
    } else {
        char buf[48];
        MKBUF(buf, "%u FAIL(s) detected — resolve before proceeding.", g_fails);
        Serial.println(F("  RESULT:  FAILURES DETECTED"));
        Serial.println(buf);
    }
    Serial.println(F("=========================================="));
    Serial.flush();
}

// ============================================================
//  Motor instances
//  Constructed inside testHwCheck_setup() so Encoder library's
//  attachInterrupt() runs after Arduino framework init.
// ============================================================
static ClosedLoopStepper* chkMotorA = nullptr;
static ClosedLoopStepper* chkMotorB = nullptr;

// ============================================================
//  Entry points
// ============================================================
void testHwCheck_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println(F("\n=========================================="));
    Serial.println(F("  CoreXY Pre-Flight Hardware Check"));
    Serial.println(F("=========================================="));
    Serial.println(F("  PASS = verified OK"));
    Serial.println(F("  FAIL = confirmed fault — fix before proceeding"));
    Serial.println(F("  WARN = potential issue or no feedback available"));
    Serial.println(F("  SKIP = hardware not assigned in PinMap.h"));
    Serial.flush();

    // Stage 1: pin audit — also determines if gantry can be constructed
    bool gantryPinsOK = stagePinAudit();

    // Construct motors now (after Arduino init) if pins are assigned
    if (gantryPinsOK) {
        static ClosedLoopStepper _mA(
            Pins::Gantry::MOTOR_A_STEP, Pins::Gantry::MOTOR_A_DIR,
            Pins::Gantry::MOTOR_A_ENC_A, Pins::Gantry::MOTOR_A_ENC_B,
            Constants::Gantry::STEPS_PER_MM, /*stateIndex=*/0, CHECK_ENCODER_CPR);
        static ClosedLoopStepper _mB(
            Pins::Gantry::MOTOR_B_STEP, Pins::Gantry::MOTOR_B_DIR,
            Pins::Gantry::MOTOR_B_ENC_A, Pins::Gantry::MOTOR_B_ENC_B,
            Constants::Gantry::STEPS_PER_MM, /*stateIndex=*/1, CHECK_ENCODER_CPR);
        chkMotorA = &_mA;
        chkMotorB = &_mB;
        chkMotorA->begin();
        chkMotorB->begin();
    }

    // Remaining stages
    stageSTS3215();
    stageSwitches();
    stageLineSensor();
    stageEncoderStatic(chkMotorA, chkMotorB);
    stageGantryMotion(chkMotorA, chkMotorB);
    stageMG996R();

    printSummary();
}

void testHwCheck_loop() {
    // All checks complete — idle heartbeat so the logger stays connected.
    static uint32_t lastMs = 0;
    if (millis() - lastMs >= 5000) {
        lastMs = millis();
        Serial.println(F("# idle — hardware check finished. Safe to re-flash."));
        Serial.flush();
    }
}

#endif // TEST_MODE
