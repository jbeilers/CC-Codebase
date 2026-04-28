// ============================================================
//  testing/test_mg996r.cpp
//
//  PURPOSE
//  -------
//  Bring-up test for up to four MG996R standard PWM servos.
//
//  SETUP — fill in PinMap.h first
//  --------------------------------
//  In include/pins/PinMap.h, under namespace Servos, set:
//
//    MG996R_1  — PWM-capable Teensy pin for servo 1
//    MG996R_2  — PWM-capable Teensy pin for servo 2
//    MG996R_3  — PWM-capable Teensy pin for servo 3
//    MG996R_4  — PWM-capable Teensy pin for servo 4
//
//  Leave a pin as -1 to skip that servo entirely.
//  All PWM-capable Teensy 4.1 pins work (any pin labelled ~).
//
//  MG996R power:
//    VCC — 5 V (servo rail; keep separate from logic if possible)
//    GND — common ground with Teensy GND
//    Signal — Teensy PWM pin (3.3 V logic is accepted by MG996R)
//
//  PULSE WIDTH
//  -----------
//  Default: 500 µs = 0°,  2500 µs = 180°.
//  If your batch uses a different range (e.g. 1000–2000 µs),
//  update MIN_PULSE_US and MAX_PULSE_US below.
//
//  COMMANDS (single keypress, no newline needed)
//  ---------------------------------------------
//    1–4    — Select active servo (default: 1)
//    A / D  — Sweep to min angle (0°) / max angle (180°)
//    C      — Centre active servo (90°)
//    W / S  — Step angle +5° / -5°
//    T      — Auto-test: sweep all wired servos 0→90→180→90→0
//    ?      — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testMG996R_setup();
//    extern void testMG996R_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include <Servo.h>
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  Pulse width limits — edit if your batch differs
// ============================================================
static constexpr int MIN_PULSE_US = static_cast<int>(Constants::Servos::MG996R_MIN_PULSE_US);
static constexpr int MAX_PULSE_US = static_cast<int>(Constants::Servos::MG996R_MAX_PULSE_US);

// ============================================================
//  Angle limits and step size
// ============================================================
static constexpr float ANGLE_MIN  = Constants::Servos::MG996R_MIN_ANGLE_DEG;  // 0°
static constexpr float ANGLE_MAX  = Constants::Servos::MG996R_MAX_ANGLE_DEG;  // 180°
static constexpr float ANGLE_STEP = 5.0f;

// ============================================================
//  Servo instances (Arduino Servo library)
// ============================================================
static Servo  g_servo[4];
static float  g_angle[4]   = {90.0f, 90.0f, 90.0f, 90.0f};
static bool   g_attached[4] = {false, false, false, false};

// Currently selected servo (0-indexed internally, 1-indexed for user)
static uint8_t g_sel = 0;

// ============================================================
//  Pin lookup — maps index to PinMap constant
// ============================================================
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
//  Number of servos that are actually wired
// ============================================================
static uint8_t wiredCount() {
    uint8_t n = 0;
    for (uint8_t i = 0; i < 4; i++) if (g_attached[i]) n++;
    return n;
}

// ============================================================
//  Move helper — clamps, stores, and writes
// ============================================================
static void setAngle(uint8_t idx, float deg) {
    if (!g_attached[idx]) {
        Serial.print(F("# Servo ")); Serial.print(idx + 1);
        Serial.println(F(" not attached — set pin in PinMap.h"));
        return;
    }
    if (deg < ANGLE_MIN) deg = ANGLE_MIN;
    if (deg > ANGLE_MAX) deg = ANGLE_MAX;
    g_angle[idx] = deg;
    g_servo[idx].write(static_cast<int>(deg));
    Serial.print(F("# Servo ")); Serial.print(idx + 1);
    Serial.print(F(" → ")); Serial.print(deg, 1); Serial.println(F(" deg"));
}

// ============================================================
//  Help / status
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# MG996R Servo Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  1–4  — Select active servo"));
    Serial.println(F("#  A    — Move to 0°  (min)"));
    Serial.println(F("#  D    — Move to 180° (max)"));
    Serial.println(F("#  C    — Centre (90°)"));
    Serial.println(F("#  W/S  — Step +5° / -5°"));
    Serial.println(F("#  T    — Auto-test (sweep all wired servos)"));
    Serial.println(F("#  ?    — This help"));
    Serial.println(F("# ---------------------------------------------"));

    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("# Servo ")); Serial.print(i + 1);
        Serial.print(F("  pin="));   Serial.print(servoPin(i));
        if (g_attached[i]) {
            Serial.print(F("  angle=")); Serial.print(g_angle[i], 1); Serial.print(F(" deg"));
        } else {
            Serial.print(F("  (not wired — pin is -1)"));
        }
        if (i == g_sel) Serial.print(F("  ← active"));
        Serial.println();
    }

    Serial.print(F("# Pulse: ")); Serial.print(MIN_PULSE_US);
    Serial.print(F(" – ")); Serial.print(MAX_PULSE_US); Serial.println(F(" µs"));

    if (wiredCount() == 0) {
        Serial.println(F("# WARNING: No servos wired."));
        Serial.println(F("#   Set MG996R_x pins in PinMap.h (Pins::Servos namespace)."));
    }
    Serial.println(F("# ============================================="));
}

// ============================================================
//  Auto-test: sweep every attached servo 0→90→180→90→0
// ============================================================
static void runAutoTest() {
    if (wiredCount() == 0) {
        Serial.println(F("# SKIP: no servos wired (all pins are -1)."));
        return;
    }

    Serial.println(F("# === Auto-test start ==="));

    // Test each attached servo individually
    for (uint8_t i = 0; i < 4; i++) {
        if (!g_attached[i]) continue;

        Serial.print(F("# Servo ")); Serial.print(i + 1); Serial.println(F(": 0°"));
        setAngle(i, 0.0f);
        delay(800);

        Serial.print(F("# Servo ")); Serial.print(i + 1); Serial.println(F(": 90°"));
        setAngle(i, 90.0f);
        delay(800);

        Serial.print(F("# Servo ")); Serial.print(i + 1); Serial.println(F(": 180°"));
        setAngle(i, 180.0f);
        delay(800);

        Serial.print(F("# Servo ")); Serial.print(i + 1); Serial.println(F(": 90°"));
        setAngle(i, 90.0f);
        delay(800);

        Serial.print(F("# Servo ")); Serial.print(i + 1); Serial.println(F(": 0°"));
        setAngle(i, 0.0f);
        delay(500);
    }

    // If more than one is wired, sweep all simultaneously
    if (wiredCount() > 1) {
        Serial.println(F("# All servos: sweep together 0→180→0"));
        for (uint8_t i = 0; i < 4; i++) if (g_attached[i]) setAngle(i, 0.0f);
        delay(800);
        for (uint8_t i = 0; i < 4; i++) if (g_attached[i]) setAngle(i, 90.0f);
        delay(800);
        for (uint8_t i = 0; i < 4; i++) if (g_attached[i]) setAngle(i, 180.0f);
        delay(800);
        for (uint8_t i = 0; i < 4; i++) if (g_attached[i]) setAngle(i, 90.0f);
        delay(500);
    }

    Serial.println(F("# === Auto-test complete ==="));
    Serial.println(F("#"));
    Serial.println(F("# Servo didn't move?  Check 5V supply, GND to Teensy,"));
    Serial.println(F("#   signal wiring, and PinMap.h MG996R_x pin values."));
    Serial.println(F("# Servo jitters?  Add a 100 µF cap across servo VCC/GND."));
    Serial.println(F("# Wrong range?    Tune MIN_PULSE_US/MAX_PULSE_US at top of file."));
}

// ============================================================
//  Entry points
// ============================================================
void testMG996R_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Attach any servo whose pin is assigned
    for (uint8_t i = 0; i < 4; i++) {
        int pin = servoPin(i);
        if (pin < 0) continue;
        g_servo[i].attach(pin, MIN_PULSE_US, MAX_PULSE_US);
        g_attached[i] = true;
        // Drive to centre on startup
        g_servo[i].write(static_cast<int>(g_angle[i]));
    }

    // Select first attached servo by default
    for (uint8_t i = 0; i < 4; i++) {
        if (g_attached[i]) { g_sel = i; break; }
    }

    printHelp();
}

void testMG996R_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {

        // ---- Servo selection ----
        case '1': case '2': case '3': case '4': {
            uint8_t idx = static_cast<uint8_t>(c - '1');
            if (!g_attached[idx]) {
                Serial.print(F("# Servo ")); Serial.print(idx + 1);
                Serial.println(F(" not wired — set pin in PinMap.h"));
            } else {
                g_sel = idx;
                Serial.print(F("# Active: Servo ")); Serial.print(g_sel + 1);
                Serial.print(F(" @ ")); Serial.print(g_angle[g_sel], 1);
                Serial.println(F(" deg"));
            }
            break;
        }

        // ---- Angle commands ----
        case 'a': case 'A':
            setAngle(g_sel, ANGLE_MIN);
            break;
        case 'd': case 'D':
            setAngle(g_sel, ANGLE_MAX);
            break;
        case 'c': case 'C':
            setAngle(g_sel, 90.0f);
            break;
        case 'w': case 'W':
            setAngle(g_sel, g_angle[g_sel] + ANGLE_STEP);
            break;
        case 's': case 'S':
            setAngle(g_sel, g_angle[g_sel] - ANGLE_STEP);
            break;

        // ---- Auto-test / help ----
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
