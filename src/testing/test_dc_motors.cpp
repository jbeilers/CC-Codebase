// ============================================================
//  testing/test_dc_motors.cpp
//
//  PURPOSE
//  -------
//  Bring-up test for the two DC motors (lead screws) wired
//  through the L298N dual H-bridge module.
//
//  SETUP — fill in PinMap.h first
//  --------------------------------
//  In include/pins/PinMap.h, under namespace DCMotors, set:
//
//    MOTOR_A_IN1  — IN1 on the L298N  → Teensy digital pin
//    MOTOR_A_IN2  — IN2 on the L298N  → Teensy digital pin
//    MOTOR_A_ENA  — ENA on the L298N  → Teensy PWM pin
//                   (remove the ENA jumper first; set -1 to keep
//                    the jumper in and run at full speed)
//
//    MOTOR_B_IN3  — IN3 on the L298N  → Teensy digital pin
//    MOTOR_B_IN4  — IN4 on the L298N  → Teensy digital pin
//    MOTOR_B_ENB  — ENB on the L298N  → Teensy PWM pin
//                   (same note as ENA above)
//
//  L298N power:
//    12V terminal — motor supply (match your motor's voltage)
//    GND          — common ground with Teensy GND
//    5V terminal  — can power Teensy if supply ≤ 12V (optional)
//
//  COMMANDS (single keypress, no newline needed)
//  ---------------------------------------------
//    W / S  — Motor A (lead screw 1): forward / reverse
//    E      — Motor A: coast (free stop)
//    I / K  — Motor B (lead screw 2): forward / reverse
//    O      — Motor B: coast (free stop)
//    B      — Brake BOTH (fast stop via back-EMF)
//    X      — Coast BOTH
//    + / -  — Increase / decrease speed (PWM only)
//    T      — Auto-test sequence (both motors, all states)
//    ?      — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testDCMotors_setup();
//    extern void testDCMotors_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"

// ============================================================
//  Speed (0–255).  Ignored if ENA/ENB pin is -1 (jumper in).
// ============================================================
static uint8_t g_speed = 180;
static constexpr uint8_t SPEED_STEP = 25;

// ============================================================
//  Low-level helpers
// ============================================================
static void motorSet(int in1, int in2, int en, bool forward, uint8_t spd) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, forward ? HIGH : LOW);
    digitalWrite(in2, forward ? LOW  : HIGH);
    if (en >= 0) analogWrite(en, spd);
}

static void motorBrake(int in1, int in2, int en) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    if (en >= 0) analogWrite(en, 255);
}

static void motorCoast(int in1, int in2, int en) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (en >= 0) analogWrite(en, 0);
}

// ============================================================
//  Named wrappers — pull pins from PinMap::DCMotors
// ============================================================
static void motorA_forward() { motorSet  (Pins::DCMotors::MOTOR_A_IN1, Pins::DCMotors::MOTOR_A_IN2, Pins::DCMotors::MOTOR_A_ENA, true,  g_speed); }
static void motorA_reverse() { motorSet  (Pins::DCMotors::MOTOR_A_IN1, Pins::DCMotors::MOTOR_A_IN2, Pins::DCMotors::MOTOR_A_ENA, false, g_speed); }
static void motorA_brake()   { motorBrake(Pins::DCMotors::MOTOR_A_IN1, Pins::DCMotors::MOTOR_A_IN2, Pins::DCMotors::MOTOR_A_ENA); }
static void motorA_coast()   { motorCoast(Pins::DCMotors::MOTOR_A_IN1, Pins::DCMotors::MOTOR_A_IN2, Pins::DCMotors::MOTOR_A_ENA); }

static void motorB_forward() { motorSet  (Pins::DCMotors::MOTOR_B_IN3, Pins::DCMotors::MOTOR_B_IN4, Pins::DCMotors::MOTOR_B_ENB, true,  g_speed); }
static void motorB_reverse() { motorSet  (Pins::DCMotors::MOTOR_B_IN3, Pins::DCMotors::MOTOR_B_IN4, Pins::DCMotors::MOTOR_B_ENB, false, g_speed); }
static void motorB_brake()   { motorBrake(Pins::DCMotors::MOTOR_B_IN3, Pins::DCMotors::MOTOR_B_IN4, Pins::DCMotors::MOTOR_B_ENB); }
static void motorB_coast()   { motorCoast(Pins::DCMotors::MOTOR_B_IN3, Pins::DCMotors::MOTOR_B_IN4, Pins::DCMotors::MOTOR_B_ENB); }

// ============================================================
//  Guard — true only when the minimum required pins are set
// ============================================================
static bool pinsAssigned() {
    return (Pins::DCMotors::MOTOR_A_IN1 >= 0 && Pins::DCMotors::MOTOR_A_IN2 >= 0 &&
            Pins::DCMotors::MOTOR_B_IN3 >= 0 && Pins::DCMotors::MOTOR_B_IN4 >= 0);
}

// ============================================================
//  Help / status
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# DC Lead-Screw Motor Test  (L298N H-bridge)"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  W / S  — Motor A: forward / reverse"));
    Serial.println(F("#  E      — Motor A: coast (free stop)"));
    Serial.println(F("#  I / K  — Motor B: forward / reverse"));
    Serial.println(F("#  O      — Motor B: coast (free stop)"));
    Serial.println(F("#  B      — Brake BOTH (back-EMF)"));
    Serial.println(F("#  X      — Coast BOTH"));
    Serial.println(F("#  + / -  — Speed up / slow down (PWM)"));
    Serial.println(F("#  T      — Auto-test sequence"));
    Serial.println(F("#  ?      — This help"));
    Serial.println(F("# ---------------------------------------------"));

    // Pin summary
    Serial.print(F("# Motor A  IN1=")); Serial.print(Pins::DCMotors::MOTOR_A_IN1);
    Serial.print(F("  IN2="));          Serial.print(Pins::DCMotors::MOTOR_A_IN2);
    Serial.print(F("  ENA="));          Serial.println(Pins::DCMotors::MOTOR_A_ENA);
    Serial.print(F("# Motor B  IN3=")); Serial.print(Pins::DCMotors::MOTOR_B_IN3);
    Serial.print(F("  IN4="));          Serial.print(Pins::DCMotors::MOTOR_B_IN4);
    Serial.print(F("  ENB="));          Serial.println(Pins::DCMotors::MOTOR_B_ENB);

    Serial.print(F("# Speed:   ")); Serial.print(g_speed);
    Serial.print(F(" / 255  (")); Serial.print(g_speed * 100 / 255);
    Serial.println(F("%)"));

    if (!pinsAssigned()) {
        Serial.println(F("# WARNING: IN pins still -1 in PinMap.h."));
        Serial.println(F("#          Set DCMotors::MOTOR_x_IN pins before running."));
    }
    Serial.println(F("# ============================================="));
}

// ============================================================
//  Auto-test sequence
// ============================================================
static void runAutoTest() {
    if (!pinsAssigned()) {
        Serial.println(F("# SKIP: DCMotors IN pins not assigned in PinMap.h."));
        return;
    }

    Serial.println(F("# === Auto-test start ==="));

    Serial.println(F("# [A] Forward 1.5 s"));
    motorA_forward();
    delay(1500);
    Serial.println(F("# [A] Coast 0.5 s"));
    motorA_coast();
    delay(500);
    Serial.println(F("# [A] Reverse 1.5 s"));
    motorA_reverse();
    delay(1500);
    Serial.println(F("# [A] Brake"));
    motorA_brake();
    delay(500);

    Serial.println(F("# [B] Forward 1.5 s"));
    motorB_forward();
    delay(1500);
    Serial.println(F("# [B] Coast 0.5 s"));
    motorB_coast();
    delay(500);
    Serial.println(F("# [B] Reverse 1.5 s"));
    motorB_reverse();
    delay(1500);
    Serial.println(F("# [B] Brake"));
    motorB_brake();
    delay(500);

    Serial.println(F("# [A+B] Both forward 1.5 s"));
    motorA_forward();
    motorB_forward();
    delay(1500);
    Serial.println(F("# [A+B] Both reverse 1.5 s"));
    motorA_reverse();
    motorB_reverse();
    delay(1500);
    Serial.println(F("# [A+B] Brake both"));
    motorA_brake();
    motorB_brake();
    delay(300);
    motorA_coast();
    motorB_coast();

    Serial.println(F("# === Auto-test complete ==="));
    Serial.println(F("#"));
    Serial.println(F("# Motor didn't move?  Check: 12V supply, GND to Teensy,"));
    Serial.println(F("#   IN1-IN4 wiring, and PinMap.h DCMotors pin values."));
    Serial.println(F("# Motor runs backwards?  Swap its two wires on L298N output."));
    Serial.println(F("# Speed control has no effect?  Remove ENA/ENB jumper and"));
    Serial.println(F("#   set MOTOR_x_ENA/ENB pin in PinMap.h DCMotors namespace."));
}

// ============================================================
//  Entry points
// ============================================================
void testDCMotors_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    if (Pins::DCMotors::MOTOR_A_IN1 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN1, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW); }
    if (Pins::DCMotors::MOTOR_A_IN2 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN2, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW); }
    if (Pins::DCMotors::MOTOR_A_ENA >= 0) { pinMode(Pins::DCMotors::MOTOR_A_ENA, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0); }

    if (Pins::DCMotors::MOTOR_B_IN3 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN3, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW); }
    if (Pins::DCMotors::MOTOR_B_IN4 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN4, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW); }
    if (Pins::DCMotors::MOTOR_B_ENB >= 0) { pinMode(Pins::DCMotors::MOTOR_B_ENB, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0); }

    printHelp();
}

void testDCMotors_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {
        case 'w': case 'W':
            if (!pinsAssigned()) { Serial.println(F("# Pins not set in PinMap.h")); break; }
            motorA_forward();
            Serial.print(F("# Motor A FORWARD @ ")); Serial.println(g_speed);
            break;
        case 's': case 'S':
            if (!pinsAssigned()) { Serial.println(F("# Pins not set in PinMap.h")); break; }
            motorA_reverse();
            Serial.print(F("# Motor A REVERSE @ ")); Serial.println(g_speed);
            break;
        case 'e': case 'E':
            motorA_coast();
            Serial.println(F("# Motor A COAST"));
            break;

        case 'i': case 'I':
            if (!pinsAssigned()) { Serial.println(F("# Pins not set in PinMap.h")); break; }
            motorB_forward();
            Serial.print(F("# Motor B FORWARD @ ")); Serial.println(g_speed);
            break;
        case 'k': case 'K':
            if (!pinsAssigned()) { Serial.println(F("# Pins not set in PinMap.h")); break; }
            motorB_reverse();
            Serial.print(F("# Motor B REVERSE @ ")); Serial.println(g_speed);
            break;
        case 'o': case 'O':
            motorB_coast();
            Serial.println(F("# Motor B COAST"));
            break;

        case 'b': case 'B':
            motorA_brake(); motorB_brake();
            Serial.println(F("# BRAKE both"));
            break;
        case 'x': case 'X':
            motorA_coast(); motorB_coast();
            Serial.println(F("# COAST both"));
            break;

        case '+': case '=':
            if (g_speed <= 255 - SPEED_STEP) g_speed += SPEED_STEP; else g_speed = 255;
            Serial.print(F("# Speed ")); Serial.print(g_speed); Serial.print(F("/255  (")); Serial.print(g_speed * 100 / 255); Serial.println(F("%)"));
            break;
        case '-': case '_':
            if (g_speed >= SPEED_STEP) g_speed -= SPEED_STEP; else g_speed = 0;
            Serial.print(F("# Speed ")); Serial.print(g_speed); Serial.print(F("/255  (")); Serial.print(g_speed * 100 / 255); Serial.println(F("%)"));
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
