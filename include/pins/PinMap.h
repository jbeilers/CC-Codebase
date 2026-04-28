#pragma once

// ============================================================
//  PinMap.h  —  Single source of truth for all pin assignments
//  Target: Teensy 4.1
//
//  RULES:
//    - Only constexpr int declarations here. No logic, ever.
//    - Unassigned pins are set to -1. Do NOT use them until wired.
//    - Group by device/subsystem, not by pin number.
//    - When you assign a real pin, remove the -1 and update the comment.
//
//  PIN ALLOCATION OVERVIEW  (Teensy 4.1)
//  ───────────────────────────────────────────────────────────
//  Left edge  (pins 0-13):
//    0-1   RESERVED — Serial1 RX/TX (keep free for future use)
//    2     LaserSensor SI
//    3     LaserSensor CLK
//    4-7   Gantry STEP/DIR (A then B)
//    8     LineLaser ENABLE
//    9     SPARE  (MG996R_3 moved to pin 36 — FlexPWM4_2_B shared submodule conflict)
//    10-12 Limit switches X/Y/Z
//    13    RESERVED — onboard LED
//
//  Left edge  (pins 14-23):
//    14    LaserSensor AO (A0 analog in)
//    15-17 Device 1 Screw 1 STEP, Screw 1 DIR, Screw 2 STEP
//    18-19 RESERVED — I2C SDA/SCL (Wire)
//    20-21 Device 1 Screw 2 DIR, Device 2 Screw 1 STEP
//    22    MG996R_4  (QuadTimer3_1)
//    23    MG996R_1  (FlexPWM2_2_B)
//
//  Right edge (pins 24-41):
//    24-26 Device 2 Screw 1 DIR, Screw 2 STEP, Screw 2 DIR
//    27    MG996R_2  (FlexPWM4_2_A)
//    28    Serial7 RX — wire to STS3215 DATA line (same node as pin 29 via 1kΩ)
//    29    STS3215 TX (Serial7)
//    30-33 SPARE
//    34-35 DC Motor A IN1/IN2
//    36    MG996R_3  (FlexPWM2_3_B — moved from pin 9 to avoid FlexPWM4_2 conflict)
//    37    SPARE (ENA jumper left on L298N)
//    38-39 DC Motor B IN3/IN4
//    40    SPARE (ENB jumper left on L298N)
//    41    SPARE
//
//  Bottom pads (42-54): SDIO / SD card / spare — not assigned
// ============================================================

namespace Pins {

    // ----------------------------------------------------------
    //  CoreXY Gantry — Steppers
    //  Driver: TB6600 (STEP + DIR only, always enabled)
    //  Motor A drives belt anchored at left  (X+Y contribution)
    //  Motor B drives belt anchored at right (X-Y contribution)
    // ----------------------------------------------------------
    namespace Gantry {
        // Motor A
        constexpr int MOTOR_A_STEP  = 4;
        constexpr int MOTOR_A_DIR   = 5;
        constexpr int MOTOR_A_ENC_A = -1;   // Not used — encoder removed
        constexpr int MOTOR_A_ENC_B = -1;

        // Motor B
        constexpr int MOTOR_B_STEP  = 6;
        constexpr int MOTOR_B_DIR   = 7;
        constexpr int MOTOR_B_ENC_A = -1;   // Not used — encoder removed
        constexpr int MOTOR_B_ENC_B = -1;
    }

    // ----------------------------------------------------------
    //  Lead Screw Steppers — NEMA 17, TB6600 (STEP + DIR only)
    //  Device 1 screws: pins 15-17, 20  (left edge, below AO)
    //  Device 2 screws: pins 21, 24-26  (wraps to right edge)
    //  Grouped to simplify wiring runs to each TB6600.
    //  STEP/DIR have no special hardware requirements.
    // ----------------------------------------------------------
    namespace LeadScrews {
        constexpr int DEVICE1_SCREW1_STEP = 15;
        constexpr int DEVICE1_SCREW1_DIR  = 16;

        constexpr int DEVICE1_SCREW2_STEP = 17;
        constexpr int DEVICE1_SCREW2_DIR  = 20;   // 18/19 reserved for I2C

        constexpr int DEVICE2_SCREW1_STEP = 21;
        constexpr int DEVICE2_SCREW1_DIR  = 24;

        constexpr int DEVICE2_SCREW2_STEP = 25;
        constexpr int DEVICE2_SCREW2_DIR  = 26;
    }

    // ----------------------------------------------------------
    //  Servos
    //
    //  STS3215 — TTL UART, two-wire half-duplex
    //    Pin 29 (TX) drives the servo DATA wire through a 1 kΩ resistor.
    //    Pin 28 (RX) connects directly to the DATA wire (same node as the
    //    resistor output). In normal UART mode RX sees the TX echo, which
    //    ServoController::stsSendPacket() discards before reading the servo
    //    response. No direction pin or buffer IC needed.
    //
    //  MG996R — Standard PWM (50 Hz)
    //    All Teensy 4.1 pins support PWM via FlexPWM / QuadTimer.
    //    Pin 9 sits beside the laser/camera block (8).
    //    Pins 22/27 are on the right edge; pins 9/23 on the left.
    //    Pin 27 completes the servo group on the right edge.
    // ----------------------------------------------------------
    namespace Servos {
        // STS3215 smart servo (two-wire half-duplex, Serial7 TX + RX)
        constexpr int STS3215_TX  = 29;   // Serial7 TX — data via 1 kΩ resistor
        constexpr int STS3215_RX  = 28;   // Serial7 RX — direct to DATA node
        constexpr int STS3215_DIR = -1;   // Not used — no direction pin needed

        // MG996R standard PWM servos
        constexpr int MG996R_1 = 23;    // FlexPWM2_2_B
        constexpr int MG996R_2 = 27;   // FlexPWM4_2_A
        constexpr int MG996R_3 = 36;   // FlexPWM2_3_B — moved from pin 9 (avoided FlexPWM4_2_B / shared-submodule conflict with pin 27)
        constexpr int MG996R_4 = 22;   // QuadTimer3_1
    }

    // ----------------------------------------------------------
    //  DC Motors — L298N Dual H-Bridge
    //  IN1/IN2 control Motor A direction/stop
    //  IN3/IN4 control Motor B direction/stop
    //  ENA/ENB jumpers left on — motors always enabled at full speed.
    //  Speed control is not used; direction and coast/brake via IN pins only.
    // ----------------------------------------------------------
    namespace DCMotors {
        constexpr int MOTOR_A_IN1 = 34;
        constexpr int MOTOR_A_IN2 = 35;
        constexpr int MOTOR_A_ENA = -1;   // Jumper left on — always enabled at 5V

        constexpr int MOTOR_B_IN3 = 38;
        constexpr int MOTOR_B_IN4 = 39;
        constexpr int MOTOR_B_ENB = -1;   // Jumper left on — always enabled at 5V
    }

    // ----------------------------------------------------------
    //  TSL1401 Line Sensor Array Camera
    //  SI  — Serial Input (start-of-scan pulse, digital out)
    //  CLK — Clock       (digital out)
    //  AO  — Analog Out  (pixel voltage, analog in)
    //
    //  SI and CLK on pins 2/3 — first free digital pins after
    //  Serial1 (0/1) reserved block; easy to find and wire.
    //  AO on pin 14 (A0) — first analog-capable pin.
    // ----------------------------------------------------------
    namespace LaserSensor {
        constexpr int SI  = 2;    // Digital out
        constexpr int CLK = 3;    // Digital out
        constexpr int AO  = 14;   // Analog in (A0)
    }

    // ----------------------------------------------------------
    //  5V Line Laser Module
    //  Controlled via PN2222 NPN transistor on pin 8:
    //    Teensy pin 8 → 1 kΩ → Base
    //    Collector → Laser GND wire
    //    Emitter  → System GND
    //    Laser VCC wire → 5 V supply
    //  HIGH = laser on, LOW = laser off.
    // ----------------------------------------------------------
    namespace LineLaser {
        constexpr int ENABLE = 8;   // Digital out → NPN base (1 kΩ series)
    }

    // ----------------------------------------------------------
    //  Mechanical Microswitches  (limit / homing switches)
    //  Wired normally-open: HIGH = not tripped, LOW = tripped.
    //  All use INPUT_PULLUP.
    //
    //  Placed on pins 10/11/12 (SPI bus pins) — no SPI device
    //  exists in this design so they are available as GPIO.
    //  This keeps Serial1 (0/1) and I2C (18/19) free.
    // ----------------------------------------------------------
    namespace Switches {
        constexpr int SWITCH_X = 10;
        constexpr int SWITCH_Y = 11;
        constexpr int SWITCH_Z = 12;
    }

    // ----------------------------------------------------------
    //  Serial Communication — HP Thin Client
    //  Teensy 4.1 communicates over native USB (Serial object).
    //  No physical pins are consumed — USB is handled by the MCU.
    //  Baud rate is defined in platformio.ini as SERIAL_BAUD.
    // ----------------------------------------------------------
    // (no pins required for USB serial)

} // namespace Pins
