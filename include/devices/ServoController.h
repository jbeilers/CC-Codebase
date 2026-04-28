#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <cstdint>
#include "state/MachineState.h"
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  ServoController.h  —  Unified servo interface.
//
//  Manages two servo types:
//
//  1. STS3215 Smart Servo  (1x, TTL UART half-duplex on Serial7)
//     Uses the Feetech STS packet protocol:
//       0xFF 0xFF | ID | LEN | INSTR | PARAMS... | CHECKSUM
//     Checksum = ~(ID + LEN + INSTR + PARAMS...) & 0xFF
//     Position range: 0–4095 (maps to 0°–360°)
//     TODO: Confirm full rotation range for your use case —
//           STS3215 can be configured as a wheel or position servo.
//
//  2. MG996R Standard Servos  (4x, PWM 50Hz via Arduino Servo lib)
//     Position range: MG996R_MIN_ANGLE_DEG – MG996R_MAX_ANGLE_DEG
//     Each instance is indexed 0–3.
//
//  Half-duplex wiring note:
//    TX and RX share one data line. The Teensy drives TX;
//    during a read, it must listen on RX after sending the
//    request. Serial7 on Teensy 4.1 handles this natively
//    with careful timing — no direction pin required if the
//    TX line is sufficiently isolated.
//    TODO: If signal contention is observed, add a direction
//          pin (STS3215_DIR in PinMap) and drive it before TX.
// ============================================================

// ----------------------------------------------------------
//  Feetech STS protocol constants
// ----------------------------------------------------------
namespace STS {
    constexpr uint8_t HEADER        = 0xFF;
    constexpr uint8_t INSTR_WRITE   = 0x03;
    constexpr uint8_t INSTR_READ    = 0x02;
    constexpr uint8_t INSTR_PING    = 0x01;

    // Register addresses for STS3215
    constexpr uint8_t REG_TORQUE_ENABLE  = 0x28;  // Torque enable  (1 byte: 0=off, 1=on)
    constexpr uint8_t REG_GOAL_POS       = 0x2A;  // Goal position  (2 bytes, LSB first) — triggers movement on write
    constexpr uint8_t REG_GOAL_TIME      = 0x2C;  // Goal time      (2 bytes) — 0 = disabled, use speed instead
    constexpr uint8_t REG_GOAL_SPEED     = 0x2E;  // Goal speed     (2 bytes) — 0 = max speed
    constexpr uint8_t REG_PRESENT_POS    = 0x38;  // Present position (2 bytes, read-only)
    constexpr uint8_t REG_TORQUE_LIMIT   = 0x23;  // Torque limit   (2 bytes)

    // Position encoding: 0–4095 maps to 0°–360°
    constexpr float COUNTS_PER_DEG = 4095.0f / 360.0f;

    // Read response timeout in milliseconds.
    // 5 ms was too short when the servo is under load or the bus is marginal.
    constexpr uint32_t READ_TIMEOUT_MS = 20;
}

class ServoController {
public:
    static ServoController& instance() {
        static ServoController inst;
        return inst;
    }

    // Disallow copy
    ServoController(const ServoController&)            = delete;
    ServoController& operator=(const ServoController&) = delete;

    // ----------------------------------------------------------
    //  Call once in setup() to initialise Serial7 and attach
    //  all MG996R PWM pins.
    // ----------------------------------------------------------
    void begin();

    // ==========================================================
    //  STS3215 Interface
    // ==========================================================

    // ----------------------------------------------------------
    //  Enable or disable STS3215 torque output.
    //  Must be called with enable=true before the servo will move.
    //  Called automatically in begin().
    // ----------------------------------------------------------
    void sts3215EnableTorque(bool enable = true);

    // ----------------------------------------------------------
    //  Read back the torque enable register (0x28).
    //  Returns 1 if torque is on, 0 if off, -1 on read failure.
    // ----------------------------------------------------------
    int8_t sts3215ReadTorqueEnable();

    // ----------------------------------------------------------
    //  Command STS3215 to move to angleDeg at given speed.
    //  @param angleDeg  Target angle in degrees (0–360)
    //  @param speed     Travel speed (0–4095; 0 = max speed)
    //                   TODO: Map speed to physical units once
    //                         tested on hardware.
    // ----------------------------------------------------------
    void sts3215SetPosition(float angleDeg, uint16_t speed = 0);

    // ----------------------------------------------------------
    //  Read current STS3215 position in degrees.
    //  Returns -1.0f on read failure.
    // ----------------------------------------------------------
    float sts3215GetPosition();

    // ----------------------------------------------------------
    //  Ping the STS3215 — returns true if it responds.
    //  Useful for connection health checks.
    // ----------------------------------------------------------
    bool sts3215Ping();

    // ----------------------------------------------------------
    //  Sync STS3215 state to MachineState.
    //  Call periodically to keep state up to date.
    // ----------------------------------------------------------
    void sts3215SyncState();

    // ==========================================================
    //  MG996R Interface  (index 0–3)
    // ==========================================================

    // ----------------------------------------------------------
    //  Command MG996R at index to move to angleDeg.
    //  @param index     0–3
    //  @param angleDeg  Target angle in degrees
    // ----------------------------------------------------------
    void mg996rSetAngle(uint8_t index, float angleDeg);

    // ----------------------------------------------------------
    //  Read last commanded angle for MG996R at index.
    //  (MG996R has no feedback — returns last set value.)
    // ----------------------------------------------------------
    [[nodiscard]] float mg996rGetAngle(uint8_t index) const;

    // ----------------------------------------------------------
    //  Sync all MG996R states to MachineState.
    // ----------------------------------------------------------
    void mg996rSyncState();

private:
    ServoController() = default;

    // ----------------------------------------------------------
    //  STS3215 low-level protocol helpers
    // ----------------------------------------------------------
    void     stsSendPacket(uint8_t id, uint8_t instr,
                           const uint8_t* params, uint8_t paramLen);
    uint8_t  stsChecksum(uint8_t id, uint8_t len,
                         uint8_t instr,
                         const uint8_t* params, uint8_t paramLen);
    bool     stsReadResponse(uint8_t* buf, uint8_t expectedLen);

    // ----------------------------------------------------------
    //  MG996R instances (Arduino Servo library)
    // ----------------------------------------------------------
    Servo    mg996r_[4];
    float    mg996rAngle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // MG996R pin assignments (set in begin() from PinMap)
    static const int MG996R_PINS[4];
};
