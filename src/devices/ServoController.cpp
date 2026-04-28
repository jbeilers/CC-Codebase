#include "devices/ServoController.h"

// ============================================================
//  ServoController.cpp
// ============================================================

// MG996R pin assignments from PinMap
const int ServoController::MG996R_PINS[4] = {
    Pins::Servos::MG996R_1,
    Pins::Servos::MG996R_2,
    Pins::Servos::MG996R_3,
    Pins::Servos::MG996R_4
};

// ----------------------------------------------------------
void ServoController::begin() {
    // Initialise Serial7 for single-wire half-duplex (pin 29 only).
    Serial7.begin(Constants::Servos::STS3215_BAUD_RATE);

    // Two-pin half-duplex wiring: TX (pin 29) drives DATA through a 1kΩ
    // resistor; RX (pin 28) connects directly to DATA.
    // The Teensy hears its own TX as an echo on RX — stsSendPacket()
    // discards that echo before reading the servo's response.
    // No LOOPS mode needed — normal full-duplex UART with shared bus.

    // Attach MG996R servos to their PWM pins
    for (uint8_t i = 0; i < 4; i++) {
        if (MG996R_PINS[i] < 0) continue;  // Skip unassigned pins (TODO)
        mg996r_[i].attach(
            MG996R_PINS[i],
            static_cast<int>(Constants::Servos::MG996R_MIN_PULSE_US),
            static_cast<int>(Constants::Servos::MG996R_MAX_PULSE_US)
        );
    }
}

// ==========================================================
//  STS3215 — Public interface
// ==========================================================

void ServoController::sts3215EnableTorque(bool enable) {
    uint8_t params[2] = { STS::REG_TORQUE_ENABLE, enable ? uint8_t(1) : uint8_t(0) };
    stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                  STS::INSTR_WRITE,
                  params, 2);
    // Read and discard the write status response so it doesn't pollute later reads
    uint8_t buf[6] = {0};
    stsReadResponse(buf, 6);
}

int8_t ServoController::sts3215ReadTorqueEnable() {
    uint8_t params[2] = { STS::REG_TORQUE_ENABLE, 1 };
    stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                  STS::INSTR_READ,
                  params, 2);
    uint8_t buf[7] = {0};   // FF FF ID LEN ERR DATA CSUM
    if (!stsReadResponse(buf, 7)) return -1;
    if (buf[0] != STS::HEADER || buf[1] != STS::HEADER) return -1;
    return static_cast<int8_t>(buf[5]);
}

void ServoController::sts3215SetPosition(float angleDeg, uint16_t speed) {
    // Wrap to [0, 360) so callers may pass negative offsets
    // (e.g. -45° → 315°, matching the physical reverse direction).
    angleDeg = fmodf(angleDeg, 360.0f);
    if (angleDeg < 0.0f) angleDeg += 360.0f;

    uint16_t pos = static_cast<uint16_t>(angleDeg * STS::COUNTS_PER_DEG);

    // 1. Clear Goal Time (0x2C) to 0 so the servo uses speed-based control.
    //    If Goal Time is non-zero the servo ignores Goal Speed and uses a
    //    time-based profile instead, which causes inconsistent motion.
    uint8_t timeParams[3] = { STS::REG_GOAL_TIME, 0x00, 0x00 };
    stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                  STS::INSTR_WRITE, timeParams, 3);
    { uint8_t buf[6] = {0}; stsReadResponse(buf, 6); }

    // 2. Write Goal Speed (0x2E) before position so the profile is set before
    //    motion starts. Writing Goal Position (step 3) triggers movement immediately.
    // Always write — even speed=0 (STS protocol: 0 = max speed) so previous
    // slow-speed test commands in SRAM don't carry over to the next move.
    {
        uint8_t spdParams[3] = {
            STS::REG_GOAL_SPEED,
            static_cast<uint8_t>(speed & 0xFF),
            static_cast<uint8_t>(speed >> 8)
        };
        stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                      STS::INSTR_WRITE, spdParams, 3);
        uint8_t buf[6] = {0};
        stsReadResponse(buf, 6);
    }

    // 3. Write Goal Position — triggers movement immediately.
    //    Retry up to 3 times; a corrupted response or bus glitch can silently
    //    drop the command, leaving the servo stationary with no error reported.
    uint8_t posParams[3] = {
        STS::REG_GOAL_POS,
        static_cast<uint8_t>(pos & 0xFF),
        static_cast<uint8_t>(pos >> 8)
    };
    bool posOk = false;
    for (uint8_t attempt = 0; attempt < 3 && !posOk; attempt++) {
        if (attempt > 0) {
            delayMicroseconds(500);   // brief pause before retry
            Serial.print(F("STS POS retry "));
            Serial.println(attempt);
        }
        stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                      STS::INSTR_WRITE, posParams, 3);
        uint8_t buf[6] = {0};
        if (stsReadResponse(buf, 6)
                && buf[0] == STS::HEADER
                && buf[1] == STS::HEADER
                && buf[4] == 0) {
            posOk = true;
        }
    }
    if (!posOk) {
        Serial.println(F("STS POS failed after 3 attempts"));
    }

    // Update MachineState immediately with commanded angle
    ServoState s = MachineState::instance().getSTS3215();
    s.angleDeg  = angleDeg;
    s.isMoving  = true;
    MachineState::instance().setSTS3215(s);
}

float ServoController::sts3215GetPosition() {
    // Send read request: register REG_PRESENT_POS, 2 bytes
    uint8_t params[2] = { STS::REG_PRESENT_POS, 2 };
    stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                  STS::INSTR_READ,
                  params, 2);

    // Response packet: 0xFF 0xFF ID LEN ERR posL posH CHECKSUM (8 bytes)
    uint8_t buf[8] = {0};
    if (!stsReadResponse(buf, 8)) return -1.0f;

    // Validate header and ID
    if (buf[0] != STS::HEADER ||
        buf[1] != STS::HEADER ||
        buf[2] != Constants::Servos::STS3215_SERVO_ID) {
        return -1.0f;
    }

    uint16_t rawPos = static_cast<uint16_t>(buf[5])
                    | (static_cast<uint16_t>(buf[6]) << 8);

    return static_cast<float>(rawPos) / STS::COUNTS_PER_DEG;
}

bool ServoController::sts3215Ping() {
    stsSendPacket(Constants::Servos::STS3215_SERVO_ID,
                  STS::INSTR_PING,
                  nullptr, 0);

    uint8_t buf[6] = {0};
    return stsReadResponse(buf, 6);
}

void ServoController::sts3215SyncState() {
    float pos = sts3215GetPosition();
    if (pos < 0.0f) return;  // Read failed — don't update state

    ServoState s = MachineState::instance().getSTS3215();
    float prev   = s.angleDeg;
    s.angleDeg   = pos;
    s.isMoving   = (fabsf(pos - prev) > 0.5f);  // TODO: Tune movement threshold in degrees
    MachineState::instance().setSTS3215(s);
}

// ==========================================================
//  MG996R — Public interface
// ==========================================================

void ServoController::mg996rSetAngle(uint8_t index, float angleDeg) {
    if (index >= 4) return;

    // Clamp to configured range
    if (angleDeg < Constants::Servos::MG996R_MIN_ANGLE_DEG)
        angleDeg = Constants::Servos::MG996R_MIN_ANGLE_DEG;
    if (angleDeg > Constants::Servos::MG996R_MAX_ANGLE_DEG)
        angleDeg = Constants::Servos::MG996R_MAX_ANGLE_DEG;

    mg996rAngle_[index] = angleDeg;
    mg996r_[index].write(static_cast<int>(angleDeg));
}

float ServoController::mg996rGetAngle(uint8_t index) const {
    if (index >= 4) return 0.0f;
    return mg996rAngle_[index];
}

void ServoController::mg996rSyncState() {
    for (uint8_t i = 0; i < 4; i++) {
        ServoState s;
        s.angleDeg = mg996rAngle_[i];
        s.isMoving = false;  // MG996R has no feedback; assume settled
        MachineState::instance().setMG996R(i, s);
    }
}

// ==========================================================
//  STS3215 — Low-level protocol helpers
// ==========================================================

void ServoController::stsSendPacket(uint8_t        id,
                                    uint8_t        instr,
                                    const uint8_t* params,
                                    uint8_t        paramLen) {
    uint8_t len  = paramLen + 2;  // LEN = num params + instr byte + checksum byte
    uint8_t csum = stsChecksum(id, len, instr, params, paramLen);

    // Flush any stale RX bytes before sending
    while (Serial7.available()) Serial7.read();

    Serial7.write(STS::HEADER);
    Serial7.write(STS::HEADER);
    Serial7.write(id);
    Serial7.write(len);
    Serial7.write(instr);
    for (uint8_t i = 0; i < paramLen; i++) Serial7.write(params[i]);
    Serial7.write(csum);

    // Wait for all bytes to physically transmit at 1 Mbaud.
    // Serial7.flush() is intentionally avoided — the LPUART TC flag on
    // Teensy 4.1 does not reliably set and causes flush() to hang.
    // Instead, compute the exact on-wire time: (6 fixed + paramLen) bytes,
    // each 10 bits @ 1 Mbaud = 10 µs/byte, plus 100 µs margin.
    uint32_t txUs = static_cast<uint32_t>((6 + paramLen) * 10) + 100;
    delayMicroseconds(txUs);

    // On a half-duplex single-wire bus the Teensy RX pin sees its own TX bytes.
    // Discard that echo (6 fixed bytes + paramLen) before reading the response.
    const uint8_t echoLen = 6 + paramLen;
    uint32_t t0 = millis();
    for (uint8_t discarded = 0; discarded < echoLen; ) {
        if (millis() - t0 > 5) break;   // 5 ms safety timeout
        if (Serial7.available()) { Serial7.read(); discarded++; }
    }
}

uint8_t ServoController::stsChecksum(uint8_t        id,
                                     uint8_t        len,
                                     uint8_t        instr,
                                     const uint8_t* params,
                                     uint8_t        paramLen) {
    uint8_t sum = id + len + instr;
    for (uint8_t i = 0; i < paramLen; i++) sum += params[i];
    return ~sum & 0xFF;
}

bool ServoController::stsReadResponse(uint8_t* buf, uint8_t expectedLen) {
    uint32_t start = millis();
    uint8_t  idx   = 0;

    while (idx < expectedLen) {
        if (millis() - start > STS::READ_TIMEOUT_MS) return false;
        if (Serial7.available() > 0) {
            buf[idx++] = static_cast<uint8_t>(Serial7.read());
        }
    }
    return true;
}
