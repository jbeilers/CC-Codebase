// ============================================================
//  testing/test_demo.cpp  —  Full-system demonstration test
//
//  PURPOSE
//  -------
//  Single file covering all 8 subsystems for a live demo.
//  Switch modes with ] / [ or number keys 1–8.
//  Each mode is self-contained — no device classes, no
//  MachineState, no TeensyThreads.  Direct GPIO everywhere.
//
//  MODES
//  -----
//    1  D1 Lead Screws    (D1-S1, D1-S2 independent, 15 mm)
//    2  Servos 1 & 2      (MG996R linear actuator, independent)
//    3  DC Motors         (conveyor, timed & continuous)
//    4  Gantry            (open-loop WASD, hold key to run)
//    5  Camera            (TSL1401 + line laser)
//    6  Servos 3 & 4      (MG996R syringe/tweezers, independent)
//    7  STS3215           (z-axis smart servo, multi-turn)
//    8  D2 Lead Screws    (D2-S1, D2-S2 independent, 15 mm)
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testDemo_setup();
//    extern void testDemo_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include <Servo.h>
#include "pins/PinMap.h"

// ============================================================
//  CONFIGURATION  —  edit before first run
// ============================================================

// ---- Lead Screws (applies to both D1 and D2) ---------------
// Match MS_SCREW to your TB6600 DIP switch microstepping setting.
static constexpr int   DEMO_LS_MS        = 1;       // microstepping (1, 2, 4, 8, 16, 32)
static constexpr float DEMO_LS_DIST_MM   = 15.0f;   // default travel per U/D press (mm)
static constexpr float DEMO_LS_SPEED_MMS = 15.0f;    // travel speed (mm/s)
static constexpr float DEMO_LS_LEAD      = 8.0f;    // T8 lead: 8 mm/rev
static constexpr int   DEMO_LS_STEPS_REV = 200;     // full steps per revolution
static constexpr uint32_t DEMO_LS_PULSE_US = 5;     // STEP pulse high time (µs)
// DIR pin level that means "platform rises / moves up".
// Flip to LOW if a screw moves the wrong way when U is pressed.
static constexpr int DEMO_LS_DIR_UP = LOW;

// ---- MG996R Servos -----------------------------------------
static constexpr int   DEMO_MG_MIN_US   = 500;     // pulse width at 0°
static constexpr int   DEMO_MG_MAX_US   = 2500;    // pulse width at 180°
static constexpr float DEMO_MG_STEP_DEG = 10.0f;   // degrees per W/S keypress

// Target angles for servos 1 & 2 (linear actuator)
// Press 'A' to go to ANGLE_A, 'D' to go to ANGLE_B, 'C' to centre.
static constexpr float DEMO_MG12_S1_A = 10.0f;    // servo 1 position A
static constexpr float DEMO_MG12_S1_B = 180.0f;   // servo 1 position B
static constexpr float DEMO_MG12_S2_A = 0.0f;    // servo 2 position A
static constexpr float DEMO_MG12_S2_B = 180.0f;   // servo 2 position B

// Target angles for servos 3 & 4 (syringe / tweezers)
static constexpr float DEMO_MG34_S3_A = 0.0f;
static constexpr float DEMO_MG34_S3_B = 90.0f;
static constexpr float DEMO_MG34_S4_A = 0.0f;
static constexpr float DEMO_MG34_S4_B = 10.0f;

// ---- DC Motors (L298N H-bridge) ----------------------------
static constexpr uint8_t  DEMO_DC_SPEED_DEF  = 40;   // 0–255 PWM
static constexpr uint8_t  DEMO_DC_SPEED_STEP = 25;
static constexpr uint32_t DEMO_DC_TIMED_MS   = 2000;  // T key timed-run duration

// ---- Gantry (open-loop, no encoder) -----------------------
// DIR pin level that drives each motor in the "forward" direction.
// If a W/A/S/D jog goes the wrong way, flip the relevant constant.
static constexpr int DEMO_G_DIR_A_FWD = LOW;    // Motor A "forward" DIR level
static constexpr int DEMO_G_DIR_B_FWD = LOW;    // Motor B "forward" DIR level

// Inter-step delay controls travel speed.  Lower = faster.
// Press F / G inside gantry mode to halve / double on the fly.
static constexpr uint32_t DEMO_G_STEP_US_DEF  = 800;   // µs between steps
static constexpr uint32_t DEMO_G_STEP_US_MIN  = 100;
static constexpr uint32_t DEMO_G_STEP_US_MAX  = 5000;
static constexpr uint32_t DEMO_G_PULSE_US     = 5;

// ---- Camera (TSL1401 + line laser) -------------------------
static constexpr uint32_t DEMO_CAM_BURST_S      = 5;    // B key burst duration (seconds)
static constexpr uint32_t DEMO_CAM_CLK_HALF_US  = 10;
static constexpr uint32_t DEMO_CAM_CLK_STEP_US = 5;
static constexpr uint32_t DEMO_CAM_CLK_MIN_US  = 5;
static constexpr uint32_t DEMO_CAM_CLK_MAX_US  = 100;
static constexpr uint8_t  DEMO_CAM_PIXELS      = 128;
static constexpr uint8_t  DEMO_CAM_EDGE_SKIP   = 5;
static constexpr uint8_t  DEMO_CAM_EDGE_CNF    = 3;
static constexpr uint8_t  DEMO_CAM_JUMP_LIMIT  = 70;
static constexpr uint16_t DEMO_CAM_MIN_CONTRAST = 10;

// ---- STS3215 Smart Servo -----------------------------------
// g_stsStepDeg is adjustable at runtime.  Keep < 180° for correct
// multi-turn direction (the servo picks the shortest ≤180° path).
static constexpr float   DEMO_STS_STEP_DEG_DEF  = 10.0f;  // default step per U/D press
static constexpr float   DEMO_STS_STEP_STEP     = 5.0f;   // +/- key adjustment
static constexpr float   DEMO_STS_STEP_MIN      = 1.0f;
static constexpr float   DEMO_STS_STEP_MAX      = 170.0f; // safety: keep < 180°
static constexpr uint16_t DEMO_STS_SPEED_DEF    = 300;    // 1–4095; 0 = max speed
static constexpr uint32_t DEMO_STS_BAUD         = 1000000;
static constexpr uint8_t  DEMO_STS_ID           = 1;
// Feetech STS protocol constants
static constexpr uint8_t STS_HDR             = 0xFF;
static constexpr uint8_t STS_INSTR_PING      = 0x01;
static constexpr uint8_t STS_INSTR_READ      = 0x02;
static constexpr uint8_t STS_INSTR_WRITE     = 0x03;
static constexpr uint8_t STS_REG_GOALPOS     = 0x2A;  // goal position (2 bytes, LSB first)
static constexpr uint8_t STS_REG_PRESENT_POS = 0x38;  // present position (2 bytes, read-only)
static constexpr float   STS_CPD             = 4095.0f / 360.0f;  // counts / degree
static constexpr uint32_t STS_READ_TIMEOUT_MS = 5;    // ms to wait for servo response

// ============================================================
//  MODE ENUM
// ============================================================
enum class DemoMode : uint8_t {
    D1_SCREWS = 1,
    SERVO_12  = 2,
    DC_MOTORS = 3,
    GANTRY    = 4,
    CAMERA    = 5,
    SERVO_34  = 6,
    STS       = 7,
    D2_SCREWS = 8
};

static DemoMode g_mode = DemoMode::D1_SCREWS;

static const char* demoModeName(DemoMode m) {
    switch (m) {
        case DemoMode::D1_SCREWS: return "1: D1 Lead Screws";
        case DemoMode::SERVO_12:  return "2: Servos 1 & 2 (linear actuator)";
        case DemoMode::DC_MOTORS: return "3: DC Motors";
        case DemoMode::GANTRY:    return "4: Gantry (open-loop)";
        case DemoMode::CAMERA:    return "5: Camera (TSL1401)";
        case DemoMode::SERVO_34:  return "6: Servos 3 & 4 (syringe/tweezers)";
        case DemoMode::STS:       return "7: STS3215 (z-axis)";
        case DemoMode::D2_SCREWS: return "8: D2 Lead Screws";
        default:                  return "?";
    }
}

// ============================================================
//  COMMON LEAD-SCREW HELPERS
// ============================================================
static constexpr float    LS_SPM =
    static_cast<float>(DEMO_LS_STEPS_REV * DEMO_LS_MS) / DEMO_LS_LEAD;

static constexpr uint32_t LS_STEP_DELAY_US =
    static_cast<uint32_t>(1000000.0f / (DEMO_LS_SPEED_MMS * LS_SPM));

struct ScrewInfo { int stepPin, dirPin; const char* name; };

static const ScrewInfo D1_SCREWS[2] = {
    { Pins::LeadScrews::DEVICE1_SCREW1_STEP, Pins::LeadScrews::DEVICE1_SCREW1_DIR, "D1-S1" },
    { Pins::LeadScrews::DEVICE1_SCREW2_STEP, Pins::LeadScrews::DEVICE1_SCREW2_DIR, "D1-S2" },
};
static const ScrewInfo D2_SCREWS[2] = {
    { Pins::LeadScrews::DEVICE2_SCREW1_STEP, Pins::LeadScrews::DEVICE2_SCREW1_DIR, "D2-S1" },
    { Pins::LeadScrews::DEVICE2_SCREW2_STEP, Pins::LeadScrews::DEVICE2_SCREW2_DIR, "D2-S2" },
};

static float   g_lsDist = DEMO_LS_DIST_MM;
static uint8_t g_lsSel  = 0;   // 0 = first screw in pair, 1 = second

static void lsMove(const ScrewInfo& sc, float distMm, bool up) {
    if (sc.stepPin < 0 || sc.dirPin < 0) {
        Serial.print(F("# ")); Serial.print(sc.name);
        Serial.println(F(": pin not set in PinMap.h — skipped"));
        return;
    }
    int32_t steps = static_cast<int32_t>(distMm * LS_SPM + 0.5f);
    bool dirLevel = up ? (DEMO_LS_DIR_UP == HIGH) : (DEMO_LS_DIR_UP != HIGH);
    digitalWrite(sc.dirPin, dirLevel ? HIGH : LOW);
    delayMicroseconds(2);

    Serial.print(F("# ")); Serial.print(sc.name);
    Serial.print(up ? F(" UP ") : F(" DOWN "));
    Serial.print(distMm, 1); Serial.println(F(" mm"));

    for (int32_t s = 0; s < steps; s++) {
        digitalWrite(sc.stepPin, HIGH);
        delayMicroseconds(DEMO_LS_PULSE_US);
        digitalWrite(sc.stepPin, LOW);
        if (LS_STEP_DELAY_US > DEMO_LS_PULSE_US)
            delayMicroseconds(LS_STEP_DELAY_US - DEMO_LS_PULSE_US);
    }
    Serial.println(F("# Done."));
}

static void lsMoveSelected(const ScrewInfo screws[], bool up) {
    lsMove(screws[g_lsSel], g_lsDist, up);
}

// ============================================================
//  MG996R SERVO HELPERS
// ============================================================
static Servo  g_servo[4];
static float  g_servoAngle[4]    = {90.0f, 90.0f, 90.0f, 90.0f};
static bool   g_servoAttached[4] = {false, false, false, false};
static uint8_t g_servoSel = 0;   // 0-indexed within the current pair

static int servoPinOf(uint8_t idx) {
    switch (idx) {
        case 0: return Pins::Servos::MG996R_1;
        case 1: return Pins::Servos::MG996R_2;
        case 2: return Pins::Servos::MG996R_3;
        case 3: return Pins::Servos::MG996R_4;
        default: return -1;
    }
}

// First index in each mode's servo pair (0 for servos 1&2, 2 for servos 3&4)
static uint8_t g_servoBase = 0;  // set when entering mode 2 or 6

static void servoSet(uint8_t idx, float deg) {
    if (!g_servoAttached[idx]) {
        Serial.print(F("# Servo ")); Serial.print(idx + 1);
        Serial.println(F(": not attached — check PinMap.h"));
        return;
    }
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    g_servoAngle[idx] = deg;
    g_servo[idx].write(static_cast<int>(deg));
    Serial.print(F("# Servo ")); Serial.print(idx + 1);
    Serial.print(F(" → ")); Serial.print(deg, 1); Serial.println(F("°"));
}

// ============================================================
//  DC MOTOR HELPERS  (L298N)
// ============================================================
static uint8_t g_dcSpeed = DEMO_DC_SPEED_DEF;

static void dcSet(int in1, int in2, int en, bool fwd, uint8_t spd) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, fwd ? HIGH : LOW);
    digitalWrite(in2, fwd ? LOW  : HIGH);
    if (en >= 0) analogWrite(en, spd);
}
static void dcCoast(int in1, int in2, int en) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    if (en >= 0) analogWrite(en, 0);
}
static void dcBrake(int in1, int in2, int en) {
    if (in1 < 0 || in2 < 0) return;
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
    if (en >= 0) analogWrite(en, 255);
}

#define DC_A_ARGS Pins::DCMotors::MOTOR_A_IN1, Pins::DCMotors::MOTOR_A_IN2, Pins::DCMotors::MOTOR_A_ENA
#define DC_B_ARGS Pins::DCMotors::MOTOR_B_IN3, Pins::DCMotors::MOTOR_B_IN4, Pins::DCMotors::MOTOR_B_ENB

// ============================================================
//  GANTRY (OPEN-LOOP) HELPERS
// ============================================================
enum class GDir : uint8_t { NONE, POS_X, NEG_X, POS_Y, NEG_Y };
static GDir     g_gDir        = GDir::NONE;
static uint32_t g_gStepUs     = DEMO_G_STEP_US_DEF;
static uint32_t g_gLastStepUs = 0;

static void gantryStep() {
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, HIGH);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, HIGH);
    delayMicroseconds(DEMO_G_PULSE_US);
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);
}

// Called every loop — fires one step if direction is active and timer elapsed
static void gantryTick() {
    if (g_gDir == GDir::NONE) return;
    uint32_t now = micros();
    if (now - g_gLastStepUs < g_gStepUs) return;
    g_gLastStepUs = now;

    const int A_BWD = (DEMO_G_DIR_A_FWD == LOW) ? HIGH : LOW;
    const int B_BWD = (DEMO_G_DIR_B_FWD == LOW) ? HIGH : LOW;

    switch (g_gDir) {
        case GDir::POS_Y:  // +Y: A fwd, B bwd
            digitalWrite(Pins::Gantry::MOTOR_A_DIR, DEMO_G_DIR_A_FWD);
            digitalWrite(Pins::Gantry::MOTOR_B_DIR, B_BWD);
            delayMicroseconds(2);
            gantryStep();
            break;
        case GDir::NEG_Y:  // -Y: A bwd, B fwd
            digitalWrite(Pins::Gantry::MOTOR_A_DIR, A_BWD);
            digitalWrite(Pins::Gantry::MOTOR_B_DIR, DEMO_G_DIR_B_FWD);
            delayMicroseconds(2);
            gantryStep();
            break;
        case GDir::POS_X:  // +X: A fwd, B fwd
            digitalWrite(Pins::Gantry::MOTOR_A_DIR, DEMO_G_DIR_A_FWD);
            digitalWrite(Pins::Gantry::MOTOR_B_DIR, DEMO_G_DIR_B_FWD);
            delayMicroseconds(2);
            gantryStep();
            break;
        case GDir::NEG_X:  // -X: A bwd, B bwd
            digitalWrite(Pins::Gantry::MOTOR_A_DIR, A_BWD);
            digitalWrite(Pins::Gantry::MOTOR_B_DIR, B_BWD);
            delayMicroseconds(2);
            gantryStep();
            break;
        default: break;
    }
}

// ============================================================
//  CAMERA HELPERS  (TSL1401 + line laser)
// ============================================================
static uint32_t g_camClkHalfUs = DEMO_CAM_CLK_HALF_US;
static uint16_t g_camPixels[DEMO_CAM_PIXELS] = {0};
static uint8_t  g_camLastCenter = 64;
static uint8_t  g_camLeftEdge   = 0;
static uint8_t  g_camRightEdge  = DEMO_CAM_PIXELS - 1;
static uint16_t g_camThreshold  = 0;
static bool     g_camDetected   = false;
static bool     g_camContinuous = false;
static bool     g_camStreaming  = false;
static bool     g_camLaserOn    = false;
static bool     g_camBurst      = false;   // burst mode active
static uint32_t g_camBurstEndMs = 0;       // millis() when burst should stop
static uint32_t g_camLastMs     = 0;

static bool camPinsOk() {
    return (Pins::LaserSensor::SI >= 0 &&
            Pins::LaserSensor::CLK >= 0 &&
            Pins::LaserSensor::AO >= 0);
}

static void camLaserOn() {
    if (Pins::LineLaser::ENABLE < 0) return;
    digitalWrite(Pins::LineLaser::ENABLE, HIGH);
    g_camLaserOn = true;
}
static void camLaserOff() {
    if (Pins::LineLaser::ENABLE < 0) return;
    digitalWrite(Pins::LineLaser::ENABLE, LOW);
    g_camLaserOn = false;
}

static void camCapture() {
    if (!camPinsOk()) return;
    camLaserOn();
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(g_camClkHalfUs);
    digitalWrite(Pins::LaserSensor::SI,  HIGH);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(g_camClkHalfUs);
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(g_camClkHalfUs);
    for (uint8_t i = 0; i < DEMO_CAM_PIXELS; i++) {
        digitalWrite(Pins::LaserSensor::CLK, LOW);
        delayMicroseconds(g_camClkHalfUs);
        g_camPixels[i] = static_cast<uint16_t>(analogRead(Pins::LaserSensor::AO));
        digitalWrite(Pins::LaserSensor::CLK, HIGH);
        delayMicroseconds(g_camClkHalfUs);
    }
    camLaserOff();
}

static uint8_t camFindCenter() {
    const uint8_t skip = DEMO_CAM_EDGE_SKIP;
    const uint8_t cnf  = DEMO_CAM_EDGE_CNF;
    uint16_t maxV = g_camPixels[skip] >> 2, minV = maxV;
    for (uint8_t i = skip; i < (DEMO_CAM_PIXELS - skip); i++) {
        uint16_t v = g_camPixels[i] >> 2;
        if (v > maxV) maxV = v;
        if (v < minV) minV = v;
    }
    g_camThreshold = (maxV + minV) / 2;
    if ((maxV - minV) < DEMO_CAM_MIN_CONTRAST) { g_camDetected = false; return g_camLastCenter; }

    uint8_t left = 0; bool lFound = false;
    for (uint8_t i = skip; i < (DEMO_CAM_PIXELS - skip - cnf * 2); i++) {
        bool above = true, below = true;
        for (uint8_t k = 0; k < cnf; k++) {
            if ((g_camPixels[i + k]       >> 2) <= g_camThreshold) above = false;
            if ((g_camPixels[i + cnf + k] >> 2) >  g_camThreshold) below = false;
        }
        if (above && below) { left = i; lFound = true; break; }
    }
    uint8_t right = DEMO_CAM_PIXELS - 1; bool rFound = false;
    for (int16_t j = DEMO_CAM_PIXELS - skip - 1; j >= skip + cnf * 2; j--) {
        bool below = true, above = true;
        for (uint8_t k = 0; k < cnf; k++) {
            if ((g_camPixels[j - k]           >> 2) >= g_camThreshold) below = false;
            if ((g_camPixels[j + cnf - k]     >> 2) <  g_camThreshold) above = false;
        }
        if (below && above) { right = static_cast<uint8_t>(j); rFound = true; break; }
    }
    if (!lFound || !rFound || right <= left) { g_camDetected = false; return g_camLastCenter; }
    uint8_t nc = (left + right) / 2;
    uint8_t delta = (nc > g_camLastCenter) ? (nc - g_camLastCenter) : (g_camLastCenter - nc);
    if (delta > DEMO_CAM_JUMP_LIMIT) { g_camDetected = false; return g_camLastCenter; }
    g_camLastCenter = nc;
    g_camLeftEdge   = left;
    g_camRightEdge  = right;
    g_camDetected   = true;
    return nc;
}

static void camPrintCsv() {
    for (uint8_t i = 0; i < DEMO_CAM_PIXELS; i++) {
        Serial.print(i); if (i < DEMO_CAM_PIXELS - 1) Serial.print(',');
    }
    Serial.println();
    for (uint8_t i = 0; i < DEMO_CAM_PIXELS; i++) {
        Serial.print(g_camPixels[i]); if (i < DEMO_CAM_PIXELS - 1) Serial.print(',');
    }
    Serial.println();
}

static void camPrintStreamFrame() {
    Serial.print(F("F:")); Serial.print(millis()); Serial.print(',');
    Serial.print(g_camClkHalfUs);   Serial.print(',');
    Serial.print(g_camLastCenter);  Serial.print(',');
    Serial.print(g_camLeftEdge);    Serial.print(',');
    Serial.print(g_camRightEdge);   Serial.print(',');
    Serial.print(g_camThreshold);   Serial.print(',');
    Serial.print(g_camDetected ? 1 : 0);
    for (uint8_t i = 0; i < DEMO_CAM_PIXELS; i++) {
        Serial.print(','); Serial.print(g_camPixels[i] >> 2);
    }
    Serial.println();
}

// ============================================================
//  STS3215 DIRECT PROTOCOL HELPERS  (no ServoController class)
// ============================================================
static int32_t g_stsAbsPos  = 0;       // cumulative counts (multi-turn)
static float   g_stsStepDeg = DEMO_STS_STEP_DEG_DEF;
static uint16_t g_stsSpeed  = DEMO_STS_SPEED_DEF;

static uint8_t stsCsum(uint8_t id, uint8_t len, uint8_t instr,
                       const uint8_t* p, uint8_t pLen) {
    uint8_t s = id + len + instr;
    for (uint8_t i = 0; i < pLen; i++) s += p[i];
    return ~s & 0xFF;
}

static void stsSend(uint8_t id, uint8_t instr, const uint8_t* p, uint8_t pLen) {
    uint8_t len  = pLen + 2;
    uint8_t csum = stsCsum(id, len, instr, p, pLen);
    while (Serial7.available()) Serial7.read();   // flush any stale RX bytes
    Serial7.write(STS_HDR); Serial7.write(STS_HDR);
    Serial7.write(id);      Serial7.write(len);
    Serial7.write(instr);
    for (uint8_t i = 0; i < pLen; i++) Serial7.write(p[i]);
    Serial7.write(csum);
    // At 1 Mbaud each byte takes 10 µs.  Wait for the full packet to finish
    // transmitting rather than calling flush(), which can deadlock in LOOPS mode.
    // Packet size = 6 + pLen bytes → max 11 bytes = 110 µs; 300 µs is safe margin.
    delayMicroseconds(300 + pLen * 10);
    // Discard internal loopback echo: LOOPS+RSRC routes TX back to RX, so every
    // transmitted byte appears on RX.  Total packet size = 6 + pLen bytes.
    uint32_t t0 = millis();
    for (uint8_t n = 0; n < (6 + pLen); ) {
        if (millis() - t0 > 3) break;
        if (Serial7.available()) { Serial7.read(); n++; }
    }
}

// Ping the servo — returns true if it responds.
static bool stsPing() {
    stsSend(DEMO_STS_ID, STS_INSTR_PING, nullptr, 0);
    // Response: FF FF ID 02 ERR CSUM  (6 bytes)
    uint8_t buf[6] = {};
    uint32_t t0 = millis();
    uint8_t  n  = 0;
    while (n < 6 && millis() - t0 < STS_READ_TIMEOUT_MS)
        if (Serial7.available()) buf[n++] = Serial7.read();
    return (n == 6 && buf[0] == STS_HDR && buf[1] == STS_HDR && buf[2] == DEMO_STS_ID);
}

// Read present position from the servo. Returns degrees, or -1 on failure.
static float stsReadPos() {
    uint8_t req[2] = { STS_REG_PRESENT_POS, 2 };
    stsSend(DEMO_STS_ID, STS_INSTR_READ, req, 2);
    // Response: FF FF ID 04 ERR posL posH CSUM  (8 bytes)
    uint8_t buf[8] = {};
    uint32_t t0 = millis();
    uint8_t  n  = 0;
    while (n < 8 && millis() - t0 < STS_READ_TIMEOUT_MS)
        if (Serial7.available()) buf[n++] = Serial7.read();
    if (n < 8 || buf[0] != STS_HDR || buf[1] != STS_HDR || buf[2] != DEMO_STS_ID)
        return -1.0f;
    uint16_t raw = static_cast<uint16_t>(buf[5]) | (static_cast<uint16_t>(buf[6]) << 8);
    return static_cast<float>(raw) / STS_CPD;
}

// Send a goal-position + speed command.
// pos is the 0-4095 wrapped target, speed is 1-4095 (0 = max).
static void stsSendGoal(uint16_t pos, uint16_t spd) {
    uint8_t p[5] = {
        STS_REG_GOALPOS,
        static_cast<uint8_t>(pos  & 0xFF),
        static_cast<uint8_t>(pos  >> 8),
        static_cast<uint8_t>(spd  & 0xFF),
        static_cast<uint8_t>(spd  >> 8)
    };
    stsSend(DEMO_STS_ID, STS_INSTR_WRITE, p, 5);
}

// Move the STS3215 by deltaDeg (positive = forward, negative = backward).
// Tracks cumulative angle in g_stsAbsPos but wraps the register value to 0-4095
// (one revolution). Successive steps are seamless as long as |deltaDeg| < 180°
// because the servo always picks the shorter arc to the new register value.
static void stsMoveBy(float deltaDeg) {
    int32_t delta = static_cast<int32_t>(deltaDeg * STS_CPD);
    g_stsAbsPos += delta;
    // Wrap to 0-4095 for the position register
    uint16_t pos = static_cast<uint16_t>(((g_stsAbsPos % 4096) + 4096) % 4096);
    stsSendGoal(pos, g_stsSpeed);
    float totalDeg = static_cast<float>(g_stsAbsPos) / STS_CPD;
    Serial.print(F("# STS3215 → "));
    Serial.print(totalDeg, 1);
    Serial.print(F("° (reg ")); Serial.print(pos); Serial.println(F(")"));
}

// ============================================================
//  HELP PRINTERS  (one per mode)
// ============================================================
static void printModeSelector() {
    Serial.println(F("# ---  Active mode: press ] / [ to cycle, or 1–8 to jump ---"));
    for (uint8_t m = 1; m <= 8; m++) {
        Serial.print(F("#   ")); Serial.print(m); Serial.print(F("  "));
        Serial.println(demoModeName(static_cast<DemoMode>(m)));
    }
    Serial.println(F("# -------------------------------------------------------"));
}

static void printHelpD1() {
    Serial.println(F("# === MODE 1: D1 Lead Screws ==="));
    Serial.println(F("#  1 / 2  — Select D1-S1 / D1-S2"));
    Serial.println(F("#  A      — Select ALL (both screws)"));
    Serial.println(F("#  U      — Move selected UP  by dist"));
    Serial.println(F("#  D      — Move selected DOWN by dist"));
    Serial.println(F("#  + / -  — Increase / decrease dist (1 mm)"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Dist: ")); Serial.print(g_lsDist, 1); Serial.println(F(" mm"));
    Serial.print(F("# Sel:  "));
    if (g_lsSel == 2) Serial.println(F("BOTH")); else Serial.println(D1_SCREWS[g_lsSel].name);
}

static void printHelpServo12() {
    Serial.println(F("# === MODE 2: Servos 1 & 2 (linear actuator) ==="));
    Serial.println(F("#  1 / 2  — Select Servo 1 / Servo 2"));
    Serial.println(F("#  A      — Move selected to angle A (config)"));
    Serial.println(F("#  D      — Move selected to angle B (config)"));
    Serial.println(F("#  C      — Centre (90°)"));
    Serial.println(F("#  W / S  — Step +° / -° by step size"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Active: Servo ")); Serial.println(g_servoBase + g_servoSel + 1);
    Serial.print(F("# Step: ")); Serial.print(DEMO_MG_STEP_DEG, 1); Serial.println(F("°"));
}

static void printHelpDC() {
    Serial.println(F("# === MODE 3: DC Motors ==="));
    Serial.println(F("#  W / S  — Motor A: forward / reverse"));
    Serial.println(F("#  E      — Motor A: coast"));
    Serial.println(F("#  I / K  — Motor B: forward / reverse"));
    Serial.println(F("#  O      — Motor B: coast"));
    Serial.println(F("#  B      — Brake BOTH"));
    Serial.println(F("#  X      — Coast BOTH"));
    Serial.println(F("#  T      — Timed run: both fwd for configured time, then stop"));
    Serial.println(F("#  + / -  — Speed up / slow down"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Speed: ")); Serial.print(g_dcSpeed);
    Serial.print(F("/255  (")); Serial.print(g_dcSpeed * 100 / 255); Serial.println(F("%)"));
    Serial.print(F("# Timed run: ")); Serial.print(DEMO_DC_TIMED_MS); Serial.println(F(" ms"));
}

static void printHelpGantry() {
    Serial.println(F("# === MODE 4: Gantry (open-loop) ==="));
    Serial.println(F("#  W / S  — +Y / -Y  (hold for continuous)"));
    Serial.println(F("#  A / D  — -X / +X  (hold for continuous)"));
    Serial.println(F("#  X      — STOP movement"));
    Serial.println(F("#  F / G  — Faster / slower step rate"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.println(F("# To stop: press X or Space"));
    Serial.print(F("# Step delay: ")); Serial.print(g_gStepUs); Serial.println(F(" µs"));
    Serial.println(F("# CoreXY: W=+Y(Afwd,Bbwd)  S=-Y  D=+X  A=-X"));
}

static void printHelpCamera() {
    Serial.println(F("# === MODE 5: Camera (TSL1401 + laser) ==="));
    Serial.println(F("#  R      — Read one frame (raw CSV)"));
    Serial.println(F("#  C      — Toggle continuous mode (~10 Hz)"));
    Serial.println(F("#  S      — Toggle stream for Python live plotter"));
    Serial.println(F("#  B      — Burst: stream for configured seconds then stop"));
    Serial.println(F("#           Python: camera_burst.py captures & animates"));
    Serial.println(F("#  W      — ASCII waveform"));
    Serial.println(F("#  + / -  — CLK half-period (sensitivity)"));
    Serial.println(F("#  L      — Toggle laser manually"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# CLK ½T: ")); Serial.print(g_camClkHalfUs); Serial.println(F(" µs"));
    if (!camPinsOk())
        Serial.println(F("# WARNING: SI/CLK pins not set in PinMap.h."));
}

static void printHelpServo34() {
    Serial.println(F("# === MODE 6: Servos 3 & 4 (syringe/tweezers) ==="));
    Serial.println(F("#  1 / 2  — Select Servo 3 / Servo 4"));
    Serial.println(F("#  A      — Move selected to angle A (config)"));
    Serial.println(F("#  D      — Move selected to angle B (config)"));
    Serial.println(F("#  C      — Centre (90°)"));
    Serial.println(F("#  W / S  — Step +° / -°"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Active: Servo ")); Serial.println(g_servoBase + g_servoSel + 1);
}

static void printHelpSTS() {
    Serial.println(F("# === MODE 7: STS3215 (z-axis) ==="));
    Serial.println(F("#  U      — Rotate forward by step angle"));
    Serial.println(F("#  D      — Rotate backward by step angle"));
    Serial.println(F("#  H      — Go to home (position 0)"));
    Serial.println(F("#  + / -  — Increase / decrease step angle"));
    Serial.println(F("#  P      — Read hardware position + software counter"));
    Serial.println(F("#  G      — Ping servo (verify connection)"));
    Serial.println(F("#  R      — Reset software counter to 0 (no move)"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Step: ")); Serial.print(g_stsStepDeg, 1); Serial.println(F("°"));
    Serial.print(F("# Speed reg: ")); Serial.println(g_stsSpeed);
    Serial.print(F("# Software counter: "));
    Serial.print(static_cast<float>(g_stsAbsPos) / STS_CPD, 1);
    Serial.println(F("°  (keep step < 180° for correct arc direction)"));
}

static void printHelpD2() {
    Serial.println(F("# === MODE 8: D2 Lead Screws ==="));
    Serial.println(F("#  1 / 2  — Select D2-S1 / D2-S2"));
    Serial.println(F("#  A      — Select ALL (both screws)"));
    Serial.println(F("#  U      — Move selected UP  by dist"));
    Serial.println(F("#  D      — Move selected DOWN by dist"));
    Serial.println(F("#  + / -  — Increase / decrease dist (1 mm)"));
    Serial.println(F("#  ?      — This help     ] / [  — Change mode"));
    Serial.print(F("# Dist: ")); Serial.print(g_lsDist, 1); Serial.println(F(" mm"));
    Serial.print(F("# Sel:  "));
    if (g_lsSel == 2) Serial.println(F("BOTH")); else Serial.println(D2_SCREWS[g_lsSel].name);
}

static void printCurrentHelp() {
    switch (g_mode) {
        case DemoMode::D1_SCREWS: printHelpD1();     break;
        case DemoMode::SERVO_12:  printHelpServo12();break;
        case DemoMode::DC_MOTORS: printHelpDC();     break;
        case DemoMode::GANTRY:    printHelpGantry(); break;
        case DemoMode::CAMERA:    printHelpCamera(); break;
        case DemoMode::SERVO_34:  printHelpServo34();break;
        case DemoMode::STS:       printHelpSTS();    break;
        case DemoMode::D2_SCREWS: printHelpD2();     break;
    }
}

// ============================================================
//  MODE SWITCH
// ============================================================
static void switchMode(DemoMode next) {
    // Exit cleanup
    if (g_mode == DemoMode::GANTRY)   g_gDir = GDir::NONE;
    if (g_mode == DemoMode::DC_MOTORS) { dcCoast(DC_A_ARGS); dcCoast(DC_B_ARGS); }
    if (g_mode == DemoMode::CAMERA && g_camLaserOn) camLaserOff();

    g_mode = next;

    // Entry setup
    if (g_mode == DemoMode::SERVO_12) {
        g_servoBase = 0; g_servoSel = 0;
    }
    if (g_mode == DemoMode::SERVO_34) {
        g_servoBase = 2; g_servoSel = 0;
    }
    if (g_mode == DemoMode::D1_SCREWS || g_mode == DemoMode::D2_SCREWS) {
        g_lsSel = 0;
    }

    Serial.println(F("# ============================================="));
    Serial.print(F("# MODE: ")); Serial.println(demoModeName(g_mode));
    Serial.println(F("# ============================================="));
    printCurrentHelp();
}

// ============================================================
//  PER-MODE KEY HANDLERS
// ============================================================

// ---- Lead-screw mode (D1 or D2) ---
static void handleLeadScrewKey(char c, const ScrewInfo screws[]) {
    static constexpr float DIST_STEP = 1.0f, DIST_MAX = 50.0f;
    switch (c) {
        case '1': g_lsSel = 0;
            Serial.print(F("# Selected: ")); Serial.println(screws[0].name); break;
        case '2': g_lsSel = 1;
            Serial.print(F("# Selected: ")); Serial.println(screws[1].name); break;
        case 'a': case 'A': g_lsSel = 2;
            Serial.println(F("# Selected: BOTH")); break;
        case 'u': case 'U':
            if (g_lsSel == 2) { lsMove(screws[0], g_lsDist, true); lsMove(screws[1], g_lsDist, true); }
            else               lsMoveSelected(screws, true);
            break;
        case 'd': case 'D':
            if (g_lsSel == 2) { lsMove(screws[0], g_lsDist, false); lsMove(screws[1], g_lsDist, false); }
            else               lsMoveSelected(screws, false);
            break;
        case '+': case '=':
            if (g_lsDist + DIST_STEP <= DIST_MAX) g_lsDist += DIST_STEP; else g_lsDist = DIST_MAX;
            Serial.print(F("# Dist: ")); Serial.print(g_lsDist, 1); Serial.println(F(" mm")); break;
        case '-': case '_':
            if (g_lsDist - DIST_STEP >= DIST_STEP) g_lsDist -= DIST_STEP; else g_lsDist = DIST_STEP;
            Serial.print(F("# Dist: ")); Serial.print(g_lsDist, 1); Serial.println(F(" mm")); break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

// ---- MG996R servo mode (base is 0 for servos 1&2, 2 for servos 3&4) ---
static void handleServoKey(char c) {
    // Angle A/B values indexed by global servo index (0–3)
    static const float ANGLE_A[4] = { DEMO_MG12_S1_A, DEMO_MG12_S2_A, DEMO_MG34_S3_A, DEMO_MG34_S4_A };
    static const float ANGLE_B[4] = { DEMO_MG12_S1_B, DEMO_MG12_S2_B, DEMO_MG34_S3_B, DEMO_MG34_S4_B };

    uint8_t idx = g_servoBase + g_servoSel;

    switch (c) {
        case '1': g_servoSel = 0;
            Serial.print(F("# Active: Servo ")); Serial.println(g_servoBase + 1); break;
        case '2': g_servoSel = 1;
            Serial.print(F("# Active: Servo ")); Serial.println(g_servoBase + 2); break;
        case 'a': case 'A': servoSet(idx, ANGLE_A[idx]); break;
        case 'd': case 'D': servoSet(idx, ANGLE_B[idx]); break;
        case 'c': case 'C': servoSet(idx, 90.0f); break;
        case 'w': case 'W': servoSet(idx, g_servoAngle[idx] + DEMO_MG_STEP_DEG); break;
        case 's': case 'S': servoSet(idx, g_servoAngle[idx] - DEMO_MG_STEP_DEG); break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

// ---- DC motors ---
static void handleDCKey(char c) {
    bool pinsOk = (Pins::DCMotors::MOTOR_A_IN1 >= 0 &&
                   Pins::DCMotors::MOTOR_A_IN2 >= 0 &&
                   Pins::DCMotors::MOTOR_B_IN3 >= 0 &&
                   Pins::DCMotors::MOTOR_B_IN4 >= 0);

    switch (c) {
        case 'w': case 'W':
            if (!pinsOk) { Serial.println(F("# DC pins not set")); break; }
            dcSet(DC_A_ARGS, true, g_dcSpeed);
            Serial.print(F("# Motor A FWD @ ")); Serial.println(g_dcSpeed); break;
        case 's': case 'S':
            if (!pinsOk) { Serial.println(F("# DC pins not set")); break; }
            dcSet(DC_A_ARGS, false, g_dcSpeed);
            Serial.print(F("# Motor A REV @ ")); Serial.println(g_dcSpeed); break;
        case 'e': case 'E':
            dcCoast(DC_A_ARGS); Serial.println(F("# Motor A COAST")); break;
        case 'i': case 'I':
            if (!pinsOk) { Serial.println(F("# DC pins not set")); break; }
            dcSet(DC_B_ARGS, true, g_dcSpeed);
            Serial.print(F("# Motor B FWD @ ")); Serial.println(g_dcSpeed); break;
        case 'k': case 'K':
            if (!pinsOk) { Serial.println(F("# DC pins not set")); break; }
            dcSet(DC_B_ARGS, false, g_dcSpeed);
            Serial.print(F("# Motor B REV @ ")); Serial.println(g_dcSpeed); break;
        case 'o': case 'O':
            dcCoast(DC_B_ARGS); Serial.println(F("# Motor B COAST")); break;
        case 'b': case 'B':
            dcBrake(DC_A_ARGS); dcBrake(DC_B_ARGS);
            Serial.println(F("# BRAKE both")); break;
        case 'x': case 'X':
            dcCoast(DC_A_ARGS); dcCoast(DC_B_ARGS);
            Serial.println(F("# COAST both")); break;
        case 't': case 'T':
            if (!pinsOk) { Serial.println(F("# DC pins not set")); break; }
            Serial.print(F("# Timed run: both FWD for ")); Serial.print(DEMO_DC_TIMED_MS); Serial.println(F(" ms"));
            dcSet(DC_A_ARGS, true, g_dcSpeed);
            dcSet(DC_B_ARGS, true, g_dcSpeed);
            delay(DEMO_DC_TIMED_MS);
            dcCoast(DC_A_ARGS); dcCoast(DC_B_ARGS);
            Serial.println(F("# Timed run complete.")); break;
        case '+': case '=':
            if (g_dcSpeed <= 255 - DEMO_DC_SPEED_STEP) g_dcSpeed += DEMO_DC_SPEED_STEP; else g_dcSpeed = 255;
            Serial.print(F("# Speed: ")); Serial.print(g_dcSpeed); Serial.print(F("/255  ("));
            Serial.print(g_dcSpeed * 100 / 255); Serial.println(F("%)")); break;
        case '-': case '_':
            if (g_dcSpeed >= DEMO_DC_SPEED_STEP) g_dcSpeed -= DEMO_DC_SPEED_STEP; else g_dcSpeed = 0;
            Serial.print(F("# Speed: ")); Serial.print(g_dcSpeed); Serial.print(F("/255  ("));
            Serial.print(g_dcSpeed * 100 / 255); Serial.println(F("%)")); break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

// ---- Gantry ---
static void handleGantryKey(char c) {
    switch (c) {
        case 'w': case 'W':
            if (g_gDir != GDir::POS_Y) {
                g_gDir = GDir::POS_Y;
                Serial.println(F("# Gantry +Y  (press X to stop)"));
            }
            break;
        case 's': case 'S':
            if (g_gDir != GDir::NEG_Y) {
                g_gDir = GDir::NEG_Y;
                Serial.println(F("# Gantry -Y  (press X to stop)"));
            }
            break;
        case 'd': case 'D':
            if (g_gDir != GDir::POS_X) {
                g_gDir = GDir::POS_X;
                Serial.println(F("# Gantry +X  (press X to stop)"));
            }
            break;
        case 'a': case 'A':
            if (g_gDir != GDir::NEG_X) {
                g_gDir = GDir::NEG_X;
                Serial.println(F("# Gantry -X  (press X to stop)"));
            }
            break;
        case 'x': case 'X': case ' ':
            g_gDir = GDir::NONE;
            Serial.println(F("# Gantry STOP")); break;
        case 'f': case 'F':
            if (g_gStepUs / 2 >= DEMO_G_STEP_US_MIN) g_gStepUs /= 2;
            else g_gStepUs = DEMO_G_STEP_US_MIN;
            Serial.print(F("# Step delay: ")); Serial.print(g_gStepUs); Serial.println(F(" µs (faster)")); break;
        case 'g': case 'G':
            if (g_gStepUs * 2 <= DEMO_G_STEP_US_MAX) g_gStepUs *= 2;
            else g_gStepUs = DEMO_G_STEP_US_MAX;
            Serial.print(F("# Step delay: ")); Serial.print(g_gStepUs); Serial.println(F(" µs (slower)")); break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F && c != '\r' && c != '\n') {
                // Any other key stops movement (safety)
                if (g_gDir != GDir::NONE) {
                    g_gDir = GDir::NONE;
                    Serial.println(F("# Gantry STOP (key '?' for help)"));
                }
            }
            break;
    }
}

// ---- Camera ---
static void handleCameraKey(char c) {
    switch (c) {
        case 'r': case 'R':
            if (!camPinsOk()) { Serial.println(F("# Pins not set")); break; }
            camCapture(); camFindCenter();
            camPrintCsv();
            Serial.print(F("# Centre: ")); Serial.print(g_camLastCenter);
            Serial.print(F("  Threshold: ")); Serial.print(g_camThreshold);
            Serial.print(F("  Detected: ")); Serial.println(g_camDetected ? F("YES") : F("NO"));
            break;
        case 'c': case 'C':
            g_camContinuous = !g_camContinuous;
            Serial.println(g_camContinuous ? F("# Continuous ON") : F("# Continuous OFF"));
            break;
        case 's': case 'S':
            g_camStreaming = !g_camStreaming;
            Serial.println(g_camStreaming ? F("# Stream ON") : F("# Stream OFF"));
            break;
        case 'w': case 'W': {
            // ASCII waveform
            Serial.println(F("# Waveform (8-bit, 4 pixels/row):"));
            for (uint8_t row = 0; row < 32; row++) {
                uint8_t base = row * 4;
                uint32_t sum = 0;
                for (uint8_t k = 0; k < 4; k++) sum += (g_camPixels[base + k] >> 2);
                uint8_t avg = static_cast<uint8_t>(sum / 4);
                uint8_t bar = static_cast<uint8_t>((uint16_t)avg * 20 / 255);
                if (base < 10) Serial.print(F("  "));
                else if (base < 100) Serial.print(' ');
                Serial.print(base); Serial.print(F(" |"));
                for (uint8_t b = 0; b < bar; b++) Serial.print('#');
                if (g_camLastCenter >= base && g_camLastCenter < base + 4) {
                    for (uint8_t b = bar; b < 20; b++) Serial.print(' ');
                    Serial.print(F("| <-- centre"));
                }
                Serial.println();
            }
            break;
        }
        case '+': case '=':
            if (g_camClkHalfUs + DEMO_CAM_CLK_STEP_US <= DEMO_CAM_CLK_MAX_US)
                g_camClkHalfUs += DEMO_CAM_CLK_STEP_US;
            else g_camClkHalfUs = DEMO_CAM_CLK_MAX_US;
            Serial.print(F("# CLK ½T: ")); Serial.print(g_camClkHalfUs); Serial.println(F(" µs")); break;
        case '-': case '_':
            if (g_camClkHalfUs > DEMO_CAM_CLK_MIN_US + DEMO_CAM_CLK_STEP_US)
                g_camClkHalfUs -= DEMO_CAM_CLK_STEP_US;
            else g_camClkHalfUs = DEMO_CAM_CLK_MIN_US;
            Serial.print(F("# CLK ½T: ")); Serial.print(g_camClkHalfUs); Serial.println(F(" µs")); break;
        case 'b': case 'B':
            if (!camPinsOk()) { Serial.println(F("# Pins not set")); break; }
            g_camBurst      = true;
            g_camStreaming   = true;
            g_camBurstEndMs = millis() + (DEMO_CAM_BURST_S * 1000UL);
            g_camLastMs     = 0;   // fire first frame immediately
            Serial.println(F("# BURST_START"));
            break;
        case 'l': case 'L':
            if (g_camLaserOn) { camLaserOff(); Serial.println(F("# Laser OFF")); }
            else              { camLaserOn();  Serial.println(F("# Laser ON (press L to turn off)")); }
            break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

// ---- STS3215 ---
static void handleSTSKey(char c) {
    switch (c) {
        case 'u': case 'U': stsMoveBy( g_stsStepDeg); break;
        case 'd': case 'D': stsMoveBy(-g_stsStepDeg); break;
        case 'h': case 'H':
            g_stsAbsPos = 0;
            stsSendGoal(0, g_stsSpeed);
            Serial.println(F("# STS3215 → home (0°)")); break;
        case '+': case '=':
            if (g_stsStepDeg + DEMO_STS_STEP_STEP <= DEMO_STS_STEP_MAX)
                g_stsStepDeg += DEMO_STS_STEP_STEP;
            else g_stsStepDeg = DEMO_STS_STEP_MAX;
            Serial.print(F("# Step: ")); Serial.print(g_stsStepDeg, 1); Serial.println(F("°")); break;
        case '-': case '_':
            if (g_stsStepDeg - DEMO_STS_STEP_STEP >= DEMO_STS_STEP_MIN)
                g_stsStepDeg -= DEMO_STS_STEP_STEP;
            else g_stsStepDeg = DEMO_STS_STEP_MIN;
            Serial.print(F("# Step: ")); Serial.print(g_stsStepDeg, 1); Serial.println(F("°")); break;
        case 'p': case 'P': {
            float hw = stsReadPos();
            Serial.print(F("# Hardware position: "));
            if (hw < 0.0f) Serial.println(F("read failed (check wiring/ID/baud)"));
            else { Serial.print(hw, 1); Serial.println(F("°")); }
            Serial.print(F("# Software counter:  "));
            Serial.print(static_cast<float>(g_stsAbsPos) / STS_CPD, 1);
            Serial.println(F("°")); break;
        }
        case 'g': case 'G':
            if (stsPing()) Serial.println(F("# Ping OK — servo responding"));
            else           Serial.println(F("# Ping FAILED — check wiring, ID, baud"));
            break;
        case 'r': case 'R':
            g_stsAbsPos = 0;
            Serial.println(F("# Counter reset to 0 (no servo move)")); break;
        case '?': printCurrentHelp(); break;
        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

// ============================================================
//  CAMERA CONTINUOUS / STREAM TICK  (called every loop)
// ============================================================
static void cameraTick() {
    // Auto-stop burst when timer expires
    if (g_camBurst && millis() >= g_camBurstEndMs) {
        g_camBurst    = false;
        g_camStreaming = false;
        Serial.println(F("# BURST_END"));
        return;
    }

    if (!g_camContinuous && !g_camStreaming) return;
    uint32_t now = millis();
    if (now - g_camLastMs < 100) return;
    g_camLastMs = now;
    if (!camPinsOk()) return;
    camCapture();
    camFindCenter();
    if (g_camStreaming) {
        camPrintStreamFrame();
    } else {
        Serial.print(now);          Serial.print(',');
        Serial.print(g_camLastCenter); Serial.print(',');
        Serial.print(g_camThreshold);  Serial.print(',');
        Serial.println(g_camDetected ? 1 : 0);
    }
}

// ============================================================
//  GLOBAL KEY ROUTER  (handles mode switching before per-mode)
// ============================================================
static bool handleGlobalKey(char c) {
    // ] advances mode, [ goes back
    if (c == ']') {
        uint8_t next = static_cast<uint8_t>(g_mode) + 1;
        if (next > 8) next = 1;
        switchMode(static_cast<DemoMode>(next));
        return true;
    }
    if (c == '[') {
        uint8_t prev = static_cast<uint8_t>(g_mode) - 1;
        if (prev < 1) prev = 8;
        switchMode(static_cast<DemoMode>(prev));
        return true;
    }
    if (c >= '1' && c <= '8') {
        DemoMode requested = static_cast<DemoMode>(c - '0');
        if (requested != g_mode) {
            switchMode(requested);
            return true;
        }
        // Fall through so mode can also use 1/2 for selection
    }
    return false;
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testDemo_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(300);

    // ---- Lead screw step/dir pins ----
    const ScrewInfo* allScrews[4] = {
        &D1_SCREWS[0], &D1_SCREWS[1], &D2_SCREWS[0], &D2_SCREWS[1]
    };
    for (uint8_t i = 0; i < 4; i++) {
        if (allScrews[i]->stepPin >= 0) {
            pinMode(allScrews[i]->stepPin, OUTPUT);
            digitalWrite(allScrews[i]->stepPin, LOW);
        }
        if (allScrews[i]->dirPin >= 0) {
            pinMode(allScrews[i]->dirPin, OUTPUT);
        }
    }

    // ---- Gantry step/dir pins ----
    if (Pins::Gantry::MOTOR_A_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_A_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW); }
    if (Pins::Gantry::MOTOR_A_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_A_DIR,  OUTPUT); }
    if (Pins::Gantry::MOTOR_B_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_B_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW); }
    if (Pins::Gantry::MOTOR_B_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_B_DIR,  OUTPUT); }

    // ---- DC motor pins ----
    if (Pins::DCMotors::MOTOR_A_IN1 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN1, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW); }
    if (Pins::DCMotors::MOTOR_A_IN2 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN2, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW); }
    if (Pins::DCMotors::MOTOR_A_ENA >= 0) { pinMode(Pins::DCMotors::MOTOR_A_ENA, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0); }
    if (Pins::DCMotors::MOTOR_B_IN3 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN3, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW); }
    if (Pins::DCMotors::MOTOR_B_IN4 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN4, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW); }
    if (Pins::DCMotors::MOTOR_B_ENB >= 0) { pinMode(Pins::DCMotors::MOTOR_B_ENB, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0); }

    // ---- MG996R servos ----
    for (uint8_t i = 0; i < 4; i++) {
        int pin = servoPinOf(i);
        if (pin < 0) continue;
        g_servo[i].attach(pin, DEMO_MG_MIN_US, DEMO_MG_MAX_US);
        g_servoAttached[i] = true;
        g_servo[i].write(static_cast<int>(g_servoAngle[i]));
    }

    // ---- Camera pins ----
    if (Pins::LaserSensor::CLK >= 0) { pinMode(Pins::LaserSensor::CLK, OUTPUT); digitalWrite(Pins::LaserSensor::CLK, LOW); }
    if (Pins::LaserSensor::SI  >= 0) { pinMode(Pins::LaserSensor::SI,  OUTPUT); digitalWrite(Pins::LaserSensor::SI,  LOW); }
    if (Pins::LineLaser::ENABLE >= 0){ pinMode(Pins::LineLaser::ENABLE, OUTPUT); digitalWrite(Pins::LineLaser::ENABLE, LOW); }
    if (camPinsOk()) camCapture();  // prime pipeline

    // ---- STS3215 — Serial7, single-wire half-duplex on pin 29 only ----
    // LOOPS+RSRC routes TX output internally back to RX so the Teensy reads
    // its own echo AND the servo response on pin 29.  Pin 28 is unused.
    Serial7.begin(DEMO_STS_BAUD);
    LPUART7_CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
    // Ping deferred to first use — press G in mode 7 to verify connection.

    // ---- Print initial mode help ----
    Serial.println(F("# ============================================="));
    Serial.println(F("# Demo Test  —  mode ] / [ or 1-8 to switch"));
    Serial.println(F("# ============================================="));
    printModeSelector();
    switchMode(DemoMode::D1_SCREWS);
}

void testDemo_loop() {
    // --- Background ticks ---
    if (g_mode == DemoMode::GANTRY) gantryTick();
    if (g_mode == DemoMode::CAMERA) cameraTick();

    // --- Serial input ---
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());

        // Skip CR/LF
        if (c == '\r' || c == '\n') continue;

        // Global mode-switch keys (] [ 1-8)
        // Numbers 1-8 are also used for sub-selection in some modes,
        // so only switch mode if we're not already in that mode.
        if (c == ']' || c == '[') { handleGlobalKey(c); continue; }

        // '1'-'8' switch mode only if not a sub-selection key in current mode
        bool modeHasNumberKeys =
            (g_mode == DemoMode::D1_SCREWS || g_mode == DemoMode::D2_SCREWS ||
             g_mode == DemoMode::SERVO_12   || g_mode == DemoMode::SERVO_34);

        if (c >= '1' && c <= '8' && !modeHasNumberKeys) {
            if (handleGlobalKey(c)) continue;
        }

        // Dispatch to active mode
        switch (g_mode) {
            case DemoMode::D1_SCREWS: handleLeadScrewKey(c, D1_SCREWS); break;
            case DemoMode::SERVO_12:  handleServoKey(c); break;
            case DemoMode::DC_MOTORS: handleDCKey(c); break;
            case DemoMode::GANTRY:    handleGantryKey(c); break;
            case DemoMode::CAMERA:    handleCameraKey(c); break;
            case DemoMode::SERVO_34:  handleServoKey(c); break;
            case DemoMode::STS:       handleSTSKey(c); break;
            case DemoMode::D2_SCREWS: handleLeadScrewKey(c, D2_SCREWS); break;
        }
    }
}

#endif // TEST_MODE
