// ============================================================
//  testing/test_transfer.cpp  —  Full transfer process simulation
//
//  SEQUENCE
//  --------
//   1. D1-S1 lift 15 mm, then D1-S2 lift 15 mm
//   2. MG996R servo 1 → angle, then servo 2 → angle (one at a time)
//   3. DC Motor A runs slow until SPACE; delay; DC Motor B until SPACE
//   4. WASD + C  → manually drive gantry, press C to set (0, 0)
//   5. Scan +Y over dish 1 (callus + outline), return, shift +X,
//      scan +Y over dish 2 (outline only), return to (0, 0)
//   6. Tool pickup: move to each dish, raise/lower STS3215, actuate MG3
//   7. Calculate 5 × 6 grid on dish 2; write scan CSV over Serial
//   8. Transfer each callus from dish 1 to grid position on dish 2
//   9. DC motors again (same as step 3)
//  10. D2-S1 down 15 mm (Space to start), then D2-S2 down 15 mm
//
//  E-STOP
//  ------
//  Press Q at any time to halt immediately. All motor outputs are
//  coasted. Re-upload firmware to reset.
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testTransfer_setup();
//    extern void testTransfer_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include <Servo.h>
#include "pins/PinMap.h"
#include "testing/callus_detect.h"

// ============================================================
//  CONFIGURATION  —  edit all values here before first run
// ============================================================

// ---- E-stop / proceed keys ---------------------------------
static constexpr char ESTOP_KEY   = 'Q';   // uppercase or lowercase both work
static constexpr char PROCEED_KEY = ' ';   // Space bar = proceed
static constexpr char SKIP_KEY    = 'N';   // N = skip current step and advance to next

// ---- Lead screws -------------------------------------------
static constexpr float    XFER_LS_LIFT_MM    = 20.0f;  // lift / lower distance (mm)
static constexpr float    XFER_LS_SPEED_MMS  = 15.0f;   // travel speed (mm/s)
static constexpr int      XFER_LS_MS         = 1;      // microstepping — match TB6600 DIP
static constexpr float    XFER_LS_LEAD_MM    = 8.0f;   // T8 leadscrew: 8 mm/rev
static constexpr int      XFER_LS_STEPS_REV  = 200;    // NEMA 17 full steps/rev
static constexpr int      XFER_LS_DIR_UP     = LOW;   // DIR level that lifts platform
static constexpr uint32_t XFER_LS_PULSE_US   = 5;      // STEP pulse high time (µs)

// ---- MG996R servos 1 & 2 (step 2) -------------------------
static constexpr float XFER_S1_INITIAL_DEG = 10.0f;     // TODO: servo 1 resting / start angle
static constexpr float XFER_S2_INITIAL_DEG = 0.0f;     // TODO: servo 2 resting / start angle
static constexpr float XFER_S1_TARGET_DEG  = 180.0f;    // TODO: servo 1 actuated angle
static constexpr float XFER_S2_TARGET_DEG  = 180.0f;    // TODO: servo 2 actuated angle
static constexpr int   XFER_MG_MIN_US      = 500;
static constexpr int   XFER_MG_MAX_US      = 2500;
static constexpr uint32_t XFER_SERVO_SETTLE_MS = 600;  // ms to wait after commanding a servo

// ---- DC motors (steps 3 & 9) -------------------------------
static constexpr uint8_t  XFER_DC_SPEED    = 10;      // 0–255; keep low for safety
static constexpr uint32_t XFER_DC_DELAY_MS = 1000;     // ms delay between motor 1 stop and motor 2 start

// ---- Gantry (open-loop, step-counted) ----------------------
// DIR pin levels — must match physical wiring (INVERT_A/B = true → FWD = LOW)
static constexpr int      XFER_G_DIR_A_FWD    = LOW;
static constexpr int      XFER_G_DIR_B_FWD    = LOW;
static constexpr uint32_t XFER_G_STEP_US      = 800;   // inter-step delay (µs); lower = faster
static constexpr uint32_t XFER_G_PULSE_US     = 5;     // STEP pin high time (µs)
static constexpr float    XFER_G_STEPS_PER_MM = 20.0f; // from Constants::Gantry::STEPS_PER_MM
static constexpr float    XFER_JOG_STEP_MM    = 5.0f;  // default distance per WASD press in step 4

// ---- Scanning (step 5) -------------------------------------
static constexpr float    XFER_SCAN_Y_MM      = 60.0f; // how far to travel in +Y while scanning
static constexpr uint16_t XFER_SCAN_SAMPLE_N  = 10;     // capture one frame every N gantry steps
static constexpr float    XFER_DISH_X_OFFSET_MM = 215.0f; // X distance from dish 1 to dish 2
static constexpr uint32_t XFER_CAM_CLK_HALF_US = 10;    // TSL1401 CLK half-period (µs)

// ---- STS3215 smart servo -----------------------------------
static constexpr uint8_t  XFER_STS_ID         = 1;
static constexpr uint32_t XFER_STS_BAUD       = 1000000;
static constexpr uint16_t XFER_STS_SPEED      = 300;    // 1–4095; 0 = max speed
static constexpr float    XFER_TOOL_RAISE_DEG = 10.0f;  // delta angle to raise tool
static constexpr float    XFER_TOOL_LOWER_DEG = 10.0f;  // delta angle to lower tool
static constexpr uint32_t XFER_STS_SETTLE_MS  = 600;    // ms to wait for servo to reach target

// ---- MG996R servo 3 (syringe actuator, step 6) -------------
static constexpr float XFER_MG3_ENGAGE_DEG  = 150.0f;   // angle to engage syringe
static constexpr float XFER_MG3_RELEASE_DEG = 90.0f;   // angle to release syringe

// ---- MG996R servo 4 (tweezers/clamp, step 8) ---------------
static constexpr float XFER_MG4_CLAMP_DEG   = 75.0f;   // angle to clamp tweezers
static constexpr float XFER_MG4_RELEASE_DEG = 90.0f;   // angle to release tweezers

// ---- Step 6: dish pick-up offsets --------------------------
static constexpr float XFER_PICKUP_X_OFFSET_MM  =  0.0f; // tool X offset from dish center
static constexpr float XFER_PICKUP_Y_OFFSET_MM  =  0.0f; // tool Y offset from dish center
static constexpr float XFER_PICKUP_Y_DELTA_MM   = 20.0f; // Y shift during the pickup sequence
static constexpr float XFER_PICKUP_X_SHIFT_D2   =  0.0f; // extra X offset to dish 2 center

// ---- Step 8: callus transfer offsets -----------------------
static constexpr float XFER_CALLUS_X_OFFSET_MM  =  0.0f; // tool X offset from callus center
static constexpr float XFER_CALLUS_Y_OFFSET_MM  =  0.0f; // tool Y offset from callus center
static constexpr float XFER_GRID_X_OFFSET_MM    =  0.0f; // tool X offset from grid position
static constexpr float XFER_GRID_Y_OFFSET_MM    =  0.0f; // tool Y offset from grid position

// ---- 5 × 6 grid on dish 2 ----------------------------------
static constexpr uint8_t GRID_COLS        = 5;
static constexpr uint8_t GRID_ROWS        = 6;
static constexpr float   GRID_MARGIN_MM   = 10.0f; // min distance from dish edge to grid corner

// ============================================================
//  DERIVED CONSTANTS
// ============================================================
static constexpr float    LS_SPM =
    static_cast<float>(XFER_LS_STEPS_REV * XFER_LS_MS) / XFER_LS_LEAD_MM;
static constexpr uint32_t LS_STEP_DELAY_US =
    static_cast<uint32_t>(1000000.0f / (XFER_LS_SPEED_MMS * LS_SPM));

static constexpr int XFER_G_DIR_A_BWD = (XFER_G_DIR_A_FWD == LOW) ? HIGH : LOW;
static constexpr int XFER_G_DIR_B_BWD = (XFER_G_DIR_B_FWD == LOW) ? HIGH : LOW;

// STS3215 protocol bytes
static constexpr uint8_t STS_HDR         = 0xFF;
static constexpr uint8_t STS_INSTR_WRITE = 0x03;
static constexpr uint8_t STS_REG_GOALPOS = 0x2A;
static constexpr float   STS_CPD         = 4095.0f / 360.0f; // counts per degree

// ============================================================
//  RUNTIME STATE
// ============================================================
static bool  g_eStopped = false;
static float g_posX_mm  = 0.0f;   // gantry X position relative to reference
static float g_posY_mm  = 0.0f;   // gantry Y position relative to reference

static Servo g_servo[4];
static bool  g_servoAttached[4] = {false, false, false, false};
static float g_servoAngle[4]    = {XFER_S1_INITIAL_DEG, XFER_S2_INITIAL_DEG, 90.0f, 90.0f};

static int32_t g_stsAbsPos = 0;   // cumulative STS3215 position (counts, multi-turn)

// TSL1401 pixel buffer
static uint16_t g_camPixels[128];

// Scan data log for CSV output
static constexpr uint16_t MAX_SCAN_FRAMES = 600;
struct ScanFrame {
    float   y_mm;
    uint8_t centerPx;
    uint8_t widthPx;
    bool    detected;
};
static ScanFrame g_scanFrames[MAX_SCAN_FRAMES];
static uint16_t  g_scanFrameCount = 0;

// Dish and callus results populated by step 5
static CallusDetect::DishInfo g_dish1;
static CallusDetect::DishInfo g_dish2;
static float g_dish2GlobalX = 0.0f;   // dish 2 center X in global frame
static float g_dish2GlobalY = 0.0f;   // dish 2 center Y in global frame

static CallusDetect::CallusPos g_calluses[CallusDetect::MAX_CALLUSES];
static uint8_t g_callusCount = 0;

// Grid positions (global frame, relative to reference origin)
static float g_gridX[GRID_ROWS][GRID_COLS];
static float g_gridY[GRID_ROWS][GRID_COLS];
static bool  g_gridValid = false;

// ============================================================
//  E-STOP
// ============================================================
static void triggerEStop()
{
    // Halt all step pins
    if (Pins::Gantry::MOTOR_A_STEP >= 0) digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    if (Pins::Gantry::MOTOR_B_STEP >= 0) digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);

    // Coast all lead-screw step pins low (TB6600 idles when STEP is held low)
    const int lsPins[] = {
        Pins::LeadScrews::DEVICE1_SCREW1_STEP, Pins::LeadScrews::DEVICE1_SCREW2_STEP,
        Pins::LeadScrews::DEVICE2_SCREW1_STEP, Pins::LeadScrews::DEVICE2_SCREW2_STEP
    };
    for (int p : lsPins) if (p >= 0) digitalWrite(p, LOW);

    // Coast DC motors
    if (Pins::DCMotors::MOTOR_A_IN1 >= 0) { digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW); }
    if (Pins::DCMotors::MOTOR_A_IN2 >= 0) { digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW); }
    if (Pins::DCMotors::MOTOR_A_ENA >= 0) { analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0); }
    if (Pins::DCMotors::MOTOR_B_IN3 >= 0) { digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW); }
    if (Pins::DCMotors::MOTOR_B_IN4 >= 0) { digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW); }
    if (Pins::DCMotors::MOTOR_B_ENB >= 0) { analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0); }

    // Laser off
    if (Pins::LineLaser::ENABLE >= 0) digitalWrite(Pins::LineLaser::ENABLE, LOW);

    g_eStopped = true;
    Serial.println(F("# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("# !!!         E-STOP TRIGGERED           !!!"));
    Serial.println(F("# !!!  All motors halted and coasted.    !!!"));
    Serial.println(F("# !!!  Re-upload firmware to reset.      !!!"));
    Serial.println(F("# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
}

// Returns true if E-stop is active or Q/q was just pressed.
// Call at any natural check-point inside loops.
static bool checkEStop()
{
    if (g_eStopped) return true;
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == ESTOP_KEY || c == (ESTOP_KEY + 32)) {
            triggerEStop();
            return true;
        }
    }
    return false;
}

// Block until PROCEED_KEY (Space) or SKIP_KEY (N) is pressed.
// Returns true = proceed, false = skip or E-stop.
// NOTE: does NOT call checkEStop() internally — that would drain the serial
// buffer and swallow the Space key before this function could see it.
static bool waitForSpace(const __FlashStringHelper* prompt = nullptr)
{
    if (prompt) {
        Serial.print(F("# ")); Serial.println(prompt);
    }
    Serial.print(F("#   → SPACE proceed  |  "));
    Serial.print(SKIP_KEY);
    Serial.print(F(" skip step  |  "));
    Serial.print(ESTOP_KEY);
    Serial.println(F(" E-stop"));

    while (true) {
        if (g_eStopped) return false;
        while (Serial.available()) {
            char c = static_cast<char>(Serial.read());
            if (c == PROCEED_KEY) return true;
            if (c == SKIP_KEY || c == static_cast<char>(SKIP_KEY + 32)) {
                Serial.println(F("# → Step skipped."));
                return false;
            }
            if (c == ESTOP_KEY || c == static_cast<char>(ESTOP_KEY + 32)) {
                triggerEStop();
                return false;
            }
        }
    }
}

// ============================================================
//  LEAD-SCREW HELPERS
// ============================================================
static void lsMove(int stepPin, int dirPin, float mm, bool up, const char* name)
{
    if (stepPin < 0 || dirPin < 0) {
        Serial.print(F("# ")); Serial.print(name);
        Serial.println(F(": pin not set in PinMap.h — skipped"));
        return;
    }
    int32_t steps = static_cast<int32_t>(mm * LS_SPM + 0.5f);
    bool dirLevel = up ? (XFER_LS_DIR_UP == HIGH) : (XFER_LS_DIR_UP == LOW);
    digitalWrite(dirPin, dirLevel ? HIGH : LOW);
    delayMicroseconds(2);

    Serial.print(F("# ")); Serial.print(name);
    Serial.print(up ? F(" UP ") : F(" DOWN "));
    Serial.print(mm, 1); Serial.println(F(" mm"));

    for (int32_t s = 0; s < steps; s++) {
        if ((s & 0x3F) == 0 && checkEStop()) return;  // check every 64 steps
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(XFER_LS_PULSE_US);
        digitalWrite(stepPin, LOW);
        if (LS_STEP_DELAY_US > XFER_LS_PULSE_US)
            delayMicroseconds(LS_STEP_DELAY_US - XFER_LS_PULSE_US);
    }
    Serial.println(F("# Done."));
}

// ============================================================
//  SERVO HELPERS  (MG996R)
// ============================================================
static void servoSet(uint8_t idx, float deg)
{
    if (!g_servoAttached[idx]) {
        Serial.print(F("# Servo ")); Serial.print(idx + 1);
        Serial.println(F(" not attached — pin not set"));
        return;
    }
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    g_servoAngle[idx] = deg;
    g_servo[idx].write(static_cast<int>(deg));
    Serial.print(F("# Servo ")); Serial.print(idx + 1);
    Serial.print(F(" → ")); Serial.print(deg, 1); Serial.println(F("°"));
    // No blocking delay here — matches test_demo behaviour.
    // Callers that need settle time add delay() explicitly.
}

// ============================================================
//  STS3215 HELPERS  (direct protocol, no ServoController class)
// ============================================================
static uint8_t _stsCsum(uint8_t id, uint8_t len, uint8_t instr,
                         const uint8_t* p, uint8_t pLen)
{
    uint8_t s = id + len + instr;
    for (uint8_t i = 0; i < pLen; i++) s += p[i];
    return ~s & 0xFF;
}

static void _stsSend(uint8_t id, uint8_t instr, const uint8_t* p, uint8_t pLen)
{
    uint8_t len  = pLen + 2;
    uint8_t csum = _stsCsum(id, len, instr, p, pLen);
    while (Serial7.available()) Serial7.read();   // flush RX echo
    Serial7.write(STS_HDR); Serial7.write(STS_HDR);
    Serial7.write(id);      Serial7.write(len);
    Serial7.write(instr);
    for (uint8_t i = 0; i < pLen; i++) Serial7.write(p[i]);
    Serial7.write(csum);
    Serial7.flush();
    // Discard loopback echo
    uint32_t t0 = millis();
    for (uint8_t n = 0; n < 6 + pLen; ) {
        if (millis() - t0 > 3) break;
        if (Serial7.available()) { Serial7.read(); n++; }
    }
}

// Move STS3215 by deltaDeg (positive = forward). Tracks multi-turn position.
static void stsMoveBy(float deltaDeg)
{
    if (checkEStop()) return;
    int32_t delta     = static_cast<int32_t>(deltaDeg * STS_CPD);
    g_stsAbsPos      += delta;
    uint16_t pos      = static_cast<uint16_t>(((g_stsAbsPos % 4096) + 4096) % 4096);
    uint8_t p[5] = {
        STS_REG_GOALPOS,
        static_cast<uint8_t>(pos              & 0xFF),
        static_cast<uint8_t>(pos              >> 8),
        static_cast<uint8_t>(XFER_STS_SPEED   & 0xFF),
        static_cast<uint8_t>(XFER_STS_SPEED   >> 8)
    };
    _stsSend(XFER_STS_ID, STS_INSTR_WRITE, p, 5);
    Serial.print(F("# STS3215 Δ")); Serial.print(deltaDeg, 1);
    Serial.print(F("°  total=")); Serial.print(static_cast<float>(g_stsAbsPos) / STS_CPD, 1);
    Serial.println(F("°"));
    delay(XFER_STS_SETTLE_MS);
}

// ============================================================
//  DC MOTOR HELPERS
// ============================================================
static void dcRun(bool motorA, bool forward, uint8_t spd)
{
    if (motorA) {
        if (Pins::DCMotors::MOTOR_A_IN1 < 0) return;
        digitalWrite(Pins::DCMotors::MOTOR_A_IN1, forward ? HIGH : LOW);
        digitalWrite(Pins::DCMotors::MOTOR_A_IN2, forward ? LOW  : HIGH);
        if (Pins::DCMotors::MOTOR_A_ENA >= 0) analogWrite(Pins::DCMotors::MOTOR_A_ENA, spd);
    } else {
        if (Pins::DCMotors::MOTOR_B_IN3 < 0) return;
        digitalWrite(Pins::DCMotors::MOTOR_B_IN3, forward ? HIGH : LOW);
        digitalWrite(Pins::DCMotors::MOTOR_B_IN4, forward ? LOW  : HIGH);
        if (Pins::DCMotors::MOTOR_B_ENB >= 0) analogWrite(Pins::DCMotors::MOTOR_B_ENB, spd);
    }
}

static void dcCoast(bool motorA)
{
    if (motorA) {
        if (Pins::DCMotors::MOTOR_A_IN1 < 0) return;
        digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW);
        digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW);
        if (Pins::DCMotors::MOTOR_A_ENA >= 0) analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0);
    } else {
        if (Pins::DCMotors::MOTOR_B_IN3 < 0) return;
        digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW);
        digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW);
        if (Pins::DCMotors::MOTOR_B_ENB >= 0) analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0);
    }
}

// ============================================================
//  CAMERA / TSL1401 HELPERS
// ============================================================
static bool camPinsOk()
{
    return (Pins::LaserSensor::SI  >= 0 &&
            Pins::LaserSensor::CLK >= 0 &&
            Pins::LaserSensor::AO  >= 0);
}

static void camCapture()
{
    if (!camPinsOk()) return;
    if (Pins::LineLaser::ENABLE >= 0) digitalWrite(Pins::LineLaser::ENABLE, HIGH);

    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(XFER_CAM_CLK_HALF_US);
    digitalWrite(Pins::LaserSensor::SI,  HIGH);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(XFER_CAM_CLK_HALF_US);
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(XFER_CAM_CLK_HALF_US);

    for (uint8_t i = 0; i < 128; i++) {
        digitalWrite(Pins::LaserSensor::CLK, LOW);
        delayMicroseconds(XFER_CAM_CLK_HALF_US);
        g_camPixels[i] = static_cast<uint16_t>(analogRead(Pins::LaserSensor::AO));
        digitalWrite(Pins::LaserSensor::CLK, HIGH);
        delayMicroseconds(XFER_CAM_CLK_HALF_US);
    }

    if (Pins::LineLaser::ENABLE >= 0) digitalWrite(Pins::LineLaser::ENABLE, LOW);
}

// Write accumulated scan frames to Serial as CSV.
// A Python script can capture this and plot it.
static void writeScanCSV()
{
    Serial.println(F("# === SCAN_CSV_START ==="));
    Serial.println(F("y_mm,center_px,width_px,detected"));
    for (uint16_t i = 0; i < g_scanFrameCount; i++) {
        Serial.print(g_scanFrames[i].y_mm, 3);    Serial.print(',');
        Serial.print(g_scanFrames[i].centerPx);   Serial.print(',');
        Serial.print(g_scanFrames[i].widthPx);    Serial.print(',');
        Serial.println(g_scanFrames[i].detected ? 1 : 0);
    }
    Serial.println(F("# === SCAN_CSV_END ==="));
}

// ============================================================
//  GANTRY MOTION  (open-loop, CoreXY, step-counted)
//
//  CoreXY kinematics:
//    +Y  →  Motor A forward,  Motor B backward
//    -Y  →  Motor A backward, Motor B forward
//    +X  →  Motor A forward,  Motor B forward
//    -X  →  Motor A backward, Motor B backward
// ============================================================

// Fire one simultaneous step pulse on both motors.
// Directions must be set before calling this.
static inline void _gantryPulse()
{
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, HIGH);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, HIGH);
    delayMicroseconds(XFER_G_PULSE_US);
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);
}

// Move in pure Y. Positive = +Y direction.
// Optionally calls the camera capture every XFER_SCAN_SAMPLE_N steps.
static void gantryMoveY(float dy_mm,
                        bool scanCallus = false,
                        bool scanDish   = false)
{
    if (checkEStop() || dy_mm == 0.0f) return;

    bool posY = (dy_mm > 0.0f);
    int32_t steps = static_cast<int32_t>(fabsf(dy_mm) * XFER_G_STEPS_PER_MM + 0.5f);

    // +Y: A fwd, B bwd  |  -Y: A bwd, B fwd
    digitalWrite(Pins::Gantry::MOTOR_A_DIR, posY ? XFER_G_DIR_A_FWD : XFER_G_DIR_A_BWD);
    digitalWrite(Pins::Gantry::MOTOR_B_DIR, posY ? XFER_G_DIR_B_BWD : XFER_G_DIR_B_FWD);
    delayMicroseconds(2);

    for (int32_t s = 0; s < steps; s++) {
        // E-stop check — every 256 steps to keep overhead low
        if ((s & 0xFF) == 0 && checkEStop()) return;

        _gantryPulse();

        if (XFER_G_STEP_US > XFER_G_PULSE_US)
            delayMicroseconds(XFER_G_STEP_US - XFER_G_PULSE_US);

        // Camera scan: capture at intervals
        if ((scanCallus || scanDish) && ((s % XFER_SCAN_SAMPLE_N) == 0)) {
            float currentY = g_posY_mm + (posY ? 1.0f : -1.0f)
                             * (static_cast<float>(s) / XFER_G_STEPS_PER_MM);
            camCapture();
            CallusDetect::update(g_camPixels, currentY, scanCallus, scanDish);

            // Log frame
            if (g_scanFrameCount < MAX_SCAN_FRAMES) {
                // Quick centre/width extraction for CSV (reuse CallusDetect internals
                // indirectly — just store pixel 64 and neighbour range as proxy)
                uint16_t maxV = g_camPixels[5] >> 2, minV = maxV;
                for (uint8_t px = 5; px < 123; px++) {
                    uint16_t v = g_camPixels[px] >> 2;
                    if (v > maxV) maxV = v;
                    if (v < minV) minV = v;
                }
                bool det = (maxV - minV) >= 10;
                g_scanFrames[g_scanFrameCount++] = {
                    currentY,
                    static_cast<uint8_t>(64),   // placeholder — CallusDetect has the real centre
                    static_cast<uint8_t>(det ? (maxV - minV) / 4 : 0),
                    det
                };
            }
        }
    }

    g_posY_mm += dy_mm;
}

// Move in pure X. Positive = +X direction.
static void gantryMoveX(float dx_mm)
{
    if (checkEStop() || dx_mm == 0.0f) return;

    bool posX = (dx_mm > 0.0f);
    int32_t steps = static_cast<int32_t>(fabsf(dx_mm) * XFER_G_STEPS_PER_MM + 0.5f);

    // +X: A fwd, B fwd  |  -X: A bwd, B bwd
    digitalWrite(Pins::Gantry::MOTOR_A_DIR, posX ? XFER_G_DIR_A_FWD : XFER_G_DIR_A_BWD);
    digitalWrite(Pins::Gantry::MOTOR_B_DIR, posX ? XFER_G_DIR_B_FWD : XFER_G_DIR_B_BWD);
    delayMicroseconds(2);

    for (int32_t s = 0; s < steps; s++) {
        if ((s & 0xFF) == 0 && checkEStop()) return;
        _gantryPulse();
        if (XFER_G_STEP_US > XFER_G_PULSE_US)
            delayMicroseconds(XFER_G_STEP_US - XFER_G_PULSE_US);
    }

    g_posX_mm += dx_mm;
}

// Return gantry to (0, 0) reference in two straight moves (Y first, then X).
static void gantryReturnToOrigin()
{
    if (checkEStop()) return;
    Serial.println(F("# Returning to origin (0, 0)..."));
    gantryMoveY(-g_posY_mm);
    gantryMoveX(-g_posX_mm);
    if (!g_eStopped) {
        Serial.println(F("# At origin."));
    }
}

// ============================================================
//  STEP 1 — Lift D1-S1 then D1-S2
// ============================================================
static void step1_liftScrews()
{
    Serial.println(F("# ── STEP 1: Lift Device 1 lead screws ──────────────"));
    if (!waitForSpace(F("Ready to lift D1-S1?"))) return;
    lsMove(Pins::LeadScrews::DEVICE1_SCREW1_STEP,
           Pins::LeadScrews::DEVICE1_SCREW1_DIR,
           XFER_LS_LIFT_MM, true, "D1-S1");
    if (g_eStopped) return;

    if (!waitForSpace(F("Ready to lift D1-S2?"))) return;
    lsMove(Pins::LeadScrews::DEVICE1_SCREW2_STEP,
           Pins::LeadScrews::DEVICE1_SCREW2_DIR,
           XFER_LS_LIFT_MM, true, "D1-S2");
    if (g_eStopped) return;
    Serial.println(F("# STEP 1 complete."));
}

// ============================================================
//  STEP 2 — Actuate MG996R servos 1 & 2
// ============================================================
static void step2_actuateServos()
{
    Serial.println(F("# ── STEP 2: Actuate servos 1 & 2 ───────────────────"));
    if (!waitForSpace(F("Ready to move servo 1?"))) return;
    servoSet(0, XFER_S1_TARGET_DEG);
    delay(XFER_SERVO_SETTLE_MS);        // give servo time to reach target
    if (g_eStopped) return;
    if (!waitForSpace(F("Servo 1 at target — SPACE to reset to initial angle"))) return;
    servoSet(0, XFER_S1_INITIAL_DEG);
    delay(XFER_SERVO_SETTLE_MS);
    if (g_eStopped) return;

    if (!waitForSpace(F("Ready to move servo 2?"))) return;
    servoSet(1, XFER_S2_TARGET_DEG);
    delay(XFER_SERVO_SETTLE_MS);        // give servo time to reach target
    if (g_eStopped) return;
    if (!waitForSpace(F("Servo 2 at target — SPACE to reset to initial angle"))) return;
    servoSet(1, XFER_S2_INITIAL_DEG);
    delay(XFER_SERVO_SETTLE_MS);
    if (g_eStopped) return;
    Serial.println(F("# STEP 2 complete."));
}

// ============================================================
//  STEP 3 / STEP 9 — DC motors (shared, used in both steps)
// ============================================================
static void runDCMotors(const __FlashStringHelper* stepName)
{
    Serial.print(F("# ── ")); Serial.print(stepName);
    Serial.println(F(": DC Motors ─────────────────────────────"));

    // Motor A
    Serial.print(F("# DC Motor A running @ speed ")); Serial.print(XFER_DC_SPEED);
    Serial.println(F(".  Press SPACE to stop."));
    dcRun(true, true, XFER_DC_SPEED);
    if (!waitForSpace()) { dcCoast(true); dcCoast(false); return; }
    dcCoast(true);
    Serial.print(F("# Motor A stopped. Motor B starts in "));
    Serial.print(XFER_DC_DELAY_MS); Serial.println(F(" ms..."));

    // Inter-motor delay
    uint32_t t0 = millis();
    while (millis() - t0 < XFER_DC_DELAY_MS) {
        if (checkEStop()) { dcCoast(false); return; }
    }

    // Motor B
    Serial.print(F("# DC Motor B running @ speed ")); Serial.print(XFER_DC_SPEED);
    Serial.println(F(".  Press SPACE to stop."));
    dcRun(false, true, XFER_DC_SPEED);
    if (!waitForSpace()) { dcCoast(false); return; }
    dcCoast(false);
    Serial.println(F("# DC motors done."));
}

// ============================================================
//  STEP 4 — Set gantry reference (0, 0) via WASD jog
// ============================================================
static void step4_setReference()
{
    Serial.println(F("# ── STEP 4: Set gantry reference (0, 0) ────────────"));
    Serial.println(F("#  W/S = +Y / -Y    A/D = -X / +X"));
    Serial.println(F("#  +/- = jog step size    C = confirm (set this pos as origin)"));
    Serial.print(F("#  Q = E-stop   (current jog step: "));
    Serial.print(XFER_JOG_STEP_MM, 1); Serial.println(F(" mm)"));

    float jogMm = XFER_JOG_STEP_MM;

    while (true) {
        if (checkEStop()) return;
        if (!Serial.available()) continue;

        char c = static_cast<char>(Serial.read());
        switch (c) {
            case 'w': case 'W': gantryMoveY( jogMm); break;
            case 's': case 'S': gantryMoveY(-jogMm); break;
            case 'a': case 'A': gantryMoveX(-jogMm); break;
            case 'd': case 'D': gantryMoveX( jogMm); break;
            case '+': case '=':
                if (jogMm < 50.0f) jogMm += 1.0f;
                Serial.print(F("# Jog step: ")); Serial.print(jogMm, 1); Serial.println(F(" mm"));
                break;
            case '-': case '_':
                if (jogMm > 0.5f) jogMm -= 1.0f;
                Serial.print(F("# Jog step: ")); Serial.print(jogMm, 1); Serial.println(F(" mm"));
                break;
            case 'c': case 'C':
                g_posX_mm = 0.0f;
                g_posY_mm = 0.0f;
                Serial.println(F("# Origin set. Current position is now (0, 0)."));
                Serial.println(F("# STEP 4 complete."));
                return;
            case 'q': case 'Q':
                triggerEStop();
                return;
            case '?':
                Serial.println(F("# W/S/A/D = jog  +/- = step size  C = confirm  Q = e-stop"));
                break;
            default: break;
        }
        if (g_eStopped) return;
    }
}

// ============================================================
//  STEP 5 — Scan both dishes
// ============================================================
static void step5_scan()
{
    Serial.println(F("# ── STEP 5: Gantry scan ─────────────────────────────"));

    if (!camPinsOk()) {
        Serial.println(F("# WARNING: Camera pins not set — scan will have no data."));
    }

    // ---- Dish 1: +Y scan (calluses + outline) ----
    CallusDetect::reset();
    g_scanFrameCount = 0;
    Serial.print(F("# Scanning dish 1: +Y ")); Serial.print(XFER_SCAN_Y_MM, 0); Serial.println(F(" mm"));
    gantryMoveY(XFER_SCAN_Y_MM, true, true);
    if (g_eStopped) return;

    g_dish1 = CallusDetect::getDishInfo();
    g_callusCount = CallusDetect::getCallusCount();
    const CallusDetect::CallusPos* raw = CallusDetect::getCalluses();
    for (uint8_t i = 0; i < g_callusCount; i++) g_calluses[i] = raw[i];
    CallusDetect::printResults();

    // Return to Y=0
    Serial.println(F("# Returning to Y = 0..."));
    gantryMoveY(-g_posY_mm);
    if (g_eStopped) return;

    // ---- Shift to dish 2 ----
    Serial.print(F("# Shifting +X to dish 2 (")); Serial.print(XFER_DISH_X_OFFSET_MM, 0); Serial.println(F(" mm)"));
    gantryMoveX(XFER_DISH_X_OFFSET_MM);
    if (g_eStopped) return;

    // ---- Dish 2: +Y scan (outline only, to find center) ----
    CallusDetect::reset();
    Serial.print(F("# Scanning dish 2: +Y ")); Serial.print(XFER_SCAN_Y_MM, 0); Serial.println(F(" mm (outline only)"));
    gantryMoveY(XFER_SCAN_Y_MM, false, true);
    if (g_eStopped) return;

    g_dish2 = CallusDetect::getDishInfo();
    // Convert dish 2 sensor-relative X to global frame
    g_dish2GlobalX = g_posX_mm + g_dish2.center_x_mm;
    g_dish2GlobalY = g_dish2.center_y_mm;
    CallusDetect::printResults();
    Serial.print(F("# Dish 2 global center: (")); Serial.print(g_dish2GlobalX, 1);
    Serial.print(F(", ")); Serial.print(g_dish2GlobalY, 1); Serial.println(F(")"));

    // Return to Y=0, then X=0
    Serial.println(F("# Returning to origin..."));
    gantryMoveY(-g_posY_mm);
    if (g_eStopped) return;
    gantryMoveX(-g_posX_mm);
    if (g_eStopped) return;

    // Write scan CSV for Python plotting / manual confirmation
    writeScanCSV();
    Serial.println(F("# STEP 5 complete."));
}

// ============================================================
//  STEP 6 — Tool operations on each dish
// ============================================================
static void step6_toolOperations()
{
    Serial.println(F("# ── STEP 6: Tool operations ─────────────────────────"));
    if (!waitForSpace(F("Ready to begin tool operations?"))) return;

    // Dish 1 center (in global frame, sensor X offset + gantry was at 0)
    float d1cx = g_dish1.valid ? g_dish1.center_x_mm : 0.0f;
    float d1cy = g_dish1.valid ? g_dish1.center_y_mm : 0.0f;

    // ---- Move to dish 1 ----
    Serial.println(F("# Moving to dish 1 center..."));
    gantryMoveX((d1cx + XFER_PICKUP_X_OFFSET_MM) - g_posX_mm);
    if (g_eStopped) return;
    gantryMoveY((d1cy + XFER_PICKUP_Y_OFFSET_MM) - g_posY_mm);
    if (g_eStopped) return;

    // Raise tool
    Serial.println(F("# Raising tool over dish 1..."));
    stsMoveBy(XFER_TOOL_RAISE_DEG);
    if (g_eStopped) return;

    // Actuate MG996R #3 (syringe engage)
    servoSet(2, XFER_MG3_ENGAGE_DEG);
    if (g_eStopped) return;

    // Lower tool onto dish 1
    stsMoveBy(-XFER_TOOL_LOWER_DEG);
    if (g_eStopped) return;

    // Shift -Y then raise
    gantryMoveY(-XFER_PICKUP_Y_DELTA_MM);
    if (g_eStopped) return;
    stsMoveBy(XFER_TOOL_RAISE_DEG);
    if (g_eStopped) return;

    // Release MG996R #3
    servoSet(2, XFER_MG3_RELEASE_DEG);
    if (g_eStopped) return;

    // Lower tool
    stsMoveBy(-XFER_TOOL_LOWER_DEG);
    if (g_eStopped) return;

    // ---- Move to dish 2 ----
    Serial.println(F("# Moving to dish 2 center..."));
    gantryMoveX((g_dish2GlobalX + XFER_PICKUP_X_SHIFT_D2) - g_posX_mm);
    if (g_eStopped) return;
    gantryMoveY(g_dish2GlobalY - g_posY_mm);
    if (g_eStopped) return;

    // Repeat raise/engage/lower/shift/raise/release/lower for dish 2
    stsMoveBy(XFER_TOOL_RAISE_DEG);
    if (g_eStopped) return;
    servoSet(2, XFER_MG3_ENGAGE_DEG);
    if (g_eStopped) return;
    stsMoveBy(-XFER_TOOL_LOWER_DEG);
    if (g_eStopped) return;
    gantryMoveY(-XFER_PICKUP_Y_DELTA_MM);
    if (g_eStopped) return;
    stsMoveBy(XFER_TOOL_RAISE_DEG);
    if (g_eStopped) return;
    servoSet(2, XFER_MG3_RELEASE_DEG);
    if (g_eStopped) return;
    stsMoveBy(-XFER_TOOL_LOWER_DEG);
    if (g_eStopped) return;

    gantryReturnToOrigin();
    if (g_eStopped) return;
    Serial.println(F("# STEP 6 complete."));
}

// ============================================================
//  STEP 7 — Calculate 5×6 grid on dish 2
// ============================================================
static void step7_calculateGrid()
{
    Serial.println(F("# ── STEP 7: Calculate transfer grid ─────────────────"));

    g_gridValid = false;

    if (!g_dish2.valid) {
        Serial.println(F("# WARNING: Dish 2 not detected — using (0,0) with r=50mm fallback."));
        g_dish2GlobalX = XFER_DISH_X_OFFSET_MM;
        g_dish2GlobalY = XFER_SCAN_Y_MM * 0.5f;
        g_dish2.radius_mm = 50.0f;
    }

    float r = g_dish2.radius_mm;
    // Available span inside margin
    float span = 2.0f * (r - GRID_MARGIN_MM);

    if (span <= 0.0f) {
        Serial.println(F("# ERROR: Dish radius too small for grid with configured margin."));
        Serial.println(F("#   Reduce GRID_MARGIN_MM or verify dish detection."));
        return;
    }

    float colSpacing = span / static_cast<float>(GRID_COLS - 1);
    float rowSpacing = span / static_cast<float>(GRID_ROWS - 1);
    float originX    = g_dish2GlobalX - span * 0.5f;
    float originY    = g_dish2GlobalY - span * 0.5f;

    Serial.println(F("# Grid positions (global frame):"));
    for (uint8_t row = 0; row < GRID_ROWS; row++) {
        for (uint8_t col = 0; col < GRID_COLS; col++) {
            g_gridX[row][col] = originX + col * colSpacing;
            g_gridY[row][col] = originY + row * rowSpacing;
        }
    }

    // Print grid
    for (uint8_t row = 0; row < GRID_ROWS; row++) {
        for (uint8_t col = 0; col < GRID_COLS; col++) {
            Serial.print(F("#  [")); Serial.print(row); Serial.print(F("]["));
            Serial.print(col); Serial.print(F("] ("));
            Serial.print(g_gridX[row][col], 1); Serial.print(F(", "));
            Serial.print(g_gridY[row][col], 1); Serial.print(F(")"));
        }
        Serial.println();
    }

    Serial.print(F("# Column spacing: ")); Serial.print(colSpacing, 2); Serial.println(F(" mm"));
    Serial.print(F("# Row spacing:    ")); Serial.print(rowSpacing, 2); Serial.println(F(" mm"));
    Serial.print(F("# Dish radius:    ")); Serial.print(r, 1); Serial.println(F(" mm"));
    Serial.print(F("# Margin:         ")); Serial.print(GRID_MARGIN_MM, 1); Serial.println(F(" mm"));

    g_gridValid = true;
    Serial.println(F("# STEP 7 complete."));
}

// ============================================================
//  STEP 8 — Transfer calluses from dish 1 to grid on dish 2
// ============================================================
static void step8_transferCalluses()
{
    Serial.println(F("# ── STEP 8: Transfer calluses ───────────────────────"));

    if (g_callusCount == 0) {
        Serial.println(F("# No calluses detected in step 5 — skipping transfer."));
        return;
    }
    if (!g_gridValid) {
        Serial.println(F("# Grid not valid (step 7 failed) — skipping transfer."));
        return;
    }

    if (!waitForSpace(F("Ready to begin callus transfer?"))) return;

    uint8_t gridIdx = 0;
    uint8_t maxGrid = static_cast<uint8_t>(GRID_ROWS * GRID_COLS);

    for (uint8_t ci = 0; ci < g_callusCount && gridIdx < maxGrid; ci++) {
        if (g_eStopped) return;

        uint8_t row = gridIdx / GRID_COLS;
        uint8_t col = gridIdx % GRID_COLS;

        Serial.print(F("# Transfer callus ")); Serial.print(ci);
        Serial.print(F(" → grid [")); Serial.print(row); Serial.print(F("]["));
        Serial.print(col); Serial.println(F("]"));

        // ---- Pick up from dish 1 callus position ----
        float targetX = g_calluses[ci].x_mm + XFER_CALLUS_X_OFFSET_MM;
        float targetY = g_calluses[ci].y_mm + XFER_CALLUS_Y_OFFSET_MM;
        gantryMoveX(targetX - g_posX_mm);
        if (g_eStopped) return;
        gantryMoveY(targetY - g_posY_mm);
        if (g_eStopped) return;

        // Lower tool onto callus
        stsMoveBy(-XFER_TOOL_LOWER_DEG);
        if (g_eStopped) return;

        // Clamp tweezers (MG996R #4)
        servoSet(3, XFER_MG4_CLAMP_DEG);
        if (g_eStopped) return;

        // Raise tool with callus
        stsMoveBy(XFER_TOOL_RAISE_DEG);
        if (g_eStopped) return;

        // ---- Deposit at grid position on dish 2 ----
        float gridTargetX = g_gridX[row][col] + XFER_GRID_X_OFFSET_MM;
        float gridTargetY = g_gridY[row][col] + XFER_GRID_Y_OFFSET_MM;
        gantryMoveX(gridTargetX - g_posX_mm);
        if (g_eStopped) return;
        gantryMoveY(gridTargetY - g_posY_mm);
        if (g_eStopped) return;

        // Lower tool
        stsMoveBy(-XFER_TOOL_LOWER_DEG);
        if (g_eStopped) return;

        // Release tweezers
        servoSet(3, XFER_MG4_RELEASE_DEG);
        if (g_eStopped) return;

        // Raise tool
        stsMoveBy(XFER_TOOL_RAISE_DEG);
        if (g_eStopped) return;

        gridIdx++;
    }

    if (gridIdx < g_callusCount) {
        Serial.print(F("# NOTE: ")); Serial.print(g_callusCount - gridIdx);
        Serial.println(F(" callus(es) not transferred — grid is full."));
    }

    // Return to origin with tool raised
    Serial.println(F("# Transfer complete. Returning to origin."));
    gantryReturnToOrigin();
    Serial.println(F("# STEP 8 complete."));
}

// ============================================================
//  STEP 10 — Lower D2-S1 then D2-S2
// ============================================================
static void step10_lowerScrews()
{
    Serial.println(F("# ── STEP 10: Lower Device 2 lead screws ────────────"));

    if (!waitForSpace(F("Ready to lower D2-S1?"))) return;
    lsMove(Pins::LeadScrews::DEVICE2_SCREW1_STEP,
           Pins::LeadScrews::DEVICE2_SCREW1_DIR,
           XFER_LS_LIFT_MM, false, "D2-S1");
    if (g_eStopped) return;

    if (!waitForSpace(F("Ready to lower D2-S2?"))) return;
    lsMove(Pins::LeadScrews::DEVICE2_SCREW2_STEP,
           Pins::LeadScrews::DEVICE2_SCREW2_DIR,
           XFER_LS_LIFT_MM, false, "D2-S2");
    if (g_eStopped) return;

    Serial.println(F("# STEP 10 complete."));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testTransfer_setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(200);

    // ---- Gantry step/dir pins ----
    if (Pins::Gantry::MOTOR_A_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_A_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW); }
    if (Pins::Gantry::MOTOR_A_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_A_DIR,  OUTPUT); }
    if (Pins::Gantry::MOTOR_B_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_B_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW); }
    if (Pins::Gantry::MOTOR_B_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_B_DIR,  OUTPUT); }

    // ---- Lead screw step/dir pins ----
    const int lsSteps[] = {
        Pins::LeadScrews::DEVICE1_SCREW1_STEP, Pins::LeadScrews::DEVICE1_SCREW2_STEP,
        Pins::LeadScrews::DEVICE2_SCREW1_STEP, Pins::LeadScrews::DEVICE2_SCREW2_STEP
    };
    const int lsDirs[] = {
        Pins::LeadScrews::DEVICE1_SCREW1_DIR,  Pins::LeadScrews::DEVICE1_SCREW2_DIR,
        Pins::LeadScrews::DEVICE2_SCREW1_DIR,  Pins::LeadScrews::DEVICE2_SCREW2_DIR
    };
    for (int i = 0; i < 4; i++) {
        if (lsSteps[i] >= 0) { pinMode(lsSteps[i], OUTPUT); digitalWrite(lsSteps[i], LOW); }
        if (lsDirs[i]  >= 0) { pinMode(lsDirs[i],  OUTPUT); }
    }

    // ---- DC motor pins ----
    if (Pins::DCMotors::MOTOR_A_IN1 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN1, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW); }
    if (Pins::DCMotors::MOTOR_A_IN2 >= 0) { pinMode(Pins::DCMotors::MOTOR_A_IN2, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW); }
    if (Pins::DCMotors::MOTOR_A_ENA >= 0) { pinMode(Pins::DCMotors::MOTOR_A_ENA, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0); }
    if (Pins::DCMotors::MOTOR_B_IN3 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN3, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW); }
    if (Pins::DCMotors::MOTOR_B_IN4 >= 0) { pinMode(Pins::DCMotors::MOTOR_B_IN4, OUTPUT); digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW); }
    if (Pins::DCMotors::MOTOR_B_ENB >= 0) { pinMode(Pins::DCMotors::MOTOR_B_ENB, OUTPUT); analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0); }

    // ---- MG996R servos ----
    const int servoPins[4] = {
        Pins::Servos::MG996R_1, Pins::Servos::MG996R_2,
        Pins::Servos::MG996R_3, Pins::Servos::MG996R_4
    };
    for (uint8_t i = 0; i < 4; i++) {
        if (servoPins[i] < 0) continue;
        g_servo[i].attach(servoPins[i], XFER_MG_MIN_US, XFER_MG_MAX_US);
        g_servoAttached[i] = true;
        g_servo[i].write(static_cast<int>(g_servoAngle[i]));
        Serial.print(F("# Servo ")); Serial.print(i + 1);
        Serial.print(F(" → ")); Serial.print(g_servoAngle[i], 1); Serial.println(F("° (initial)"));
    }

    // ---- Camera pins ----
    if (Pins::LaserSensor::CLK >= 0) { pinMode(Pins::LaserSensor::CLK, OUTPUT); digitalWrite(Pins::LaserSensor::CLK, LOW); }
    if (Pins::LaserSensor::SI  >= 0) { pinMode(Pins::LaserSensor::SI,  OUTPUT); digitalWrite(Pins::LaserSensor::SI,  LOW); }
    if (Pins::LineLaser::ENABLE >= 0){ pinMode(Pins::LineLaser::ENABLE, OUTPUT); digitalWrite(Pins::LineLaser::ENABLE, LOW); }
    // Prime TSL1401 pipeline
    if (camPinsOk()) camCapture();

    // ---- STS3215 Serial7 with loopback ----
    Serial7.begin(XFER_STS_BAUD);
    LPUART7_CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

    Serial.println(F("# ============================================="));
    Serial.println(F("# Transfer Process Test"));
    Serial.println(F("# ============================================="));
    Serial.print(F("# E-stop key: ")); Serial.println(ESTOP_KEY);
    Serial.println(F("# Proceed key: SPACE"));
    Serial.println(F("# Sequence will begin automatically."));
    Serial.println(F("# Press SPACE at each prompt to advance."));
    Serial.println(F("# ============================================="));
}

static bool g_sequenceDone = false;

void testTransfer_loop()
{
    if (g_eStopped || g_sequenceDone) return;

    step1_liftScrews();        if (g_eStopped) return;
    step2_actuateServos();     if (g_eStopped) return;
    runDCMotors(F("STEP 3"));  if (g_eStopped) return;
    step4_setReference();      if (g_eStopped) return;
    step5_scan();              if (g_eStopped) return;
    step6_toolOperations();    if (g_eStopped) return;
    step7_calculateGrid();     if (g_eStopped) return;
    step8_transferCalluses();  if (g_eStopped) return;
    runDCMotors(F("STEP 9"));  if (g_eStopped) return;
    step10_lowerScrews();      if (g_eStopped) return;

    g_sequenceDone = true;
    Serial.println(F("# ============================================="));
    Serial.println(F("# Transfer sequence complete."));
    Serial.println(F("# Re-upload firmware to run again."));
    Serial.println(F("# ============================================="));
}

#endif // TEST_MODE
