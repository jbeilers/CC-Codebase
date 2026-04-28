// ============================================================
//  testing/test_tsl1401.cpp
//
//  PURPOSE
//  -------
//  Bring-up test for the TSL1401 128-pixel linear CCD sensor
//  and the 5V line laser module.
//
//  The line laser is turned ON immediately before each CCD
//  capture and OFF immediately after, matching the intended
//  production behaviour.
//
//  SETUP — fill in PinMap.h first
//  --------------------------------
//  In include/pins/PinMap.h:
//
//    Pins::LaserSensor::SI   — Serial Input (start-of-scan pulse)  → digital out
//    Pins::LaserSensor::CLK  — Clock                               → digital out
//    Pins::LaserSensor::AO   — Analog Out (pixel voltage)          → analog in
//                              (already set to pin 14 / A0)
//
//    Pins::LineLaser::ENABLE — Laser on/off                        → digital out
//
//  TSL1401 wiring (5-pin XH2.54 connector, left to right):
//    GND  — Teensy GND
//    VCC  — 3.3 V or 5 V (module is 3.3–5 V tolerant)
//    AO   — Analog input pin  (Pins::LaserSensor::AO)
//    CLK  — Digital output pin (Pins::LaserSensor::CLK)
//    SI   — Digital output pin (Pins::LaserSensor::SI)
//
//  Line laser wiring:
//    GND  — Teensy GND
//    VCC  — 5 V
//    IN   — Pins::LineLaser::ENABLE  (digital out, HIGH = on)
//
//  COMMANDS (single keypress, no newline needed)
//  ---------------------------------------------
//    R    — Read one frame, print raw pixel CSV (128 values)
//    C    — Toggle continuous mode (capture + detect centre ~10 Hz)
//    W    — Print ASCII waveform of last captured frame
//    +/-  — Increase / decrease CLK half-period (sensitivity)
//    L    — Toggle laser manually (for wiring verification)
//    ?    — Print this help
//
//  CONTINUOUS MODE OUTPUT
//  ----------------------
//  Each line:  frame_ms, centre_pixel, threshold, detected (0/1)
//  Import into a spreadsheet or serial plotter to verify tracking.
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testTSL1401_setup();
//    extern void testTSL1401_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  Tuning parameters
// ============================================================

// CLK half-period in microseconds.
// Longer = more light integration = more sensitive in dim conditions.
// Docs use 10 µs (100 kHz).  Safe range: 5–100 µs.
static uint32_t g_clkHalfUs = 10;
static constexpr uint32_t CLK_STEP_US  = 5;
static constexpr uint32_t CLK_MIN_US   = 5;
static constexpr uint32_t CLK_MAX_US   = 100;

// Pixels to skip on each edge (border noise rejection)
static constexpr uint8_t EDGE_SKIP    = 5;

// Consecutive pixels required to confirm a threshold crossing
static constexpr uint8_t EDGE_CONFIRM = 3;

// Maximum plausible centre jump between frames before discarding result
static constexpr uint8_t JUMP_LIMIT   = 70;

// Minimum pixel contrast (max - min) to consider a valid target present
static constexpr uint16_t MIN_CONTRAST = 10;

// Number of pixels in the array
static constexpr uint8_t PIXEL_COUNT  = 128;

// ============================================================
//  State
// ============================================================
static uint16_t g_pixels[PIXEL_COUNT] = {0};
static uint8_t  g_lastCenter          = 64;
static uint8_t  g_leftEdge            = 0;    // Last valid left edge pixel
static uint8_t  g_rightEdge           = 127;  // Last valid right edge pixel
static uint16_t g_threshold           = 0;
static bool     g_detected            = false;
static bool     g_continuous          = false;
static bool     g_streaming           = false; // Full-frame stream mode for Python plotter
static bool     g_laserOn             = false;

// ============================================================
//  Pin guard — true when minimum required pins are set
// ============================================================
static bool pinsAssigned() {
    return (Pins::LaserSensor::SI  >= 0 &&
            Pins::LaserSensor::CLK >= 0 &&
            Pins::LaserSensor::AO  >= 0);
}

// ============================================================
//  Line laser helpers — direct pin drive (no MachineState)
// ============================================================
static void laserOn() {
    if (Pins::LineLaser::ENABLE < 0) return;
    digitalWrite(Pins::LineLaser::ENABLE, HIGH);
    g_laserOn = true;
}

static void laserOff() {
    if (Pins::LineLaser::ENABLE < 0) return;
    digitalWrite(Pins::LineLaser::ENABLE, LOW);
    g_laserOn = false;
}

// ============================================================
//  TSL1401 capture sequence (direct pin drive, no class/MachineState)
//
//  Timing per datasheet and reference docs:
//    1. CLK=HIGH, SI=LOW     (starting state)
//    2. SI=HIGH,  CLK=LOW    (SI must be high for the first falling CLK edge)
//    3. CLK=HIGH, SI=LOW     (rising CLK edge with SI high starts integration cycle)
//    4. For each of 128 pixels:
//         CLK=LOW  → AO becomes valid (pixel output on falling edge)
//         Sample AO via analogRead
//         CLK=HIGH
//
//  AO outputs pixel data from the PREVIOUS integration cycle.
//  The dummy capture in setup() flushes stale data.
//
//  Laser is activated before the sequence and deactivated after.
// ============================================================
static void capture() {
    if (!pinsAssigned()) return;

    laserOn();

    // Step 1
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(g_clkHalfUs);

    // Step 2
    digitalWrite(Pins::LaserSensor::SI,  HIGH);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(g_clkHalfUs);

    // Step 3
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(g_clkHalfUs);

    // Step 4: read all 128 pixels
    for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
        digitalWrite(Pins::LaserSensor::CLK, LOW);
        delayMicroseconds(g_clkHalfUs);   // AO settles after falling CLK edge
        g_pixels[i] = static_cast<uint16_t>(analogRead(Pins::LaserSensor::AO));
        digitalWrite(Pins::LaserSensor::CLK, HIGH);
        delayMicroseconds(g_clkHalfUs);
    }

    laserOff();
}

// ============================================================
//  Dynamic threshold centre-finding algorithm.
//
//  From TSL1401 reference docs (Yahboom):
//    1. Find min + max in the valid pixel window (EDGE_SKIP..122).
//    2. threshold = (max + min) / 2.
//    3. Left edge:  scan L→R for EDGE_CONFIRM pixels above threshold
//       immediately followed by EDGE_CONFIRM pixels below.
//    4. Right edge: scan R→L for EDGE_CONFIRM pixels below threshold
//       immediately followed (to the right) by EDGE_CONFIRM above.
//    5. centre = (leftEdge + rightEdge) / 2.
//    6. Reject if jump > JUMP_LIMIT.
//
//  The TSL1401 outputs HIGH voltage for bright (unobstructed) pixels
//  and LOW for dark (shadowed) pixels, so the laser line / object
//  appears as a dip in the waveform.
// ============================================================
static uint8_t findCenter() {
    // ---- 1. Find min / max in valid window ----
    uint16_t maxVal = g_pixels[EDGE_SKIP] >> 2;   // 10-bit → 8-bit
    uint16_t minVal = maxVal;

    for (uint8_t i = EDGE_SKIP; i < (PIXEL_COUNT - EDGE_SKIP); i++) {
        uint16_t v = g_pixels[i] >> 2;
        if (v > maxVal) maxVal = v;
        if (v < minVal) minVal = v;
    }

    // ---- 2. Dynamic threshold ----
    g_threshold = (maxVal + minVal) / 2;

    // Not enough contrast — no valid target
    if ((maxVal - minVal) < MIN_CONTRAST) {
        g_detected = false;
        return g_lastCenter;
    }

    // ---- 3. Left edge (ABOVE → BELOW) ----
    uint8_t leftEdge  = 0;
    bool    leftFound = false;

    for (uint8_t i = EDGE_SKIP; i < (PIXEL_COUNT - EDGE_SKIP - (EDGE_CONFIRM * 2)); i++) {
        bool above = true, below = true;
        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if ((g_pixels[i + k]             >> 2) <= g_threshold) above = false;
            if ((g_pixels[i + EDGE_CONFIRM + k] >> 2) >  g_threshold) below = false;
        }
        if (above && below) { leftEdge = i; leftFound = true; break; }
    }

    // ---- 4. Right edge (BELOW → ABOVE) ----
    uint8_t rightEdge = PIXEL_COUNT - 1;
    bool    rightFound = false;

    for (int16_t j = (PIXEL_COUNT - EDGE_SKIP - 1);
         j >= (EDGE_SKIP + static_cast<int16_t>(EDGE_CONFIRM * 2));
         j--) {
        bool below = true, above = true;
        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if ((g_pixels[j - k]                    >> 2) >= g_threshold) below = false;
            if ((g_pixels[j + EDGE_CONFIRM - k]     >> 2) <  g_threshold) above = false;
        }
        if (below && above) { rightEdge = static_cast<uint8_t>(j); rightFound = true; break; }
    }

    // ---- 5. Validate ----
    if (!leftFound || !rightFound || rightEdge <= leftEdge) {
        g_detected = false;
        return g_lastCenter;
    }

    uint8_t newCenter = (leftEdge + rightEdge) / 2;

    // ---- 6. Stability guard ----
    uint8_t delta = (newCenter > g_lastCenter)
                    ? (newCenter - g_lastCenter)
                    : (g_lastCenter - newCenter);
    if (delta > JUMP_LIMIT) {
        g_detected = false;
        return g_lastCenter;
    }

    g_lastCenter = newCenter;
    g_leftEdge   = leftEdge;
    g_rightEdge  = rightEdge;
    g_detected   = true;
    return newCenter;
}

// ============================================================
//  Print raw pixel CSV (128 values, tab-separated header)
// ============================================================
static void printRawCsv() {
    // Header: pixel indices
    for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
        Serial.print(i);
        if (i < PIXEL_COUNT - 1) Serial.print(',');
    }
    Serial.println();
    // Values: raw 10-bit ADC counts
    for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
        Serial.print(g_pixels[i]);
        if (i < PIXEL_COUNT - 1) Serial.print(',');
    }
    Serial.println();
}

// ============================================================
//  Print 40-column ASCII bar chart of last captured frame.
//  Uses 8-bit normalised values so the chart is consistent
//  regardless of the CLK half-period setting.
//
//  Each row covers 4 pixels (128 / 32 rows).  Bar width is
//  scaled so 255 → 20 characters.
// ============================================================
static void printWaveform() {
    Serial.println(F("# --- Waveform (8-bit, 4 pixels averaged per row) ---"));
    Serial.print(F("# threshold (8-bit) = ")); Serial.println(g_threshold);
    Serial.println(F("# 0         128       255"));
    Serial.println(F("# |---------|---------|"));

    for (uint8_t row = 0; row < 32; row++) {
        uint8_t base = row * 4;

        // Average 4 consecutive pixels, normalise to 8-bit
        uint32_t sum = 0;
        for (uint8_t k = 0; k < 4; k++) sum += (g_pixels[base + k] >> 2);
        uint8_t avg = static_cast<uint8_t>(sum / 4);

        // Scale 0–255 → 0–20 characters
        uint8_t barLen = (uint8_t)((uint16_t)avg * 20 / 255);

        // Pixel index label (right-justified to 3 chars)
        if (base < 10)        Serial.print(F("  "));
        else if (base < 100)  Serial.print(F(" "));
        Serial.print(base);
        Serial.print(F(" |"));

        for (uint8_t b = 0; b < barLen; b++) Serial.print('#');

        // Mark the centre pixel row
        bool isCentreRow = (g_lastCenter >= base && g_lastCenter < base + 4);
        if (isCentreRow) {
            for (uint8_t b = barLen; b < 20; b++) Serial.print(' ');
            Serial.print(F("| <-- centre"));
        }

        Serial.println();
    }
    Serial.println(F("# ---------------------------------------------------"));
}

// ============================================================
//  Full-frame stream output for the Python real-time plotter.
//
//  One line per frame, prefix "F:" so the plotter can identify it
//  unambiguously among comment lines starting with "#".
//
//  Format (all values comma-separated after "F:"):
//    ms        — millis() timestamp
//    clk_us    — CLK half-period (integration time indicator)
//    center    — detected centre pixel (0–127)
//    left      — left edge pixel
//    right     — right edge pixel
//    threshold — 8-bit dynamic threshold
//    detected  — 0 or 1
//    p0..p127  — 128 pixel values, 8-bit (10-bit ADC >> 2)
// ============================================================
static void printStreamFrame() {
    Serial.print(F("F:"));
    Serial.print(millis());       Serial.print(',');
    Serial.print(g_clkHalfUs);   Serial.print(',');
    Serial.print(g_lastCenter);  Serial.print(',');
    Serial.print(g_leftEdge);    Serial.print(',');
    Serial.print(g_rightEdge);   Serial.print(',');
    Serial.print(g_threshold);   Serial.print(',');
    Serial.print(g_detected ? 1 : 0);
    for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
        Serial.print(',');
        Serial.print(g_pixels[i] >> 2);   // 10-bit → 8-bit
    }
    Serial.println();
}

// ============================================================
//  Help / status
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# TSL1401 CCD + Line Laser Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  R    — Read one frame (raw CSV, 128 values)"));
    Serial.println(F("#  C    — Toggle continuous mode (~10 Hz)"));
    Serial.println(F("#  S    — Toggle stream mode for Python plotter"));
    Serial.println(F("#  W    — ASCII waveform of last frame"));
    Serial.println(F("#  + /- — Increase / decrease CLK half-period"));
    Serial.println(F("#  L    — Toggle laser (wiring check)"));
    Serial.println(F("#  ?    — This help"));
    Serial.println(F("# ---------------------------------------------"));

    // Pin summary
    Serial.print(F("# SI  pin : ")); Serial.println(Pins::LaserSensor::SI);
    Serial.print(F("# CLK pin : ")); Serial.println(Pins::LaserSensor::CLK);
    Serial.print(F("# AO  pin : ")); Serial.println(Pins::LaserSensor::AO);
    Serial.print(F("# Laser   : ")); Serial.println(Pins::LineLaser::ENABLE);
    Serial.print(F("# CLK ½T  : ")); Serial.print(g_clkHalfUs); Serial.println(F(" µs"));

    if (!pinsAssigned()) {
        Serial.println(F("# WARNING: SI or CLK still -1 in PinMap.h."));
        Serial.println(F("#   Set Pins::LaserSensor::SI and CLK before running."));
    }
    if (Pins::LineLaser::ENABLE < 0) {
        Serial.println(F("# NOTE: Laser pin not set — laser steps skipped."));
        Serial.println(F("#   Set Pins::LineLaser::ENABLE in PinMap.h to enable."));
    }
    Serial.println(F("# ============================================="));
}

// ============================================================
//  Entry points
// ============================================================
void testTSL1401_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Configure digital output pins
    if (Pins::LaserSensor::CLK >= 0) {
        pinMode(Pins::LaserSensor::CLK, OUTPUT);
        digitalWrite(Pins::LaserSensor::CLK, LOW);
    }
    if (Pins::LaserSensor::SI >= 0) {
        pinMode(Pins::LaserSensor::SI, OUTPUT);
        digitalWrite(Pins::LaserSensor::SI, LOW);
    }
    if (Pins::LineLaser::ENABLE >= 0) {
        pinMode(Pins::LineLaser::ENABLE, OUTPUT);
        digitalWrite(Pins::LineLaser::ENABLE, LOW);   // Laser off at startup
    }
    // AO is analog in — no pinMode needed for Teensy analogRead

    // Prime the TSL1401 pipeline.
    // The sensor outputs pixel data from the PREVIOUS integration
    // cycle, so the first capture after power-on holds stale data.
    // Run one dummy capture (laser on briefly) to flush it.
    if (pinsAssigned()) {
        capture();   // result discarded
        Serial.println(F("# TSL1401 primed (dummy frame captured)."));
    }

    printHelp();
}

void testTSL1401_loop() {

    // ---- Stream mode: full-frame output for Python plotter ~10 Hz ----
    if (g_streaming) {
        static uint32_t lastStreamMs = 0;
        if (millis() - lastStreamMs >= 100) {
            lastStreamMs = millis();
            if (!pinsAssigned()) {
                Serial.println(F("# Stream mode: pins not set, stopping."));
                g_streaming = false;
            } else {
                capture();
                findCenter();
                printStreamFrame();
            }
        }
    }

    // ---- Continuous mode: capture + detect ~10 Hz ----
    if (g_continuous) {
        static uint32_t lastCaptureMs = 0;
        if (millis() - lastCaptureMs >= 100) {
            lastCaptureMs = millis();

            if (!pinsAssigned()) {
                Serial.println(F("# Continuous mode: pins not set, skipping."));
                g_continuous = false;
                return;
            }

            capture();
            uint8_t ctr = findCenter();

            // Output: frame_ms, centre, threshold (8-bit), detected
            Serial.print(millis());     Serial.print(',');
            Serial.print(ctr);          Serial.print(',');
            Serial.print(g_threshold);  Serial.print(',');
            Serial.println(g_detected ? 1 : 0);
        }
    }

    // ---- Serial commands ----
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {

        case 'r': case 'R':
            if (!pinsAssigned()) {
                Serial.println(F("# Pins not set in PinMap.h (SI, CLK)."));
                break;
            }
            capture();
            findCenter();
            printRawCsv();
            Serial.print(F("# Centre: ")); Serial.print(g_lastCenter);
            Serial.print(F("  Threshold: ")); Serial.print(g_threshold);
            Serial.print(F("  Detected: ")); Serial.println(g_detected ? F("YES") : F("NO"));
            break;

        case 'c': case 'C':
            g_continuous = !g_continuous;
            if (g_continuous) {
                Serial.println(F("# Continuous ON  (frame_ms,centre,threshold,detected)"));
                Serial.println(F("# Press C again to stop."));
            } else {
                Serial.println(F("# Continuous OFF"));
            }
            break;

        case 's': case 'S':
            g_streaming = !g_streaming;
            if (g_streaming) {
                Serial.println(F("# Stream ON  — full-frame F: lines for Python plotter."));
                Serial.println(F("# Press S again to stop."));
            } else {
                Serial.println(F("# Stream OFF"));
            }
            break;

        case 'w': case 'W':
            printWaveform();
            break;

        case '+': case '=':
            if (g_clkHalfUs + CLK_STEP_US <= CLK_MAX_US)
                g_clkHalfUs += CLK_STEP_US;
            else
                g_clkHalfUs = CLK_MAX_US;
            Serial.print(F("# CLK half-period: ")); Serial.print(g_clkHalfUs);
            Serial.println(F(" µs  (longer = more sensitive)"));
            break;

        case '-': case '_':
            if (g_clkHalfUs > CLK_MIN_US + CLK_STEP_US)
                g_clkHalfUs -= CLK_STEP_US;
            else
                g_clkHalfUs = CLK_MIN_US;
            Serial.print(F("# CLK half-period: ")); Serial.print(g_clkHalfUs);
            Serial.println(F(" µs  (shorter = faster, less sensitive)"));
            break;

        case 'l': case 'L':
            if (Pins::LineLaser::ENABLE < 0) {
                Serial.println(F("# Laser pin not set in PinMap.h (Pins::LineLaser::ENABLE)."));
                break;
            }
            if (g_laserOn) {
                laserOff();
                Serial.println(F("# Laser OFF"));
            } else {
                laserOn();
                Serial.println(F("# Laser ON  (remember to turn off — press L again)"));
            }
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

#endif // TEST_MODE
