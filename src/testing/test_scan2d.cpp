// ============================================================
//  testing/test_scan2d.cpp
//
//  PURPOSE
//  -------
//  2D raster scan: move the gantry slowly in the Y direction
//  while the TSL1401 CCD (line sensor) captures 128-pixel X
//  slices.  The resulting intensity map is analysed to detect
//  objects (petri dishes) and report their centres in mm
//  relative to the global Cartesian coordinate frame.
//
//  HOW IT WORKS
//  ------------
//  1. The gantry is driven open-loop in +Y (no encoder feedback).
//  2. Every SCAN_STEP_MM of travel the line laser fires, the CCD
//     captures one 128-pixel row, and the frame is stored.
//  3. After the scan the gantry returns to its starting position.
//  4. The stored map is analysed:
//       a. Each pixel is normalised to 8-bit (0–255).
//       b. Pixels ≥ DETECT_THRESHOLD are marked as "bright"
//          (laser reflection from object surfaces).
//       c. Connected bright regions are grouped into blobs using
//          a single-pass scan-line algorithm.
//       d. Each blob's centroid is converted to mm using the
//          calibrated sensor geometry (see below).
//
//  COORDINATE CONVENTIONS
//  ----------------------
//  Pixel X → physical mm:
//    x_mm = GANTRY_X_SCAN_MM + (pixel - 63.5) × MM_PER_PIXEL
//
//    GANTRY_X_SCAN_MM — Cartesian X position of the gantry
//                       during the scan (set before calling S).
//    MM_PER_PIXEL     — calibrated: move gantry to a known
//                       position, place an object of known width,
//                       measure pixel span, divide width by span.
//
//  Row Y → physical mm:
//    y_mm = SCAN_START_Y_MM + row × SCAN_STEP_MM
//
//  SETUP — fill in PinMap.h and the config section below
//  ------------------------------------------------------
//  Pins::Gantry::MOTOR_A_STEP / _DIR   (already set: 4, 5)
//  Pins::Gantry::MOTOR_B_STEP / _DIR   (already set: 6, 7)
//  Pins::LaserSensor::SI / CLK / AO    (set SI and CLK; AO = 14)
//  Pins::LineLaser::ENABLE             (set this pin)
//
//  CoreXY — pure +Y motion:
//    Motor A: DIR = HIGH (forward)   → a_steps = +Y
//    Motor B: DIR = LOW  (reverse)   → b_steps = -Y
//  If Y moves backwards, swap MOTOR_Y_DIR_A / MOTOR_Y_DIR_B.
//
//  COMMANDS
//  --------
//    S    — Start scan (gantry moves, captures, then returns)
//    A    — Re-run object detection on stored map
//    D    — Dump raw map as CSV (paste into spreadsheet / Python)
//    M    — Print ASCII overview of stored map
//    +/-  — Raise / lower detection threshold
//    ?    — Print this help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testScan2D_setup();
//    extern void testScan2D_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  USER CONFIGURATION  —  edit these before first run
// ============================================================

// ---- Coordinate calibration --------------------------------
// Physical mm per CCD pixel at your working distance.
// Measure: place a ruler under the sensor, capture a frame,
// count how many pixels a known mm span covers, then
//   MM_PER_PIXEL = known_mm / pixel_span
// Typical range: 0.1–1.0 mm/pixel depending on lens & distance.
static constexpr float MM_PER_PIXEL = 0.5f;   // TODO: calibrate

// Gantry Cartesian X position (mm) during the scan.
// Position the gantry here before pressing S.
static constexpr float GANTRY_X_SCAN_MM = 100.0f;  // TODO: set

// Y position the gantry starts the scan from (mm).
// Position the gantry here before pressing S.
static constexpr float SCAN_START_Y_MM = 5.0f;     // TODO: set

// ---- Scan geometry -----------------------------------------
// Total Y range to sweep (mm).
static constexpr float SCAN_RANGE_MM = 150.0f;     // TODO: set

// Y distance between successive CCD captures (mm).
// Smaller = finer resolution, more rows, longer scan.
// Must be ≥ 1 / STEPS_PER_MM (one step resolution).
static constexpr float SCAN_STEP_MM = 1.0f;        // TODO: tune

// Maximum scan rows (caps memory use regardless of range).
// At 1.0 mm/step and 150 mm range: 150 rows × 128 × 2 = 38 KB.
// Raise up to ~350 rows before hitting Teensy SRAM limits.
static constexpr uint16_t MAX_ROWS = 200;

// ---- Scan speed --------------------------------------------
// Gantry scan speed in mm/s.  Keep slow for clean captures.
// TB6600 + 17HS24-2004D-E1K: 5–20 mm/s is safe open-loop.
static constexpr float SCAN_SPEED_MM_S = 5.0f;     // TODO: tune

// Return speed after scan completes (can be faster).
static constexpr float RETURN_SPEED_MM_S = 20.0f;

// ---- CoreXY direction sense --------------------------------
// For pure +Y motion: Motor A steps forward, Motor B backward.
// If Y moves in the wrong direction, swap these two values.
static constexpr bool MOTOR_Y_DIR_A = HIGH;  // Motor A DIR for +Y
static constexpr bool MOTOR_Y_DIR_B = LOW;   // Motor B DIR for +Y

// ---- Object detection threshold ----------------------------
// Pixel values (0–255, 8-bit normalised from 10-bit ADC) at or
// above this level are considered "bright" (object reflection).
// Start at 150 and adjust with +/- until only true reflections
// are marked. Too low → noise; too high → misses dim objects.
static uint8_t g_threshold = 150;

// Minimum connected bright pixels to count as a real object.
// Reject blobs smaller than this (dust, sensor noise).
static constexpr uint16_t MIN_BLOB_PIXELS = 30;

// ---- CCD clock half-period ---------------------------------
// Longer = more light integration = more sensitive.
// 10 µs (100 kHz) is the documented default.
static constexpr uint32_t CLK_HALF_US = 10;

// ============================================================
//  DERIVED CONSTANTS  —  do not edit
// ============================================================
static constexpr uint8_t  PIXEL_COUNT = 128;

// Steps per mm from Constants (stepper + microstepping + belt)
static constexpr float STEPS_PER_MM  = Constants::Gantry::STEPS_PER_MM;

// Steps between captures
static constexpr uint32_t STEPS_BETWEEN_CAPTURES =
    static_cast<uint32_t>(SCAN_STEP_MM * STEPS_PER_MM + 0.5f);

// Total steps for full scan range
static constexpr uint32_t TOTAL_SCAN_STEPS =
    static_cast<uint32_t>(SCAN_RANGE_MM * STEPS_PER_MM + 0.5f);

// Step pulse high time (µs); TB6600 needs ≥ 2.2 µs
static constexpr uint32_t STEP_PULSE_US = 5;

// ============================================================
//  2D MAP STORAGE
// ============================================================
// g_map[row][pixel] — raw 10-bit ADC values from each capture.
// row 0 = SCAN_START_Y_MM, row 1 = SCAN_START_Y_MM + SCAN_STEP_MM, etc.
static uint16_t g_map[MAX_ROWS][PIXEL_COUNT];
static uint16_t g_rowCount = 0;   // Rows stored so far

// ============================================================
//  BLOB DETECTION
// ============================================================
// A blob tracks a connected region of bright pixels across rows.
// Single-pass scan-line algorithm: O(rows × pixels), no heap.

static constexpr uint8_t MAX_BLOBS = 16;

struct Blob {
    uint32_t sumPx;         // Σ pixel_index  (for centroid X)
    uint32_t sumRow;        // Σ row_index    (for centroid Y)
    uint32_t pixelCount;    // Total bright pixels accumulated
    uint8_t  prevMinPx;     // Previous row's run min pixel (for row overlap)
    uint8_t  prevMaxPx;     // Previous row's run max pixel
    uint8_t  lastActiveRow; // Last row in which this blob was extended
    bool     alive;         // Still being tracked
};

static Blob    g_blobs[MAX_BLOBS];
static uint8_t g_blobCount = 0;
static uint8_t g_finalBlobs = 0;   // Blobs surviving MIN_BLOB_PIXELS after last analysis

// ============================================================
//  PIN GUARD HELPERS
// ============================================================
static bool ccdPinsOk() {
    return (Pins::LaserSensor::SI  >= 0 &&
            Pins::LaserSensor::CLK >= 0 &&
            Pins::LaserSensor::AO  >= 0);
}

static bool stepperPinsOk() {
    return (Pins::Gantry::MOTOR_A_STEP >= 0 && Pins::Gantry::MOTOR_A_DIR >= 0 &&
            Pins::Gantry::MOTOR_B_STEP >= 0 && Pins::Gantry::MOTOR_B_DIR >= 0);
}

// ============================================================
//  LASER HELPERS  (direct pin drive — no LineLaser class)
// ============================================================
static void laserOn() {
    if (Pins::LineLaser::ENABLE >= 0)
        digitalWrite(Pins::LineLaser::ENABLE, HIGH);
}
static void laserOff() {
    if (Pins::LineLaser::ENABLE >= 0)
        digitalWrite(Pins::LineLaser::ENABLE, LOW);
}

// ============================================================
//  CCD CAPTURE  (direct pin drive — no LaserSensor class)
//
//  Laser on → full 129-clock TSL1401 sequence → laser off.
//  Result written into dest[0..127].
// ============================================================
static void captureInto(uint16_t* dest) {
    if (!ccdPinsOk()) return;

    laserOn();

    // Step 1: starting state
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(CLK_HALF_US);

    // Step 2: assert SI, drop CLK
    digitalWrite(Pins::LaserSensor::SI,  HIGH);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(CLK_HALF_US);

    // Step 3: rise CLK, de-assert SI
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(CLK_HALF_US);

    // Step 4: read 128 pixels (sample on CLK falling edge)
    for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
        digitalWrite(Pins::LaserSensor::CLK, LOW);
        delayMicroseconds(CLK_HALF_US);
        dest[i] = static_cast<uint16_t>(analogRead(Pins::LaserSensor::AO));
        digitalWrite(Pins::LaserSensor::CLK, HIGH);
        delayMicroseconds(CLK_HALF_US);
    }

    laserOff();
}

// ============================================================
//  STEPPER HELPERS  (direct STEP/DIR — no ClosedLoopStepper)
//
//  For CoreXY pure ±Y motion:
//    Motor A steps in MOTOR_Y_DIR_A direction.
//    Motor B steps in MOTOR_Y_DIR_B direction (opposite of A).
//  Reverse both booleans to move in −Y.
// ============================================================
static void stepY(bool forward, uint32_t delayBetweenStepsUs) {
    bool dirA = forward ? MOTOR_Y_DIR_A : !MOTOR_Y_DIR_A;
    bool dirB = forward ? MOTOR_Y_DIR_B : !MOTOR_Y_DIR_B;

    digitalWrite(Pins::Gantry::MOTOR_A_DIR, dirA ? HIGH : LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_DIR, dirB ? HIGH : LOW);
    delayMicroseconds(2);   // DIR setup time before STEP pulse

    // Fire one step pulse on both motors simultaneously
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, HIGH);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW);
    digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW);

    // Remaining inter-step delay (minus the pulse time already spent)
    if (delayBetweenStepsUs > STEP_PULSE_US)
        delayMicroseconds(delayBetweenStepsUs - STEP_PULSE_US);
}

// ============================================================
//  SCAN SEQUENCE
//
//  Moves the gantry +Y, capturing a CCD row every
//  STEPS_BETWEEN_CAPTURES steps.  Stores up to MAX_ROWS frames.
//  Returns the gantry to its starting position when done.
// ============================================================
static void runScan() {
    if (!ccdPinsOk()) {
        Serial.println(F("# SKIP: CCD pins not set in PinMap.h (SI, CLK)."));
        return;
    }
    if (!stepperPinsOk()) {
        Serial.println(F("# SKIP: Gantry step/dir pins not set in PinMap.h."));
        return;
    }

    g_rowCount = 0;

    // Delay between steps for scan speed (µs)
    uint32_t scanStepDelayUs =
        static_cast<uint32_t>(1000000.0f / (SCAN_SPEED_MM_S * STEPS_PER_MM));

    // Delay between steps for return speed (µs)
    uint32_t returnStepDelayUs =
        static_cast<uint32_t>(1000000.0f / (RETURN_SPEED_MM_S * STEPS_PER_MM));

    // Prime the CCD pipeline: first capture is stale (previous cycle's data).
    // Fire a dummy capture before moving.
    uint16_t dummy[PIXEL_COUNT];
    captureInto(dummy);

    Serial.println(F("# Scan started — press no keys during scan."));
    Serial.print(F("# Range: ")); Serial.print(SCAN_RANGE_MM, 1); Serial.println(F(" mm"));
    Serial.print(F("# Step:  ")); Serial.print(SCAN_STEP_MM, 2); Serial.println(F(" mm/row"));
    Serial.print(F("# Speed: ")); Serial.print(SCAN_SPEED_MM_S, 1); Serial.println(F(" mm/s"));
    Serial.print(F("# Steps per capture: ")); Serial.println(STEPS_BETWEEN_CAPTURES);

    uint32_t totalSteps     = min(TOTAL_SCAN_STEPS,
                                  static_cast<uint32_t>(MAX_ROWS) * STEPS_BETWEEN_CAPTURES);
    uint32_t stepsSinceCapture = 0;
    uint32_t stepsTaken        = 0;
    uint32_t progressPrintStep = totalSteps / 10;   // Print progress every 10%
    if (progressPrintStep == 0) progressPrintStep = 1;

    // ---- Forward sweep (+Y) ----
    while (stepsTaken < totalSteps && g_rowCount < MAX_ROWS) {
        stepY(true, scanStepDelayUs);
        stepsTaken++;
        stepsSinceCapture++;

        // Progress update every ~10% of travel
        if (stepsTaken % progressPrintStep == 0) {
            Serial.print(F("# ... ")); Serial.print(stepsTaken * 100 / totalSteps);
            Serial.print(F("%  row ")); Serial.print(g_rowCount);
            Serial.print(F(" / ")); Serial.println(MAX_ROWS);
        }

        // Capture every STEPS_BETWEEN_CAPTURES steps
        if (stepsSinceCapture >= STEPS_BETWEEN_CAPTURES) {
            stepsSinceCapture = 0;
            captureInto(g_map[g_rowCount]);
            g_rowCount++;
        }

        // Honour serial stop command mid-scan
        if (Serial.available()) {
            char c = static_cast<char>(Serial.peek());
            if (c == 'x' || c == 'X') {
                Serial.read();
                Serial.println(F("# Scan aborted by user."));
                goto returnHome;
            }
        }
    }

    Serial.print(F("# Scan complete — ")); Serial.print(g_rowCount);
    Serial.println(F(" rows captured."));

returnHome:
    // ---- Return sweep (-Y) — same step count ----
    Serial.println(F("# Returning to start position..."));
    for (uint32_t i = 0; i < stepsTaken; i++) {
        stepY(false, returnStepDelayUs);
    }
    Serial.println(F("# Gantry returned. Ready for analysis (press A)."));
}

// ============================================================
//  OBJECT DETECTION
//
//  Single-pass scan-line blob detection on the stored map.
//
//  Algorithm:
//    For each row:
//      1. Find all contiguous runs of pixels ≥ g_threshold (8-bit).
//      2. For each run, check if it overlaps any blob active in
//         the previous row (prev_min_px ≤ run_max && run_min ≤ prev_max_px).
//         - Overlap found → extend that blob, update its prev extents.
//         - No overlap    → create a new blob.
//    After processing all rows:
//      3. Discard blobs with pixelCount < MIN_BLOB_PIXELS (noise).
//      4. Convert surviving blob centroids to mm.
// ============================================================
static void detectObjects() {
    if (g_rowCount == 0) {
        Serial.println(F("# No scan data. Run a scan first (press S)."));
        return;
    }

    // Clear blob table
    for (uint8_t b = 0; b < MAX_BLOBS; b++) {
        g_blobs[b] = {0, 0, 0, 0, 0, 0, false};
    }
    g_blobCount  = 0;
    g_finalBlobs = 0;

    // Rows are closed (made inactive) if they weren't seen for >2 rows.
    // We keep this gap tolerance to bridge thin gaps between petri dish
    // edges that might dip below threshold momentarily.
    constexpr uint8_t ROW_GAP_TOLERANCE = 2;

    for (uint16_t row = 0; row < g_rowCount; row++) {

        // Build a list of bright pixel runs in this row.
        // A run is [runMin, runMax] — inclusive pixel indices.
        struct Run { uint8_t minPx; uint8_t maxPx; bool matched; };
        Run runs[16];
        uint8_t runCount = 0;

        bool inRun = false;
        uint8_t runStart = 0;

        for (uint8_t px = 0; px < PIXEL_COUNT && runCount < 16; px++) {
            bool bright = ((g_map[row][px] >> 2) >= g_threshold);   // 10-bit → 8-bit

            if (bright && !inRun) {
                runStart = px;
                inRun    = true;
            } else if (!bright && inRun) {
                runs[runCount++] = {runStart, static_cast<uint8_t>(px - 1), false};
                inRun = false;
            }
        }
        if (inRun) runs[runCount++] = {runStart, PIXEL_COUNT - 1, false};

        // Match each run to an existing blob or create a new one.
        for (uint8_t r = 0; r < runCount; r++) {
            uint8_t bestBlob = MAX_BLOBS;   // Sentinel: no match yet

            for (uint8_t b = 0; b < g_blobCount; b++) {
                if (!g_blobs[b].alive) continue;

                // Still active within gap tolerance?
                if ((row - g_blobs[b].lastActiveRow) > ROW_GAP_TOLERANCE) {
                    g_blobs[b].alive = false;   // Close stale blob
                    continue;
                }

                // Check X overlap
                if (runs[r].minPx <= g_blobs[b].prevMaxPx &&
                    g_blobs[b].prevMinPx <= runs[r].maxPx) {
                    bestBlob = b;
                    break;
                }
            }

            if (bestBlob == MAX_BLOBS) {
                // No matching blob — create a new one if capacity allows
                if (g_blobCount < MAX_BLOBS) {
                    bestBlob = g_blobCount++;
                    g_blobs[bestBlob].alive = true;
                    g_blobs[bestBlob].sumPx = 0;
                    g_blobs[bestBlob].sumRow = 0;
                    g_blobs[bestBlob].pixelCount = 0;
                }
            }

            if (bestBlob < MAX_BLOBS) {
                // Accumulate centroid sums over all bright pixels in this run
                for (uint8_t px = runs[r].minPx; px <= runs[r].maxPx; px++) {
                    if ((g_map[row][px] >> 2) >= g_threshold) {
                        g_blobs[bestBlob].sumPx  += px;
                        g_blobs[bestBlob].sumRow += row;
                        g_blobs[bestBlob].pixelCount++;
                    }
                }
                g_blobs[bestBlob].prevMinPx     = runs[r].minPx;
                g_blobs[bestBlob].prevMaxPx     = runs[r].maxPx;
                g_blobs[bestBlob].lastActiveRow = static_cast<uint8_t>(
                    row < 255 ? row : 255);
                runs[r].matched = true;
            }
        }
    }

    // ---- Report results ----
    Serial.println(F("# ============================================="));
    Serial.println(F("# Object Detection Results"));
    Serial.print(F("# Threshold (8-bit): ")); Serial.println(g_threshold);
    Serial.print(F("# Min blob size:     ")); Serial.println(MIN_BLOB_PIXELS);
    Serial.println(F("# -----"));

    uint8_t objIdx = 1;
    for (uint8_t b = 0; b < g_blobCount; b++) {
        if (g_blobs[b].pixelCount < MIN_BLOB_PIXELS) continue;

        float centPx  = static_cast<float>(g_blobs[b].sumPx)  / g_blobs[b].pixelCount;
        float centRow = static_cast<float>(g_blobs[b].sumRow) / g_blobs[b].pixelCount;

        // Convert centroid to physical mm
        float x_mm = GANTRY_X_SCAN_MM + (centPx  - 63.5f) * MM_PER_PIXEL;
        float y_mm = SCAN_START_Y_MM   + (centRow * SCAN_STEP_MM);

        Serial.print(F("# Object ")); Serial.print(objIdx++); Serial.println(F(":"));
        Serial.print(F("#   Centre  X = ")); Serial.print(x_mm,  2); Serial.println(F(" mm"));
        Serial.print(F("#   Centre  Y = ")); Serial.print(y_mm,  2); Serial.println(F(" mm"));
        Serial.print(F("#   Pixels  : ")); Serial.println(g_blobs[b].pixelCount);
        Serial.print(F("#   Centroid pixel  = ")); Serial.print(centPx,  1);
        Serial.print(F("  row = ")); Serial.println(centRow, 1);

        g_finalBlobs++;
    }

    if (g_finalBlobs == 0) {
        Serial.println(F("# No objects detected."));
        Serial.println(F("# Tips:"));
        Serial.println(F("#   - Lower threshold with '-' if objects are missed."));
        Serial.println(F("#   - Raise threshold with '+' if noise is detected."));
        Serial.println(F("#   - Check laser is firing (press L in test_tsl1401)."));
        Serial.println(F("#   - Verify MM_PER_PIXEL calibration."));
    } else {
        Serial.print(F("# Total objects: ")); Serial.println(g_finalBlobs);
        Serial.println(F("#"));
        Serial.println(F("# NOTE: Accuracy depends on MM_PER_PIXEL calibration."));
        Serial.println(F("# To calibrate: place an object of known width, run S,"));
        Serial.println(F("#   measure its pixel span in the CSV dump (D), then:"));
        Serial.println(F("#   MM_PER_PIXEL = known_width_mm / pixel_span"));
    }
    Serial.println(F("# ============================================="));
}

// ============================================================
//  ASCII MAP OVERVIEW
//
//  Prints a downsampled ASCII intensity map.
//  Columns: pixel index (every 8th = 16 columns for 128 pixels).
//  Rows: every PREVIEW_ROW_STRIDE-th stored row.
//  Brightness: ' ' . : + # (dark → bright)
// ============================================================
static void printMapOverview() {
    if (g_rowCount == 0) {
        Serial.println(F("# No scan data yet."));
        return;
    }

    constexpr uint8_t PREVIEW_COLS       = 32;    // columns in preview (every 4th pixel)
    constexpr uint8_t PIXEL_STRIDE       = PIXEL_COUNT / PREVIEW_COLS;  // 4
    constexpr uint8_t PREVIEW_ROWS       = 40;
    const uint16_t    rowStride          = max((uint16_t)1,
                                               (uint16_t)(g_rowCount / PREVIEW_ROWS));

    static const char SHADE[] = " .:+#";   // 5 levels (0–255 → 0–4)

    Serial.println(F("# === Intensity Map (ASCII preview) ==="));
    Serial.print(F("# Y\\X   "));
    for (uint8_t c = 0; c < PREVIEW_COLS; c++) {
        if (c % 4 == 0) {
            uint8_t px = c * PIXEL_STRIDE;
            if (px < 10) Serial.print('0');
            Serial.print(px);
            Serial.print(' ');
        }
    }
    Serial.println();

    for (uint16_t row = 0; row < g_rowCount; row += rowStride) {
        float y_mm = SCAN_START_Y_MM + row * SCAN_STEP_MM;
        // Right-justify Y label (4 chars)
        if (y_mm < 10.0f)        Serial.print(F("   "));
        else if (y_mm < 100.0f)  Serial.print(F("  "));
        else                     Serial.print(' ');
        Serial.print(y_mm, 0); Serial.print(F(" |"));

        for (uint8_t c = 0; c < PREVIEW_COLS; c++) {
            uint8_t px = c * PIXEL_STRIDE;
            uint8_t v  = static_cast<uint8_t>(g_map[row][px] >> 2);   // 10→8 bit
            uint8_t level = v * 4 / 255;
            Serial.print(SHADE[level]);
        }

        // Mark rows with detected blob centroids
        bool markedRow = false;
        for (uint8_t b = 0; b < g_blobCount; b++) {
            if (!g_blobs[b].alive && g_blobs[b].pixelCount < MIN_BLOB_PIXELS) continue;
            float centRow = static_cast<float>(g_blobs[b].sumRow) / g_blobs[b].pixelCount;
            if (fabsf(centRow - row) < rowStride) {
                Serial.print(F(" <obj"));
                markedRow = true;
                break;
            }
        }
        if (!markedRow) Serial.println();
        else            Serial.println('>');
    }
    Serial.println(F("# ====================================="));
}

// ============================================================
//  RAW MAP CSV DUMP
//
//  Format: header row of pixel indices, then one row per
//  captured frame: y_mm, p0, p1, ..., p127
//  Paste into a spreadsheet or Python to plot.
// ============================================================
static void dumpMapCsv() {
    if (g_rowCount == 0) {
        Serial.println(F("# No scan data yet."));
        return;
    }

    Serial.println(F("# === Raw Map CSV (y_mm, pixel[0..127]) ==="));
    // Header
    Serial.print(F("y_mm"));
    for (uint8_t px = 0; px < PIXEL_COUNT; px++) {
        Serial.print(','); Serial.print(F("p")); Serial.print(px);
    }
    Serial.println();

    // Data rows
    for (uint16_t row = 0; row < g_rowCount; row++) {
        float y_mm = SCAN_START_Y_MM + row * SCAN_STEP_MM;
        Serial.print(y_mm, 2);
        for (uint8_t px = 0; px < PIXEL_COUNT; px++) {
            Serial.print(',');
            Serial.print(g_map[row][px] >> 2);   // 10-bit → 8-bit
        }
        Serial.println();
    }
    Serial.println(F("# === End CSV ==="));
}

// ============================================================
//  HELP / STATUS
// ============================================================
static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# 2D Scan + Object Detection Test"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  S    — Start scan (gantry moves in +Y, then returns)"));
    Serial.println(F("#  A    — Re-run object detection on stored map"));
    Serial.println(F("#  D    — Dump raw map as CSV"));
    Serial.println(F("#  M    — ASCII intensity map overview"));
    Serial.println(F("#  +/-  — Raise/lower detection threshold"));
    Serial.println(F("#  X    — Abort scan in progress"));
    Serial.println(F("#  ?    — This help"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("# Scan range:    ")); Serial.print(SCAN_RANGE_MM, 1); Serial.println(F(" mm Y"));
    Serial.print(F("# Step size:     ")); Serial.print(SCAN_STEP_MM, 2); Serial.println(F(" mm/row"));
    Serial.print(F("# Max rows:      ")); Serial.println(MAX_ROWS);
    Serial.print(F("# Scan speed:    ")); Serial.print(SCAN_SPEED_MM_S, 1); Serial.println(F(" mm/s"));
    Serial.print(F("# MM_PER_PIXEL:  ")); Serial.println(MM_PER_PIXEL, 4);
    Serial.print(F("# Gantry X:      ")); Serial.print(GANTRY_X_SCAN_MM, 1); Serial.println(F(" mm"));
    Serial.print(F("# Scan start Y:  ")); Serial.print(SCAN_START_Y_MM, 1); Serial.println(F(" mm"));
    Serial.print(F("# Threshold:     ")); Serial.print(g_threshold); Serial.println(F(" / 255"));
    Serial.print(F("# Stored rows:   ")); Serial.println(g_rowCount);
    Serial.println(F("# ---- Pins ----"));
    Serial.print(F("# CCD SI:        ")); Serial.println(Pins::LaserSensor::SI);
    Serial.print(F("# CCD CLK:       ")); Serial.println(Pins::LaserSensor::CLK);
    Serial.print(F("# CCD AO:        ")); Serial.println(Pins::LaserSensor::AO);
    Serial.print(F("# Laser:         ")); Serial.println(Pins::LineLaser::ENABLE);
    Serial.print(F("# Motor A STEP:  ")); Serial.println(Pins::Gantry::MOTOR_A_STEP);
    Serial.print(F("# Motor B STEP:  ")); Serial.println(Pins::Gantry::MOTOR_B_STEP);

    if (!ccdPinsOk())
        Serial.println(F("# WARNING: CCD SI or CLK not set in PinMap.h."));
    if (!stepperPinsOk())
        Serial.println(F("# WARNING: Gantry STEP/DIR pins not set in PinMap.h."));
    if (STEPS_BETWEEN_CAPTURES == 0)
        Serial.println(F("# WARNING: SCAN_STEP_MM too small for STEPS_PER_MM — increase it."));
    Serial.println(F("# ============================================="));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testScan2D_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Configure gantry pins
    if (Pins::Gantry::MOTOR_A_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_A_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_A_STEP, LOW); }
    if (Pins::Gantry::MOTOR_A_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_A_DIR,  OUTPUT); }
    if (Pins::Gantry::MOTOR_B_STEP >= 0) { pinMode(Pins::Gantry::MOTOR_B_STEP, OUTPUT); digitalWrite(Pins::Gantry::MOTOR_B_STEP, LOW); }
    if (Pins::Gantry::MOTOR_B_DIR  >= 0) { pinMode(Pins::Gantry::MOTOR_B_DIR,  OUTPUT); }

    // Configure CCD pins
    if (Pins::LaserSensor::CLK >= 0) { pinMode(Pins::LaserSensor::CLK, OUTPUT); digitalWrite(Pins::LaserSensor::CLK, LOW); }
    if (Pins::LaserSensor::SI  >= 0) { pinMode(Pins::LaserSensor::SI,  OUTPUT); digitalWrite(Pins::LaserSensor::SI,  LOW); }

    // Configure laser pin (OFF at startup)
    if (Pins::LineLaser::ENABLE >= 0) { pinMode(Pins::LineLaser::ENABLE, OUTPUT); laserOff(); }

    // Clear map
    memset(g_map, 0, sizeof(g_map));

    printHelp();
}

void testScan2D_loop() {
    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {

        case 's': case 'S':
            runScan();
            if (g_rowCount > 0) detectObjects();
            break;

        case 'a': case 'A':
            detectObjects();
            break;

        case 'd': case 'D':
            dumpMapCsv();
            break;

        case 'm': case 'M':
            printMapOverview();
            break;

        case '+': case '=':
            if (g_threshold < 245) g_threshold += 10; else g_threshold = 255;
            Serial.print(F("# Threshold: ")); Serial.print(g_threshold); Serial.println(F(" / 255"));
            break;

        case '-': case '_':
            if (g_threshold > 10) g_threshold -= 10; else g_threshold = 0;
            Serial.print(F("# Threshold: ")); Serial.print(g_threshold); Serial.println(F(" / 255"));
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
