#pragma once

#include <Arduino.h>
#include <cstdint>
#include "state/MachineState.h"
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  LaserSensor.h  —  TSL1401 128-pixel linear CCD array.
//
//  Hardware overview:
//    128-pixel photodiode array, one analog output (AO).
//    3-wire interface: CLK, SI, AO.
//    CLK drives the shift register. SI triggers a new
//    acquisition + output cycle simultaneously.
//
//  Timing (from TSL1401 datasheet and confirmed by docs):
//    - One full cycle = 129 CLK pulses.
//    - On each cycle, assert SI HIGH on the 1st CLK rising edge.
//    - Clocks 1–18:  Internal reset (no integration).
//    - Clocks 19–129: Integration (light accumulates).
//    - AO outputs each pixel on CLK falling edges 2–129.
//    - Data output this cycle = data collected in the PREVIOUS
//      cycle. Always discard the first capture after power-on.
//
//  ADC note:
//    Teensy 4.1 ADC is 10-bit (0–1023) by default.
//    Pixel values are stored as uint16_t (raw ADC counts).
//    The normalize() helper rescales to 0–255 for the
//    algorithm if preferred. See PIXEL_SHIFT_BITS below.
//
//  Dynamic threshold algorithm (from reference docs):
//    1. Skip 5 pixels on each edge (indices 5–122).
//    2. Find global min and max in that window.
//    3. Threshold = (max + min) / 2.
//    4. Left edge:  scan left→right, find 3 consecutive ABOVE
//       threshold immediately followed by 3 BELOW.
//    5. Right edge: scan right→left, find 3 consecutive BELOW
//       threshold immediately followed by 3 ABOVE.
//    6. Center = (leftEdge + rightEdge) / 2.
//    7. Stability guard: if |newCenter - lastCenter| > JUMP_LIMIT,
//       keep the previous center value.
//
//  This algorithm is isolated in findCenter() so it can be
//  easily replaced or tuned without touching capture logic.
// ============================================================

// Number of right-shifts to downscale 10-bit ADC to 8-bit range.
// Set to 0 to keep full 10-bit values in the algorithm.
// TODO: Tune based on your lighting conditions.
constexpr uint8_t PIXEL_SHIFT_BITS = 2;   // 10-bit → 8-bit

// Pixels to skip on each edge of the array to avoid border noise
constexpr uint8_t EDGE_SKIP = 5;

// Minimum consecutive pixels required to confirm a threshold crossing
constexpr uint8_t EDGE_CONFIRM = 3;        // docs use 3 — do not reduce below 2

// Maximum allowed center jump between frames before value is rejected
// TODO: Tune — docs used 70 (out of 128); adjust for your petri dish geometry
constexpr uint8_t CENTER_JUMP_LIMIT = 70;

// TSL1401 CLK half-period in microseconds (sets integration time / sensitivity)
// Longer = more integration = more sensitive. Docs use 10µs (100 kHz).
// TODO: Tune for your lighting environment. Safe range: 1–100µs (TSL1401 supports up to 8 MHz).
constexpr uint32_t CCD_CLK_HALF_US = 10;  // 10µs → 50 kHz clock; safe for long wires

// Number of pixels in the TSL1401
constexpr uint8_t CCD_PIXEL_COUNT = 128;

class LaserSensor {
public:
    static LaserSensor& instance() {
        static LaserSensor inst;
        return inst;
    }

    // Disallow copy
    LaserSensor(const LaserSensor&)            = delete;
    LaserSensor& operator=(const LaserSensor&) = delete;

    // ----------------------------------------------------------
    //  Call once in setup() to configure CLK and SI as outputs.
    //  AO is handled by analogRead() — no pinMode needed.
    //  Performs one dummy capture to prime the pixel pipeline.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Drive the TSL1401 timing sequence to capture one frame.
    //  Fills pixels_[] with 128 raw ADC values.
    //  Call once per control loop tick (or at desired frame rate).
    // ----------------------------------------------------------
    void capture();

    // ----------------------------------------------------------
    //  Run the dynamic threshold algorithm on the last captured
    //  frame. Returns the center pixel index (0–127).
    //  Returns lastCenter_ if no valid edges are found this frame.
    // ----------------------------------------------------------
    uint8_t findCenter();

    // ----------------------------------------------------------
    //  Returns the dynamic threshold computed during the last
    //  findCenter() call.
    // ----------------------------------------------------------
    uint16_t getThreshold() const { return threshold_; }

    // ----------------------------------------------------------
    //  Returns the raw pixel array from the last capture.
    //  Array is CCD_PIXEL_COUNT elements long.
    // ----------------------------------------------------------
    const uint16_t* getPixels() const { return pixels_; }

    // ----------------------------------------------------------
    //  Returns the pixel value at index, clamped to valid range.
    // ----------------------------------------------------------
    uint16_t getPixel(uint8_t index) const;

    // ----------------------------------------------------------
    //  Returns true if a valid dark region (black object/line)
    //  was detected in the last findCenter() call.
    // ----------------------------------------------------------
    bool isTargetDetected() const { return targetDetected_; }

    // ----------------------------------------------------------
    //  Returns the last valid center pixel index.
    //  This is stable across frames where no target is detected.
    // ----------------------------------------------------------
    uint8_t getLastCenter() const { return lastCenter_; }

    // ----------------------------------------------------------
    //  Normalize a raw pixel value from 10-bit to 8-bit range.
    //  Applies PIXEL_SHIFT_BITS right shift.
    // ----------------------------------------------------------
    static uint16_t normalize(uint16_t raw) { return raw >> PIXEL_SHIFT_BITS; }

    // ----------------------------------------------------------
    //  Sync sensor state to MachineState.
    //  Call after capture() + findCenter() each frame.
    // ----------------------------------------------------------
    void syncState();

private:
    LaserSensor() = default;

    // ----------------------------------------------------------
    //  Low-level CLK pulse helper (blocking, delay-based)
    // ----------------------------------------------------------
    void clockPulse();

    uint16_t pixels_[CCD_PIXEL_COUNT] = {0};   // Raw ADC values
    uint16_t threshold_    = 0;                 // Last computed dynamic threshold
    uint8_t  lastCenter_   = 64;                // Stable center (pixel 64 = optical center)
    bool     targetDetected_ = false;           // True when a valid dark region is found
    bool     initialized_  = false;
};
