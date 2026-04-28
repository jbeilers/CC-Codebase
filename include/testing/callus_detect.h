#pragma once
// ============================================================
//  testing/callus_detect.h  —  Standalone callus & dish utility
//
//  PURPOSE
//  -------
//  Processes TSL1401 pixel frames captured while the gantry
//  moves in Y to:
//    1. Locate callus positions  (narrow shadow → center pixel → mm)
//    2. Track petri-dish outline (wide shadow → first/last Y → center)
//
//  USAGE
//  -----
//    #include "testing/callus_detect.h"
//
//    CallusDetect::reset();
//    // ... for each captured frame while moving in Y:
//    CallusDetect::update(pixels, gantry_y_mm);
//
//    uint8_t n = CallusDetect::getCallusCount();
//    const CallusDetect::CallusPos* c = CallusDetect::getCalluses();
//    CallusDetect::DishInfo dish = CallusDetect::getDishInfo();
//
//  COORDINATE CONVENTION
//  ---------------------
//  CallusPos::x_mm  — pixel-space offset from sensor centre,
//                     in the Cartesian X direction.
//                     Add the gantry X position at scan time to get
//                     the global X coordinate.
//  CallusPos::y_mm  — gantry Y position passed to update().
//                     Already in global frame if gantry was zeroed
//                     before scanning.
//
//  SHADOW CLASSIFICATION (by pixel width)
//  ----------------------------------------
//    widthPx < MIN_CALLUS_WIDTH_PX           → noise, ignored
//    MIN_CALLUS_WIDTH_PX ≤ width ≤ MAX_CALLUS_WIDTH_PX → callus
//    width > MIN_DISH_WIDTH_PX               → dish body / rim
//
//  All thresholds are adjustable in the CONFIG section below.
// ============================================================

#include <Arduino.h>

namespace CallusDetect {

// ============================================================
//  CONFIG  —  adjust to match your hardware
// ============================================================
static constexpr uint8_t  PIXEL_COUNT         = 128;
static constexpr float    PIXEL_PITCH_MM      = 0.0635f;   // TSL1401: 63.5 µm pitch
static constexpr uint8_t  SENSOR_CENTER_PX    = 64;        // reference pixel (centre of array)

// Edge-finding parameters (same algorithm as test_tsl1401 / test_demo)
static constexpr uint8_t  EDGE_SKIP           = 5;         // ignore this many pixels at each end
static constexpr uint8_t  EDGE_CONFIRM        = 3;         // consecutive pixels to confirm an edge
static constexpr uint16_t MIN_CONTRAST        = 10;        // min (max-min) ADC range to detect anything

// Shadow classification thresholds (in pixels)
static constexpr uint8_t  MIN_CALLUS_WIDTH_PX = 3;         // narrower shadows are noise
static constexpr uint8_t  MAX_CALLUS_WIDTH_PX = 40;        // wider shadows are not a callus
static constexpr uint8_t  MIN_DISH_WIDTH_PX   = 60;        // shadows wider than this = dish

// Callus merging: detections within this distance in both X and Y
// are averaged into one callus (handles the same callus seen on
// consecutive frames as the gantry advances).
static constexpr float    CALLUS_MERGE_MM     = 3.0f;

static constexpr uint8_t  MAX_CALLUSES        = 30;

// ============================================================
//  DATA STRUCTURES
// ============================================================
struct CallusPos {
    float x_mm;   // X offset from sensor centre at the time of scan
    float y_mm;   // gantry Y position when the callus was detected
};

struct DishInfo {
    float center_x_mm;   // average sensor-centre-relative X of dish body
    float center_y_mm;   // midpoint of first and last dish detection in Y
    float radius_mm;     // half the maximum detected shadow width
    bool  valid;         // false if no dish pixels were ever seen
};

// ============================================================
//  INTERNAL STATE  (file-scope statics inside the namespace)
// ============================================================
static CallusPos _calluses[MAX_CALLUSES];
static uint8_t   _callusCount        = 0;

static float     _dishFirstY         = 0.0f;
static float     _dishLastY          = 0.0f;
static float     _dishCenterXAcc     = 0.0f;
static uint16_t  _dishCenterXCount   = 0;
static float     _dishMaxWidthMM     = 0.0f;
static bool      _dishSeenThisFrame  = false;

// ============================================================
//  INTERNAL: edge finder
//  Returns shadow width in pixels, or 0 if contrast too low /
//  edges not found. Writes left/right pixel indices.
// ============================================================
static uint8_t _findEdges(const uint16_t* pixels,
                           uint8_t* out_left,
                           uint8_t* out_right)
{
    // 1. Dynamic min/max in the valid pixel window
    uint16_t maxV = pixels[EDGE_SKIP] >> 2;
    uint16_t minV = maxV;
    for (uint8_t i = EDGE_SKIP; i < PIXEL_COUNT - EDGE_SKIP; i++) {
        uint16_t v = pixels[i] >> 2;
        if (v > maxV) maxV = v;
        if (v < minV) minV = v;
    }
    if (maxV - minV < MIN_CONTRAST) return 0;
    uint16_t thr = (maxV + minV) / 2;

    // 2. Left edge: scan L→R for EDGE_CONFIRM bright pixels followed
    //    by EDGE_CONFIRM dark pixels (bright = unobstructed background,
    //    dark = shadow cast by callus / dish under the laser line).
    uint8_t left = 0;
    bool    lFound = false;
    for (uint8_t i = EDGE_SKIP;
         i < PIXEL_COUNT - EDGE_SKIP - EDGE_CONFIRM * 2;
         i++) {
        bool above = true, below = true;
        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if ((pixels[i + k]             >> 2) <= thr) above = false;
            if ((pixels[i + EDGE_CONFIRM + k] >> 2) >  thr) below = false;
        }
        if (above && below) { left = i; lFound = true; break; }
    }

    // 3. Right edge: scan R→L for EDGE_CONFIRM dark then EDGE_CONFIRM bright
    uint8_t right = PIXEL_COUNT - 1;
    bool    rFound = false;
    for (int16_t j = static_cast<int16_t>(PIXEL_COUNT - EDGE_SKIP - 1);
         j >= static_cast<int16_t>(EDGE_SKIP + EDGE_CONFIRM * 2);
         j--) {
        bool below = true, above = true;
        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if ((pixels[j - k]           >> 2) >= thr) below = false;
            if ((pixels[j + EDGE_CONFIRM - k] >> 2) <  thr) above = false;
        }
        if (below && above) {
            right = static_cast<uint8_t>(j);
            rFound = true;
            break;
        }
    }

    if (!lFound || !rFound || right <= left) return 0;
    *out_left  = left;
    *out_right = right;
    return static_cast<uint8_t>(right - left);
}

// ============================================================
//  PUBLIC API
// ============================================================

// Reset all accumulated data. Call before starting a new scan pass.
inline void reset()
{
    _callusCount       = 0;
    _dishFirstY        = 0.0f;
    _dishLastY         = 0.0f;
    _dishCenterXAcc    = 0.0f;
    _dishCenterXCount  = 0;
    _dishMaxWidthMM    = 0.0f;
    _dishSeenThisFrame = false;
}

// Process one captured frame.
//   pixels       — raw 10-bit ADC array, PIXEL_COUNT elements
//   gantry_y_mm  — gantry Y position at the moment of capture
//   detectCallus — if true, classify narrow shadows as calluses
//   detectDish   — if true, accumulate wide shadows as dish outline
inline void update(const uint16_t* pixels,
                   float            gantry_y_mm,
                   bool             detectCallus = true,
                   bool             detectDish   = true)
{
    uint8_t left = 0, right = 0;
    uint8_t widthPx = _findEdges(pixels, &left, &right);

    if (widthPx == 0) {
        _dishSeenThisFrame = false;
        return;
    }

    uint8_t centerPx = static_cast<uint8_t>((left + right) / 2);
    float   centerMM = (static_cast<float>(centerPx)
                        - static_cast<float>(SENSOR_CENTER_PX))
                       * PIXEL_PITCH_MM;
    float   widthMM  = static_cast<float>(widthPx) * PIXEL_PITCH_MM;

    // ---- Dish body / rim ----
    if (detectDish && widthPx >= MIN_DISH_WIDTH_PX) {
        if (!_dishSeenThisFrame || _dishCenterXCount == 0) {
            if (_dishCenterXCount == 0) _dishFirstY = gantry_y_mm;
            _dishSeenThisFrame = true;
        }
        _dishLastY = gantry_y_mm;
        _dishCenterXAcc += centerMM;
        _dishCenterXCount++;
        if (widthMM > _dishMaxWidthMM) _dishMaxWidthMM = widthMM;
        return;  // don't double-classify as callus
    }
    _dishSeenThisFrame = false;

    // ---- Callus (narrow shadow) ----
    if (!detectCallus) return;
    if (widthPx < MIN_CALLUS_WIDTH_PX || widthPx > MAX_CALLUS_WIDTH_PX) return;

    // Merge with an existing nearby callus detection
    for (uint8_t i = 0; i < _callusCount; i++) {
        float dy = fabsf(_calluses[i].y_mm - gantry_y_mm);
        float dx = fabsf(_calluses[i].x_mm - centerMM);
        if (dy < CALLUS_MERGE_MM && dx < CALLUS_MERGE_MM) {
            // Running average of position
            _calluses[i].x_mm = (_calluses[i].x_mm + centerMM)    * 0.5f;
            _calluses[i].y_mm = (_calluses[i].y_mm + gantry_y_mm) * 0.5f;
            return;
        }
    }

    // New callus
    if (_callusCount < MAX_CALLUSES) {
        _calluses[_callusCount].x_mm = centerMM;
        _calluses[_callusCount].y_mm = gantry_y_mm;
        _callusCount++;
    }
}

// Returns the number of distinct callus positions found so far.
inline uint8_t getCallusCount() { return _callusCount; }

// Returns pointer to the callus position array (valid up to getCallusCount()).
inline const CallusPos* getCalluses() { return _calluses; }

// Returns dish information derived from accumulated wide-shadow detections.
inline DishInfo getDishInfo()
{
    DishInfo info;
    info.valid = (_dishCenterXCount > 0);
    if (!info.valid) {
        info.center_x_mm = 0.0f;
        info.center_y_mm = 0.0f;
        info.radius_mm   = 0.0f;
        return info;
    }
    info.center_x_mm = _dishCenterXAcc / static_cast<float>(_dishCenterXCount);
    info.center_y_mm = (_dishFirstY + _dishLastY) * 0.5f;
    info.radius_mm   = _dishMaxWidthMM * 0.5f;
    return info;
}

// Convenience: print all results to Serial.
inline void printResults()
{
    Serial.print(F("# CallusDetect: ")); Serial.print(_callusCount);
    Serial.println(F(" callus(es)"));
    for (uint8_t i = 0; i < _callusCount; i++) {
        Serial.print(F("#   [")); Serial.print(i);
        Serial.print(F("] x=")); Serial.print(_calluses[i].x_mm, 2);
        Serial.print(F(" mm  y=")); Serial.print(_calluses[i].y_mm, 2);
        Serial.println(F(" mm"));
    }
    DishInfo d = getDishInfo();
    if (d.valid) {
        Serial.print(F("# Dish: center=("));
        Serial.print(d.center_x_mm, 2); Serial.print(F(", "));
        Serial.print(d.center_y_mm, 2); Serial.print(F(") r="));
        Serial.print(d.radius_mm, 2);   Serial.println(F(" mm"));
    } else {
        Serial.println(F("# Dish: not detected"));
    }
}

} // namespace CallusDetect
