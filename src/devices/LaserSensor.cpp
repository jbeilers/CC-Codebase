#include "devices/LaserSensor.h"

// ============================================================
//  LaserSensor.cpp
// ============================================================

// ----------------------------------------------------------
void LaserSensor::begin() {
    pinMode(Pins::LaserSensor::CLK, OUTPUT);
    pinMode(Pins::LaserSensor::SI,  OUTPUT);

    digitalWrite(Pins::LaserSensor::CLK, LOW);
    digitalWrite(Pins::LaserSensor::SI,  LOW);

    // Prime the pipeline: TSL1401 outputs previous-cycle data,
    // so first capture is invalid. Run a dummy frame to flush.
    capture();
    initialized_ = true;
}

// ----------------------------------------------------------
//  TSL1401 capture sequence (matches timing in docs / datasheet)
//
//  Step-by-step:
//    1. CLK=HIGH, SI=LOW  (ensure CLK starts high)
//    2. SI=HIGH, CLK=LOW  (SI must be high for first falling CLK edge)
//    3. CLK=HIGH, SI=LOW  (first CLK rising edge with SI high triggers cycle)
//    4. For each of 128 pixels:
//         CLK=LOW  → AO becomes valid (pixel output on falling edge)
//         ADC sample
//         CLK=HIGH
// ----------------------------------------------------------
void LaserSensor::capture() {
    // Step 1: Starting state
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(CCD_CLK_HALF_US);

    // Step 2: Assert SI, drop CLK
    digitalWrite(Pins::LaserSensor::SI,  HIGH);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(CCD_CLK_HALF_US);

    // Step 3: Rise CLK, de-assert SI
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    digitalWrite(Pins::LaserSensor::SI,  LOW);
    delayMicroseconds(CCD_CLK_HALF_US);

    // Step 4: Read 128 pixels
    for (uint8_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        digitalWrite(Pins::LaserSensor::CLK, LOW);
        delayMicroseconds(CCD_CLK_HALF_US);   // AO settles after falling edge

        // Sample AO — analogRead returns 0–1023 on Teensy 4.1 default 10-bit
        pixels_[i] = static_cast<uint16_t>(analogRead(Pins::LaserSensor::AO));

        digitalWrite(Pins::LaserSensor::CLK, HIGH);
        delayMicroseconds(CCD_CLK_HALF_US);
    }
}

// ----------------------------------------------------------
//  Dynamic threshold algorithm.
//  Directly implements the algorithm from TSL1401 reference docs,
//  adapted to C++ with named constants.
//
//  The algorithm finds the dark region (object/laser line shadow)
//  as a dip in the pixel intensity waveform.
//
//  Returns center pixel index (0–127).
// ----------------------------------------------------------
uint8_t LaserSensor::findCenter() {
    // ---- 1. Find min and max in the valid pixel window ----
    uint16_t maxVal = normalize(pixels_[EDGE_SKIP]);
    uint16_t minVal = normalize(pixels_[EDGE_SKIP]);

    for (uint8_t i = EDGE_SKIP; i < (CCD_PIXEL_COUNT - EDGE_SKIP); i++) {
        uint16_t v = normalize(pixels_[i]);
        if (v > maxVal) maxVal = v;
        if (v < minVal) minVal = v;
    }

    // ---- 2. Compute dynamic threshold ----
    threshold_ = (maxVal + minVal) / 2;

    // If contrast is too low, no valid target — return last center
    // TODO: Tune minimum contrast threshold in Constants.h if needed
    if ((maxVal - minVal) < 10) {
        targetDetected_ = false;
        return lastCenter_;
    }

    // ---- 3. Find left jump edge (ABOVE → BELOW transition) ----
    // Search left to right: look for EDGE_CONFIRM pixels above,
    // followed immediately by EDGE_CONFIRM pixels below threshold.
    uint8_t leftEdge = 0;
    bool    leftFound = false;

    for (uint8_t i = EDGE_SKIP; i < (CCD_PIXEL_COUNT - EDGE_SKIP - (EDGE_CONFIRM * 2)); i++) {
        bool confirmAbove = true;
        bool confirmBelow = true;

        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if (normalize(pixels_[i + k]) <= threshold_)         confirmAbove = false;
            if (normalize(pixels_[i + EDGE_CONFIRM + k]) > threshold_) confirmBelow = false;
        }

        if (confirmAbove && confirmBelow) {
            leftEdge  = i;
            leftFound = true;
            break;
        }
    }

    // ---- 4. Find right jump edge (BELOW → ABOVE transition) ----
    // Search right to left: look for EDGE_CONFIRM pixels below,
    // followed (to the right) by EDGE_CONFIRM pixels above threshold.
    uint8_t rightEdge = CCD_PIXEL_COUNT - 1;
    bool    rightFound = false;

    for (int16_t j = (CCD_PIXEL_COUNT - EDGE_SKIP - 1);
         j >= (EDGE_SKIP + static_cast<int16_t>(EDGE_CONFIRM * 2));
         j--) {

        bool confirmBelow = true;
        bool confirmAbove = true;

        for (uint8_t k = 0; k < EDGE_CONFIRM; k++) {
            if (normalize(pixels_[j - k]) >= threshold_)          confirmBelow = false;
            if (normalize(pixels_[j + EDGE_CONFIRM - k]) < threshold_) confirmAbove = false;
        }

        if (confirmBelow && confirmAbove) {
            rightEdge = static_cast<uint8_t>(j);
            rightFound = true;
            break;
        }
    }

    // ---- 5. Validate and compute center ----
    if (!leftFound || !rightFound || rightEdge <= leftEdge) {
        // No valid dark region detected — hold last value
        targetDetected_ = false;
        return lastCenter_;
    }

    uint8_t newCenter = (leftEdge + rightEdge) / 2;

    // ---- 6. Stability guard: reject implausible jumps ----
    uint8_t delta = (newCenter > lastCenter_)
                    ? (newCenter - lastCenter_)
                    : (lastCenter_ - newCenter);

    if (delta > CENTER_JUMP_LIMIT) {
        // Jump too large — likely noise or sensor loss; keep last value
        targetDetected_ = false;
        return lastCenter_;
    }

    // Accept new center
    lastCenter_    = newCenter;
    targetDetected_ = true;
    return newCenter;
}

// ----------------------------------------------------------
uint16_t LaserSensor::getPixel(uint8_t index) const {
    if (index >= CCD_PIXEL_COUNT) return 0;
    return pixels_[index];
}

// ----------------------------------------------------------
void LaserSensor::syncState() {
    SensorState s = MachineState::instance().getSensorState();
    s.ccdCenter         = lastCenter_;
    s.ccdThreshold      = static_cast<uint8_t>(threshold_ & 0xFF);
    s.ccdTargetDetected = targetDetected_;
    MachineState::instance().setSensorState(s);
}

// ----------------------------------------------------------
void LaserSensor::clockPulse() {
    digitalWrite(Pins::LaserSensor::CLK, HIGH);
    delayMicroseconds(CCD_CLK_HALF_US);
    digitalWrite(Pins::LaserSensor::CLK, LOW);
    delayMicroseconds(CCD_CLK_HALF_US);
}
