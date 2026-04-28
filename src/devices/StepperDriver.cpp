#include "devices/StepperDriver.h"

// ============================================================
//  StepperDriver.cpp
// ============================================================

StepperDriver::StepperDriver(int   stepPin,
                             int   dirPin,
                             float stepsPerMm,
                             bool  invertDir)
    : stepPin_(stepPin)
    , dirPin_(dirPin)
    , stepsPerMm_(stepsPerMm)
    , invertDir_(invertDir)
{}

// ----------------------------------------------------------
void StepperDriver::begin() {
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_,  OUTPUT);
    digitalWrite(stepPin_, LOW);
    setDirection(true);   // Default forward
}

// ----------------------------------------------------------
void StepperDriver::step() {
    // Emit one pulse — TB6600 latches on rising edge
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(TB6600_MIN_PULSE_US);
    digitalWrite(stepPin_, LOW);

    // Update position counter
    positionSteps_ += dirForward_ ? 1 : -1;
}

// ----------------------------------------------------------
void StepperDriver::setDirection(bool forward) {
    dirForward_ = forward;
    // Apply inversion if wired in reverse
    bool pinState = invertDir_ ? !forward : forward;
    digitalWrite(dirPin_, pinState ? HIGH : LOW);
}

// ----------------------------------------------------------
float StepperDriver::getPositionMM() const {
    return static_cast<float>(positionSteps_) / stepsPerMm_;
}

// ----------------------------------------------------------
void StepperDriver::setPositionMM(float mm) {
    positionSteps_ = static_cast<int32_t>(mm * stepsPerMm_);
}

// ----------------------------------------------------------
void StepperDriver::setTargetVelocity(long stepsPerSec) {
    targetVelocity_ = stepsPerSec;
    if (stepsPerSec == 0) {
        stepAccumulator_ = 0.0f;
    }
    setDirection(stepsPerSec >= 0);
}

// ----------------------------------------------------------
void StepperDriver::stepAtRate(float dt) {
    if (targetVelocity_ == 0) return;
    stepAccumulator_ += fabsf(static_cast<float>(targetVelocity_)) * dt;
    while (stepAccumulator_ >= 1.0f) {
        step();
        stepAccumulator_ -= 1.0f;
    }
}
