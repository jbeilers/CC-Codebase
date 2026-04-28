#include "devices/ClosedLoopStepper.h"

// ============================================================
//  ClosedLoopStepper.cpp
// ============================================================

ClosedLoopStepper::ClosedLoopStepper(int     stepPin,
                                     int     dirPin,
                                     int     encPinA,
                                     int     encPinB,
                                     float   stepsPerMm,
                                     uint8_t stateIndex,
                                     int32_t encoderCountsPerRev,
                                     bool    invertDir)
    : StepperDriver(stepPin, dirPin, stepsPerMm, invertDir)
    , encoder_(encPinA, encPinB)
    , stateIndex_(stateIndex)
    , encoderCountsPerRev_(encoderCountsPerRev)
{
    // Derive encoder counts per mm from CPR and pulley/belt geometry.
    // This mirrors the STEPS_PER_MM derivation in Constants.h.
    // TODO: Confirm this formula matches your pulley tooth count and belt pitch.
    //       encoderCountsPerMm = CPR / (belt_pitch_mm * pulley_teeth)
    encoderCountsPerMm_ = static_cast<float>(encoderCountsPerRev_)
                          / (Constants::Gantry::BELT_PITCH_MM
                             * Constants::Gantry::PULLEY_TEETH);
}

// ----------------------------------------------------------
void ClosedLoopStepper::begin() {
    StepperDriver::begin();
    encoder_.write(0);   // Zero encoder on startup
}

// ----------------------------------------------------------
int32_t ClosedLoopStepper::getEncoderCount() {
    return encoder_.read();
}

// ----------------------------------------------------------
float ClosedLoopStepper::getEncoderPositionMM() {
    return static_cast<float>(encoder_.read()) / encoderCountsPerMm_;
}

// ----------------------------------------------------------
void ClosedLoopStepper::zeroPosition() {
    StepperDriver::zeroPosition();
    encoder_.write(0);
}

// ----------------------------------------------------------
float ClosedLoopStepper::getFollowingErrorMM() {
    float commandedMM = getPositionMM();        // From step counter
    float actualMM    = getEncoderPositionMM(); // From encoder
    return commandedMM - actualMM;
}

// ----------------------------------------------------------
void ClosedLoopStepper::syncState(bool isMoving, float velocityMMS) {
    StepperState s;
    s.positionMM  = getEncoderPositionMM();  // Encoder is authoritative
    s.velocityMMS = velocityMMS;
    s.isMoving    = isMoving;
    s.isHomed     = (encoder_.read() != 0 || !isMoving);
    MachineState::instance().setGantryStepper(stateIndex_, s);
}

// setTargetVelocity and stepAtRate are now on StepperDriver base class.