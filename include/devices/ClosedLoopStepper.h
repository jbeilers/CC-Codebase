#pragma once

#include <Encoder.h>
#include "devices/StepperDriver.h"
#include "math/Constants.h"

// ============================================================
//  ClosedLoopStepper.h  —  17HS24-2004D-E1K closed-loop stepper.
//
//  Extends StepperDriver with quadrature encoder reading.
//  The TB6600 still receives STEP/DIR — the encoder is read
//  independently by the Teensy to provide position feedback
//  to the Kalman filter.
//
//  The encoder position is the authoritative position source.
//  The step counter from the base class serves as a cross-check.
//
//  Encoder library: paulstoffregen/Encoder
//    - Uses hardware quadrature decoding on Teensy 4.1
//    - Best performance when both pins are interrupt-capable
//    - All digital pins on Teensy 4.1 support interrupts
//
//  Usage:
//    ClosedLoopStepper motorA(
//        Pins::Gantry::MOTOR_A_STEP,
//        Pins::Gantry::MOTOR_A_DIR,
//        Pins::Gantry::MOTOR_A_ENC_A,   // TODO: Add to PinMap
//        Pins::Gantry::MOTOR_A_ENC_B,   // TODO: Add to PinMap
//        Constants::Gantry::STEPS_PER_MM,
//        0   // MachineState gantryStepper index
//    );
//    motorA.begin();
//    motorA.setDirection(true);
//    motorA.step();          // Called at step rate from control loop
//    motorA.syncState();     // Push encoder position to MachineState
// ============================================================

class ClosedLoopStepper : public StepperDriver {
public:
    // ----------------------------------------------------------
    //  Constructor
    //  @param stepPin        GPIO pin → TB6600 PUL+
    //  @param dirPin         GPIO pin → TB6600 DIR+
    //  @param encPinA        Encoder channel A pin
    //  @param encPinB        Encoder channel B pin
    //  @param stepsPerMm     From Constants::Gantry::STEPS_PER_MM
    //  @param stateIndex     Index into MachineState::gantryStepper[] (0–1)
    //  @param encoderCountsPerRev  Encoder CPR (counts per revolution)
    //                              TODO: Set to your encoder's CPR
    //                              (17HS24-2004D-E1K encoder is typically 1000 CPR)
    //  @param invertDir      Set true if motor is wired in reverse
    // ----------------------------------------------------------
    ClosedLoopStepper(int     stepPin,
                      int     dirPin,
                      int     encPinA,
                      int     encPinB,
                      float   stepsPerMm,
                      uint8_t stateIndex,
                      int32_t encoderCountsPerRev = 1000, // TODO: Confirm CPR from encoder datasheet
                      bool    invertDir = false);

    // ----------------------------------------------------------
    //  begin() — also zeros the encoder
    // ----------------------------------------------------------
    void begin() override;

    // ----------------------------------------------------------
    //  Read raw encoder count (quadrature ticks since last zero)
    // ----------------------------------------------------------
    int32_t getEncoderCount();

    // ----------------------------------------------------------
    //  Get encoder-derived position in mm.
    //  Uses encoderCountsPerMm_ derived from CPR and belt/pulley.
    // ----------------------------------------------------------
    float getEncoderPositionMM();

    // ----------------------------------------------------------
    //  Zero the encoder counter — call after homing.
    //  Also calls base class zeroPosition().
    // ----------------------------------------------------------
    void zeroPosition();

    // ----------------------------------------------------------
    //  Reset encoder to zero — alias used by MotorController
    //  after homing completes.
    // ----------------------------------------------------------
    void resetEncoder() { zeroPosition(); }

    // ----------------------------------------------------------
    //  setTargetVelocity / stepAtRate are inherited from StepperDriver.
    // ----------------------------------------------------------

    // ----------------------------------------------------------
    //  Compute following error: difference between commanded
    //  step position and actual encoder position (in mm).
    //  Large following error indicates a stall or missed steps.
    //  MotorController should trigger a FAULT if this exceeds
    //  Constants::Gantry::FOLLOWING_ERROR_FAULT_MM.
    // ----------------------------------------------------------
    float getFollowingErrorMM();

    // ----------------------------------------------------------
    //  Write encoder position and motion state to
    //  MachineState::gantryStepper[stateIndex].
    //  Call once per control loop tick.
    // ----------------------------------------------------------
    void syncState(bool isMoving = false, float velocityMMS = 0.0f);

private:
    Encoder  encoder_;
    uint8_t  stateIndex_;
    int32_t  encoderCountsPerRev_;
    float    encoderCountsPerMm_;    // Derived in constructor
    // targetVelocity_ and stepAccumulator_ are inherited from StepperDriver
};