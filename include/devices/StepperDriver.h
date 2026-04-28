#pragma once

#include <Arduino.h>
#include <cstdint>
#include "state/MachineState.h"
#include "math/Constants.h"

// ============================================================
//  StepperDriver.h  —  Base class for all TB6600 STEP/DIR
//                      stepper motors.
//
//  Handles:
//    - Step pulse generation (direct GPIO toggling)
//    - Direction control
//    - Step-based position tracking
//    - mm <-> step conversion using Constants
//
//  Does NOT handle:
//    - Velocity profiling (owned by TrajectoryPlanner)
//    - Encoder feedback (owned by ClosedLoopStepper subclass)
//    - MachineState writes (owned by subclass — base has no
//      knowledge of which index it occupies)
//
//  Timing note:
//    step() must be called at the correct rate by the control
//    loop to achieve the desired velocity. The base class does
//    not self-time — it pulses exactly once per call.
//    TB6600 minimum pulse width is 2.2µs — respected via
//    delayMicroseconds() in step().
// ============================================================

// Minimum STEP pulse high time in microseconds per TB6600 datasheet
// TODO: Verify against your specific TB6600 variant's datasheet
constexpr uint32_t TB6600_MIN_PULSE_US = 3;

class StepperDriver {
public:
    // ----------------------------------------------------------
    //  Constructor
    //  @param stepPin   GPIO pin connected to TB6600 PUL+
    //  @param dirPin    GPIO pin connected to TB6600 DIR+
    //  @param stepsPerMm Steps per mm (from Constants)
    //  @param invertDir  Set true to invert direction logic
    // ----------------------------------------------------------
    StepperDriver(int      stepPin,
                  int      dirPin,
                  float    stepsPerMm,
                  bool     invertDir = false);

    virtual ~StepperDriver() = default;

    // ----------------------------------------------------------
    //  Call once in setup() to configure pin modes
    // ----------------------------------------------------------
    virtual void begin();

    // ----------------------------------------------------------
    //  Emit one STEP pulse in the current direction.
    //  Call this at the desired step rate from the control loop.
    //  Updates internal step counter.
    // ----------------------------------------------------------
    virtual void step();

    // ----------------------------------------------------------
    //  Set direction: true = forward, false = reverse.
    //  Takes invertDir into account automatically.
    // ----------------------------------------------------------
    void setDirection(bool forward);

    // ----------------------------------------------------------
    //  Get/set position in steps (raw, from step counter)
    // ----------------------------------------------------------
    int32_t getPositionSteps() const { return positionSteps_; }
    void    setPositionSteps(int32_t steps) { positionSteps_ = steps; }

    // ----------------------------------------------------------
    //  Get/set position in mm (converted via stepsPerMm_)
    // ----------------------------------------------------------
    float getPositionMM() const;
    void  setPositionMM(float mm);

    // ----------------------------------------------------------
    //  Zero the position counter — call after homing
    // ----------------------------------------------------------
    void zeroPosition() { positionSteps_ = 0; }

    // ----------------------------------------------------------
    //  Current direction state
    // ----------------------------------------------------------
    bool isMovingForward() const { return dirForward_; }

    // ----------------------------------------------------------
    //  Velocity-mode stepping.
    //  setTargetVelocity — store desired rate (steps/s).
    //    Positive = forward, negative = reverse, 0 = stop.
    //  stepAtRate — call each control tick (dt seconds) to emit
    //    the correct number of STEP pulses for the stored rate.
    //    Uses a fractional accumulator (Bresenham-style) so
    //    non-integer rates are correctly averaged over time.
    // ----------------------------------------------------------
    void setTargetVelocity(long stepsPerSec);
    void stepAtRate(float dt);

protected:
    int      stepPin_;
    int      dirPin_;
    float    stepsPerMm_;
    bool     invertDir_;
    bool     dirForward_    = true;
    int32_t  positionSteps_ = 0;    // Accumulated step count since last zero

    long     targetVelocity_  = 0;
    float    stepAccumulator_ = 0.0f;
};
