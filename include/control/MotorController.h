#pragma once

#include "control/TrajectoryPlanner.h"
#include "devices/StepperDriver.h"
#include "math/Kinematics.h"
#include "math/Constants.h"

// ============================================================
//  MotorController.h  —  Open-loop CoreXY gantry controller.
//
//  Uses the TrajectoryPlanner (S-curve) to generate a smooth
//  velocity profile and commands both steppers directly from
//  the planned velocity — no encoder feedback, no Kalman filter,
//  no LQR.
//
//  Pipeline (called each control tick from ThreadManager):
//    1. TrajectoryPlanner::step(dt)
//       → reference pos/vel in Cartesian space
//    2. Kinematics::cartesianVelToMotorVel(vx, vy)
//       → motor A and B step rates (steps/s)
//    3. StepperDriver::setTargetVelocity + stepAtRate
//       → physical STEP pulses to TB6600 drivers
//    4. MachineState updated from planned position.
//
//  Homing:
//    homingStep() drives both motors at a fixed crawl speed
//    toward the endstops.  The control thread still calls
//    update() each tick to generate the actual step pulses.
// ============================================================

class MotorController {
public:
    MotorController() = default;

    void begin(StepperDriver& stepperA, StepperDriver& stepperB);

    // Queue a move to the given Cartesian target (mm).
    // Thread-safe: may be called from the serial thread.
    void moveTo(float x_mm, float y_mm);

    // Main update — call every control tick (1 kHz).
    // Returns true when the current move is complete.
    bool update();

    // Stop all motion and hold position.
    void stop();

    // Emergency stop — halt immediately.
    void eStop();

    // Execute one homing step; returns true when both endstops trip.
    bool homingStep(bool xTripped, bool yTripped);

    // Reset tracked position (call after homing or discontinuity).
    void resetPosition(float x_mm, float y_mm);

    // Planned position / velocity accessors (for telemetry)
    float getX()  const { return posX_; }
    float getY()  const { return posY_; }
    float getVx() const { return velX_; }
    float getVy() const { return velY_; }

    bool isMoving()   const { return !planner_.isDone(); }
    bool isEStopped() const { return eStopped_; }

private:
    TrajectoryPlanner planner_;

    StepperDriver* stepperA_ = nullptr;
    StepperDriver* stepperB_ = nullptr;

    // Planned position and velocity (updated each tick from planner)
    float posX_ = 0.0f;
    float posY_ = 0.0f;
    float velX_ = 0.0f;
    float velY_ = 0.0f;

    uint32_t lastUs_  = 0;
    volatile bool eStopped_ = false;
    bool          homing_   = false;
};
