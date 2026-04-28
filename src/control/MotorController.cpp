#include "control/MotorController.h"
#include "state/MachineState.h"
#include "math/Kinematics.h"
#include <Arduino.h>   // micros()

// ============================================================
//  MotorController.cpp  —  Open-loop CoreXY control
// ============================================================

void MotorController::begin(StepperDriver& stepperA, StepperDriver& stepperB) {
    stepperA_ = &stepperA;
    stepperB_ = &stepperB;
    planner_.begin();
    posX_ = posY_ = velX_ = velY_ = 0.0f;
    eStopped_ = false;
    homing_   = false;
    lastUs_   = micros();
}

// ----------------------------------------------------------
void MotorController::moveTo(float x_mm, float y_mm) {
    if (eStopped_) return;

    Vec2 clamped = Kinematics::clampToWorkspace(x_mm, y_mm);
    planner_.moveTo(clamped(0, 0), clamped(1, 0), posX_, posY_);
}

// ----------------------------------------------------------
bool MotorController::update() {
    if (eStopped_) return false;
    if (!stepperA_ || !stepperB_) return false;

    uint32_t nowUs = micros();
    float dt = static_cast<float>(nowUs - lastUs_) * 1e-6f;
    lastUs_ = nowUs;
    if (dt <= 0.0f || dt > 0.1f) dt = Constants::Timing::CONTROL_LOOP_DT;

    if (homing_) {
        // Velocities are set by homingStep(); just generate steps.
        stepperA_->stepAtRate(dt);
        stepperB_->stepAtRate(dt);
        return false;
    }

    // Get the next point on the planned trajectory
    TrajectoryState ts = planner_.step(dt);

    // Store planned position/velocity for telemetry and next moveTo() start
    posX_ = ts.pos(0, 0);
    posY_ = ts.pos(1, 0);
    velX_ = ts.vel(0, 0);
    velY_ = ts.vel(1, 0);

    // Convert Cartesian velocity → motor step rates and command steppers
    Vec2 motorVel = Kinematics::cartesianVelToMotorVel(velX_, velY_);
    stepperA_->setTargetVelocity(static_cast<long>(motorVel(0, 0)));
    stepperB_->setTargetVelocity(static_cast<long>(motorVel(1, 0)));
    stepperA_->stepAtRate(dt);
    stepperB_->stepAtRate(dt);

    // Publish to MachineState
    auto& ms = MachineState::instance();
    auto  gs = ms.getGantry();
    gs.x         = posX_;
    gs.y         = posY_;
    gs.xVelocity = velX_;
    gs.yVelocity = velY_;
    gs.isMoving  = !ts.done;
    ms.setGantry(gs);

    return ts.done;
}

// ----------------------------------------------------------
void MotorController::stop() {
    planner_.stop();
    velX_ = velY_ = 0.0f;
    if (stepperA_) stepperA_->setTargetVelocity(0);
    if (stepperB_) stepperB_->setTargetVelocity(0);
}

// ----------------------------------------------------------
void MotorController::eStop() {
    eStopped_ = true;
    velX_ = velY_ = 0.0f;
    if (stepperA_) stepperA_->setTargetVelocity(0);
    if (stepperB_) stepperB_->setTargetVelocity(0);
    planner_.stop();
}

// ----------------------------------------------------------
bool MotorController::homingStep(bool xTripped, bool yTripped) {
    homing_ = true;
    constexpr float HS  = Constants::Gantry::HOMING_SPEED_MMS;
    const float     spm = Constants::Gantry::STEPS_PER_MM;

    if (!xTripped) {
        // Both motors crawl in the same direction → moves carriage toward X home
        stepperA_->setTargetVelocity(static_cast<long>(-HS * spm));
        stepperB_->setTargetVelocity(static_cast<long>(-HS * spm));
    } else if (!yTripped) {
        // Opposite directions → moves carriage toward Y home
        stepperA_->setTargetVelocity(static_cast<long>(-HS * spm));
        stepperB_->setTargetVelocity(static_cast<long>( HS * spm));
    }

    if (xTripped && yTripped) {
        homing_ = false;
        stepperA_->setTargetVelocity(0);
        stepperB_->setTargetVelocity(0);
        stepperA_->zeroPosition();
        stepperB_->zeroPosition();
        resetPosition(0.0f, 0.0f);
        return true;
    }
    return false;
}

// ----------------------------------------------------------
void MotorController::resetPosition(float x_mm, float y_mm) {
    posX_ = x_mm;
    posY_ = y_mm;
    velX_ = velY_ = 0.0f;
    eStopped_ = false;
    planner_.stop();
}
