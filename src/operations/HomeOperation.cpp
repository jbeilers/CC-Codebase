#include "operations/HomeOperation.h"
#include "control/MotorController.h"
#include "devices/OpenLoopStepper.h"
#include "devices/Microswitch.h"
#include "state/MachineState.h"
#include "math/Constants.h"
#include <TeensyThreads.h>
#include <Arduino.h>

// ============================================================
//  HomeOperation.cpp
// ============================================================

HomeOperation::HomeOperation(MotorController* motor,
                             OpenLoopStepper* zScrews[4],
                             Microswitch*     switchX,
                             Microswitch*     switchY,
                             Microswitch*     switchZ)
    : Operation("Home")
    , motor_(motor)
    , switchX_(switchX)
    , switchY_(switchY)
    , switchZ_(switchZ)
{
    for (int i = 0; i < 4; i++) zScrews_[i] = zScrews[i];
}

// ----------------------------------------------------------
void HomeOperation::run() {
    status_ = OperationStatus::RUNNING;
    MachineState::instance().setStatus(SystemStatus::HOMING);

    homeZ();
    if (shouldAbort()) { status_ = OperationStatus::ABORTED; return; }
    if (status_ == OperationStatus::FAILED) return;

    homeY();
    if (shouldAbort()) { status_ = OperationStatus::ABORTED; return; }
    if (status_ == OperationStatus::FAILED) return;

    homeX();
    if (shouldAbort()) { status_ = OperationStatus::ABORTED; return; }
    if (status_ == OperationStatus::FAILED) return;

    // ---- Zero all positions ----
    motor_->resetPosition(0.0f, 0.0f);

    for (int i = 0; i < 4; i++) {
        zScrews_[i]->zeroPosition();
        StepperState s = MachineState::instance().getLeadScrew(i);
        s.isHomed    = true;
        s.positionMM = 0.0f;
        MachineState::instance().setLeadScrew(i, s);
    }

    // ---- Mark gantry homed ----
    GantryState g = MachineState::instance().getGantry();
    g.isHomed = true;
    g.x       = 0.0f;
    g.y       = 0.0f;
    MachineState::instance().setGantry(g);

    MachineState::instance().setStatus(SystemStatus::IDLE);
    status_ = OperationStatus::COMPLETE;
}

// ----------------------------------------------------------
//  homeZ — step all four lead screws toward the Z switch,
//  then back off by BACKOFF_MM once it trips.
//
//  Screws are driven via step() in a loop. The step interval
//  is derived from HOMING_SPEED_MMS and STEPS_PER_MM so the
//  crawl speed is predictable regardless of microstepping.
// ----------------------------------------------------------
void HomeOperation::homeZ() {
    const float    spm     = Constants::LeadScrews::STEPS_PER_MM;
    const float    speed   = HOMING_SPEED_MMS;
    // Time between steps in microseconds at homing speed
    const uint32_t stepUs  = (spm > 0.0f && speed > 0.0f)
                             ? static_cast<uint32_t>(1e6f / (spm * speed))
                             : 5000u;

    // TODO: Confirm false = toward home for your Z axis wiring
    for (int i = 0; i < 4; i++) zScrews_[i]->setDirection(false);

    uint32_t elapsed = 0;
    while (elapsed < HOMING_TIMEOUT_MS) {
        if (shouldAbort()) return;

        switchZ_->update();
        if (switchZ_->isTripped()) break;

        for (int i = 0; i < 4; i++) zScrews_[i]->step();
        delayMicroseconds(stepUs);
        elapsed += (stepUs / 1000u) + 1u;
    }

    if (!switchZ_->isTripped()) {
        MachineState::instance().setStatus(SystemStatus::FAULT);
        status_ = OperationStatus::FAILED;
        return;
    }

    // Back off BACKOFF_MM away from the switch
    const long backoffSteps = static_cast<long>(BACKOFF_MM * spm);
    for (int i = 0; i < 4; i++) zScrews_[i]->setDirection(true);
    for (long s = 0; s < backoffSteps; s++) {
        if (shouldAbort()) return;
        for (int i = 0; i < 4; i++) zScrews_[i]->step();
        delayMicroseconds(stepUs);
    }
}

// ----------------------------------------------------------
//  homeY — crawl the gantry toward the Y switch using
//  MotorController::homingStep(), then back off once tripped.
//  xTripped=true tells homingStep to only move in the Y direction.
// ----------------------------------------------------------
void HomeOperation::homeY() {
    const uint32_t POLL_MS = 2;
    uint32_t elapsed = 0;

    while (elapsed < HOMING_TIMEOUT_MS) {
        if (shouldAbort()) return;

        switchY_->update();
        if (switchY_->isTripped()) break;

        motor_->homingStep(/*xTripped=*/ true, /*yTripped=*/ false);

        threads.delay(POLL_MS);
        elapsed += POLL_MS;
    }

    if (!switchY_->isTripped()) {
        motor_->stop();
        MachineState::instance().setStatus(SystemStatus::FAULT);
        status_ = OperationStatus::FAILED;
        return;
    }

    motor_->stop();

    // Back off BACKOFF_MM in +Y
    motor_->moveTo(motor_->getX(), motor_->getY() + BACKOFF_MM);
    waitForMotionComplete(5000);
}

// ----------------------------------------------------------
//  homeX — same pattern as homeY, but for the X axis.
//  yTripped=true tells homingStep to only move in the X direction.
// ----------------------------------------------------------
void HomeOperation::homeX() {
    const uint32_t POLL_MS = 2;
    uint32_t elapsed = 0;

    while (elapsed < HOMING_TIMEOUT_MS) {
        if (shouldAbort()) return;

        switchX_->update();
        if (switchX_->isTripped()) break;

        motor_->homingStep(/*xTripped=*/ false, /*yTripped=*/ true);

        threads.delay(POLL_MS);
        elapsed += POLL_MS;
    }

    if (!switchX_->isTripped()) {
        motor_->stop();
        MachineState::instance().setStatus(SystemStatus::FAULT);
        status_ = OperationStatus::FAILED;
        return;
    }

    motor_->stop();

    // Back off BACKOFF_MM in +X
    motor_->moveTo(motor_->getX() + BACKOFF_MM, motor_->getY());
    waitForMotionComplete(5000);
}
