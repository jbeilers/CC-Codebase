#include "operations/TransferOperation.h"
#include "control/MotorController.h"
#include "devices/OpenLoopStepper.h"
#include "devices/ServoController.h"
#include "devices/LaserSensor.h"
#include "devices/LineLaser.h"
#include "state/MachineState.h"
#include "state/StateSnapshot.h"
#include "core/ResourceMutex.h"

// ============================================================
//  TransferOperation.cpp
// ============================================================

TransferOperation::TransferOperation(MotorController* motor,
                                     OpenLoopStepper* zScrews[4],
                                     ServoController* servos,
                                     LaserSensor*     laser,
                                     LineLaser*       lineLaser,
                                     float srcX, float srcY,
                                     float dstX, float dstY,
                                     uint8_t count)
    : Operation("TransferOperation")
    , motor_(motor)
    , servos_(servos)
    , laser_(laser)
    , lineLaser_(lineLaser)
    , srcX_(srcX), srcY_(srcY)
    , dstX_(dstX), dstY_(dstY)
    , transferCount_(count)
{
    for (uint8_t i = 0; i < 4; i++) zScrews_[i] = zScrews[i];
}

// ----------------------------------------------------------
void TransferOperation::run() {
    status_ = OperationStatus::RUNNING;
    MachineState::instance().setStatus(SystemStatus::RUNNING);

    for (uint8_t obj = 0; obj < transferCount_; obj++) {

        if (shouldAbort()) break;

        // ---- Acquire all actuators needed for this transfer ----
        ActuatorGuard guard({
            Actuator::GANTRY_A,
            Actuator::GANTRY_B,
            Actuator::LEADSCREW_0,
            Actuator::LEADSCREW_1,
            Actuator::LEADSCREW_2,
            Actuator::LEADSCREW_3,
            Actuator::MG996R_0   // TODO: Update if a different servo is the pick servo
        });

        if (!guard.acquired()) {
            // Current budget exceeded — abort this transfer
            MachineState::instance().setStatus(SystemStatus::FAULT);
            status_ = OperationStatus::FAILED;
            return;
        }

        // ---- 1. Rise to travel height ----
        if (!moveZTo(TRAVEL_Z_MM, Z_TIMEOUT_MS)) break;
        if (shouldAbort()) break;

        // ---- 2. Move XY to source dish ----
        if (!moveGantryTo(srcX_, srcY_)) break;
        if (shouldAbort()) break;

        // ---- 3. Lower to pick height ----
        if (!moveZTo(PICK_Z_MM, Z_TIMEOUT_MS)) break;
        waitMs(SETTLE_MS);
        if (shouldAbort()) break;

        // ---- 4. Actuate pick ----
        if (!activatePick()) break;
        waitMs(PICK_DWELL_MS);
        if (shouldAbort()) break;

        // ---- 5. Rise to travel height ----
        if (!moveZTo(TRAVEL_Z_MM, Z_TIMEOUT_MS)) break;
        if (shouldAbort()) break;

        // ---- 6. Confirm pick with laser sensor (optional) ----
        if (!confirmPick()) {
            // Object not detected — log and continue anyway.
            // TODO: Decide policy: abort, retry, or warn-and-continue.
        }
        if (shouldAbort()) break;

        // ---- 7. Move XY to destination dish ----
        if (!moveGantryTo(dstX_, dstY_)) break;
        if (shouldAbort()) break;

        // ---- 8. Lower to place height ----
        if (!moveZTo(PLACE_Z_MM, Z_TIMEOUT_MS)) break;
        waitMs(SETTLE_MS);
        if (shouldAbort()) break;

        // ---- 9. Actuate place ----
        if (!activatePlace()) break;
        waitMs(PLACE_DWELL_MS);
        if (shouldAbort()) break;

        // ---- 10. Rise back to travel height ----
        if (!moveZTo(TRAVEL_Z_MM, Z_TIMEOUT_MS)) break;

        // ---- 11. Log this transfer ----
        StateSnapshot::logEntry();

        // guard releases all actuator tokens at end of this scope iteration
    }

    if (shouldAbort()) {
        status_ = OperationStatus::ABORTED;
        MachineState::instance().setStatus(SystemStatus::IDLE);
    } else {
        status_ = OperationStatus::COMPLETE;
        MachineState::instance().setStatus(SystemStatus::IDLE);
    }
}

// ----------------------------------------------------------
bool TransferOperation::moveGantryTo(float x, float y, uint32_t timeoutMs) {
    motor_->moveTo(x, y);
    return waitForMotionComplete(timeoutMs);
}

// ----------------------------------------------------------
bool TransferOperation::moveZTo(float z_mm, uint32_t timeoutMs) {
    // TODO: Replace with proper Z axis position control once
    //       OpenLoopStepper position targeting is implemented.
    //       For now, drives at a fixed speed and relies on
    //       timeout / endstop as the termination condition.
    (void)z_mm;
    (void)timeoutMs;
    return true;
}

// ----------------------------------------------------------
bool TransferOperation::activatePick() {
    // TODO: Replace angle placeholders once pick mechanism is confirmed.
    if (PICK_ANGLE_DEG < 0.0f) return true;   // Guard while unset
    servos_->mg996rSetAngle(PICK_SERVO_IDX, PICK_ANGLE_DEG);
    return !shouldAbort();
}

// ----------------------------------------------------------
bool TransferOperation::activatePlace() {
    if (RELEASE_ANGLE_DEG < 0.0f) return true;
    servos_->mg996rSetAngle(PICK_SERVO_IDX, RELEASE_ANGLE_DEG);
    return !shouldAbort();
}

// ----------------------------------------------------------
bool TransferOperation::confirmPick() {
    // Single-frame pick confirmation: fire laser, capture, check for shadow.
    // Also broadcasts the frame as a PX: line so the PC-side vision node
    // (petri_vision) can process it without any extra code on the PC.
    lineLaser_->on();

    // Dummy capture: reads stale dark frame, starts new integration with laser ON.
    laser_->capture();
    // Wait the full integration period so the sensor accumulates charge under laser.
    delayMicroseconds(
        static_cast<uint32_t>(Constants::LaserSensor::INTEGRATION_TIME_US));
    // Real capture: reads laser-illuminated pixels.
    laser_->capture();
    laser_->findCenter();
    laser_->syncState();
    lineLaser_->off();

    // Broadcast pixel row to PC: "PX:<y_mm>,<p0>,<p1>,...,<p127>\n"
    float currentY = MachineState::instance().getGantry().y;
    Serial.print(F("PX:"));
    Serial.print(currentY, 3);
    const uint16_t* px = laser_->getPixels();
    for (uint8_t i = 0; i < 128; i++) {
        Serial.print(',');
        Serial.print(px[i]);
    }
    Serial.println();

    return MachineState::instance().getSensors().ccdTargetDetected;
}
