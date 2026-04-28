#include "operations/CalibrationOperation.h"
#include "control/MotorController.h"
#include "devices/OpenLoopStepper.h"
#include "devices/LaserSensor.h"
#include "devices/LineLaser.h"
#include "state/MachineState.h"
#include "state/StateSnapshot.h"
#include "core/ResourceMutex.h"

// ============================================================
//  CalibrationOperation.cpp
// ============================================================

CalibrationOperation::CalibrationOperation(MotorController* motor,
                                           OpenLoopStepper* zScrews[4],
                                           LaserSensor*     laser,
                                           LineLaser*       lineLaser,
                                           CalibData*       out,
                                           bool enableLaser,
                                           bool enableXY,
                                           bool enableZ)
    : Operation("CalibrationOperation")
    , motor_(motor)
    , laser_(laser)
    , lineLaser_(lineLaser)
    , out_(out)
    , enableLaser_(enableLaser)
    , enableXY_(enableXY)
    , enableZ_(enableZ)
{
    for (uint8_t i = 0; i < 4; i++) zScrews_[i] = zScrews[i];
}

// ----------------------------------------------------------
void CalibrationOperation::run() {
    status_ = OperationStatus::RUNNING;
    MachineState::instance().setStatus(SystemStatus::RUNNING);

    if (enableLaser_ && !shouldAbort()) calibrateLaser();
    if (enableXY_    && !shouldAbort()) calibrateAxes();
    if (enableZ_     && !shouldAbort()) calibrateZ();

    if (shouldAbort()) {
        status_ = OperationStatus::ABORTED;
    } else {
        status_ = OperationStatus::COMPLETE;
        StateSnapshot::logEntry();
    }

    MachineState::instance().setStatus(SystemStatus::IDLE);
}

// ----------------------------------------------------------
//  Phase 1 — Laser Sensor Calibration
//
//  With no object in the beam, captures CALIB_FRAMES and
//  averages the minimum pixel value across frames to
//  establish the dark noise floor.  The detection threshold
//  is set to noiseFloor × THRESH_FACTOR.
// ----------------------------------------------------------
void CalibrationOperation::calibrateLaser() {
    // Move gantry to calibration reference position
    if (CALIB_X_MM > 0.0f && CALIB_Y_MM > 0.0f) {
        motor_->moveTo(CALIB_X_MM, CALIB_Y_MM);
        if (!waitForMotionComplete(10000)) return;
    }

    if (shouldAbort()) return;

    // Warm up the laser
    lineLaser_->on();
    waitMs(100);

    // Capture background frames (no object)
    float minSum = 0.0f;
    for (uint8_t f = 0; f < CALIB_FRAMES; f++) {
        if (shouldAbort()) { lineLaser_->off(); return; }
        laser_->capture();
        // Sum the minimum pixel value per frame as noise floor estimate
        const uint16_t* pix = laser_->getPixels();
        uint16_t minVal = pix[0];
        for (uint8_t p = 1; p < 128; p++) {
            if (pix[p] < minVal) minVal = pix[p];
        }
        minSum += static_cast<float>(minVal);
        waitMs(10);
    }
    lineLaser_->off();

    if (out_) {
        out_->laserNoiseFloor = minSum / CALIB_FRAMES;
        out_->laserThreshold  = out_->laserNoiseFloor * THRESH_FACTOR;
        out_->laserCalibDone  = true;
    }
}

// ----------------------------------------------------------
//  Phase 2 — XY Axis Straightness Check
//
//  Sweeps the gantry across each axis while sampling the
//  CCD centre pixel.  Stores per-sample deviation from 64
//  (the optical centre pixel of the 128-pixel array).
// ----------------------------------------------------------
void CalibrationOperation::calibrateAxes() {
    if (shouldAbort() || !out_) return;

    ActuatorGuard guard({Actuator::GANTRY_A, Actuator::GANTRY_B});
    if (!guard.acquired()) return;

    // Sample positions evenly spaced across each axis
    // Using DEVIATION_SAMPLES points
    const float xMax = Constants::Gantry::X_TRAVEL_MM;
    const float yMax = Constants::Gantry::Y_TRAVEL_MM;

    if (xMax <= 0.0f || yMax <= 0.0f) return;   // Constants not yet set

    // X sweep — hold Y at midpoint
    float yMid = yMax / 2.0f;
    for (uint8_t i = 0; i < CalibData::DEVIATION_SAMPLES; i++) {
        if (shouldAbort()) return;
        float x = (xMax / (CalibData::DEVIATION_SAMPLES - 1)) * i;
        motor_->moveTo(x, yMid);
        if (!waitForMotionComplete(10000)) return;
        waitMs(50);

        laser_->capture();
        laser_->findCenter();
        laser_->syncState();
        int8_t dev = static_cast<int8_t>(
            static_cast<int>(MachineState::instance().getSensors().ccdCenter) - 64
        );
        out_->xDeviation[i] = dev;
    }

    // Y sweep — hold X at midpoint
    float xMid = xMax / 2.0f;
    for (uint8_t i = 0; i < CalibData::DEVIATION_SAMPLES; i++) {
        if (shouldAbort()) return;
        float y = (yMax / (CalibData::DEVIATION_SAMPLES - 1)) * i;
        motor_->moveTo(xMid, y);
        if (!waitForMotionComplete(10000)) return;
        waitMs(50);

        laser_->capture();
        laser_->findCenter();
        laser_->syncState();
        int8_t dev = static_cast<int8_t>(
            static_cast<int>(MachineState::instance().getSensors().ccdCenter) - 64
        );
        out_->yDeviation[i] = dev;
    }

    out_->axisCalibDone = true;
}

// ----------------------------------------------------------
//  Phase 3 — Z Height Reference
//
//  TODO: Implement once Z-axis control and sensing strategy
//        is confirmed (current-sense stall detection vs.
//        a dedicated contact switch on the tool head).
//        For now this is a safe no-op.
// ----------------------------------------------------------
void CalibrationOperation::calibrateZ() {
    // TODO: Lower Z slowly, detect dish surface, record height.
    (void)zScrews_;
    if (out_) {
        out_->zCalibDone = false;   // Not yet implemented
    }
}
