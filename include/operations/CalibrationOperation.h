#pragma once

#include "operations/Operation.h"
#include <cstdint>

// ============================================================
//  CalibrationOperation.h  —  Calibrate the system before
//                              a transfer session.
//
//  SEQUENCE
//  --------
//  Phase 1 — Laser Sensor Calibration
//    1. Move gantry to a known calibration reference position
//       (Constants::Gantry::CALIB_X_MM, CALIB_Y_MM — TODO).
//    2. Turn on line laser.
//    3. Capture N frames from TSL1401 with no object present
//       (background / dark baseline).
//    4. Capture N frames with a reference object in the beam.
//    5. Compute and store an empirical noise floor and threshold.
//       → Writes to a CalibData struct (defined below).
//    6. Turn off line laser.
//
//  Phase 2 — XY Axis Straightness Check
//    1. Drive X axis from 0 to TRAVEL_X_MM at constant speed
//       while sampling the CCD every tick.
//    2. Record any lateral deviation from centre pixel.
//    3. Repeat for Y axis.
//    4. Store the deviation map in CalibData.
//    → Used by TransferOperation to apply a straightness
//      correction to pick/place positions (future enhancement).
//
//  Phase 3 — Z Height Reference
//    1. Lower Z to dish surface (detected by current spike
//       on DC motor or by a limit switch — TODO: confirm).
//    2. Record this encoder count as the "dish surface" Z.
//    3. Retract to travel height.
//
//  All phases are optional — set enableLaser / enableXY /
//  enableZ in the constructor to skip phases not needed.
//
//  TODO: Add CALIB_X_MM and CALIB_Y_MM to Constants::Gantry
//        once the reference position is measured on the machine.
// ============================================================

class MotorController;
class OpenLoopStepper;
class LaserSensor;
class LineLaser;

// ----------------------------------------------------------
//  Calibration output data — populated by run(), read by
//  the TransferOperation and logging system afterwards.
// ----------------------------------------------------------
struct CalibData {
    // Laser sensor noise floor (ADC counts, 0–255 normalised)
    float laserNoiseFloor  = 0.0f;
    // Empirical detection threshold (set to noiseFloor * THRESH_FACTOR)
    float laserThreshold   = 0.0f;
    // True if laser calibration completed successfully
    bool  laserCalibDone   = false;

    // Lateral CCD deviation at each calibration point (pixels)
    // Indexed by sample number along the axis sweep
    static constexpr uint8_t DEVIATION_SAMPLES = 16;
    int8_t xDeviation[DEVIATION_SAMPLES] = {};   // Pixels off-centre along X
    int8_t yDeviation[DEVIATION_SAMPLES] = {};   // Pixels off-centre along Y
    bool   axisCalibDone = false;

    // Z height at dish surface (mm from Z home)
    float dishSurfaceZ_mm = 0.0f;
    bool  zCalibDone      = false;
};

class CalibrationOperation : public Operation {
public:
    // ----------------------------------------------------------
    //  @param motor        CoreXY gantry controller
    //  @param zScrews      4x NEMA 17 lead screw steppers
    //  @param laser        TSL1401 CCD sensor
    //  @param lineLaser    5V line laser
    //  @param out          Pointer to caller-owned CalibData;
    //                      written by run() on completion.
    //  @param enableLaser  Run Phase 1 (laser calibration)
    //  @param enableXY     Run Phase 2 (axis straightness)
    //  @param enableZ      Run Phase 3 (Z height reference)
    // ----------------------------------------------------------
    CalibrationOperation(MotorController* motor,
                         OpenLoopStepper* zScrews[4],
                         LaserSensor*     laser,
                         LineLaser*       lineLaser,
                         CalibData*       out,
                         bool enableLaser = true,
                         bool enableXY    = true,
                         bool enableZ     = false);

    void run() override;

private:
    void calibrateLaser();
    void calibrateAxes();
    void calibrateZ();

    MotorController* motor_;
    OpenLoopStepper* zScrews_[4];
    LaserSensor*     laser_;
    LineLaser*       lineLaser_;
    CalibData*       out_;

    bool enableLaser_;
    bool enableXY_;
    bool enableZ_;

    // Number of frames averaged per calibration phase
    static constexpr uint8_t  CALIB_FRAMES       = 16;
    // Threshold = noiseFloor * this factor
    static constexpr float    THRESH_FACTOR       = 1.5f;
    // Speed used for the axis sweep (slow, steady)
    static constexpr float    SWEEP_SPEED_MMS     = 5.0f;   // mm/s
    // TODO: Set calibration reference position in Constants::Gantry
    static constexpr float    CALIB_X_MM          = -1.0f;  // TODO
    static constexpr float    CALIB_Y_MM          = -1.0f;  // TODO
};
