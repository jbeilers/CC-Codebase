#pragma once

#include "operations/Operation.h"
#include "math/Constants.h"

// ============================================================
//  HomeOperation.h  —  Homes all axes in order: Z → Y → X.
//
//  Sequence:
//    1. Move Z axis (lead screws) to home slowly until
//       switchZ trips, then back off by BACKOFF_MM.
//    2. Move Y gantry toward home switch until switchY trips,
//       then back off.
//    3. Move X gantry toward home switch until switchX trips,
//       then back off.
//    4. Zero all position counters.
//    5. Set MachineState isHomed = true, status = IDLE.
//
//  Safety:
//    - If any move times out without a switch trip, the
//      operation fails and sets system status to FAULT.
//    - E-stop mid-homing is respected via shouldAbort().
//
//  Speeds:
//    HOMING_SPEED_MMS   — cautious speed toward switch
//    BACKOFF_MM         — distance to retract after trip
//
//  TODO: Confirm homing direction for each axis (toward which
//        end is the switch mounted?). Flip HOMING_DIR_* if needed.
// ============================================================

class MotorController;
class OpenLoopStepper;
class Microswitch;

class HomeOperation : public Operation {
public:
    // ----------------------------------------------------------
    //  @param motor       CoreXY MotorController (X and Y axes)
    //  @param zScrews     Array of 4 lead screw steppers for Z
    //  @param switchX     X-axis home switch
    //  @param switchY     Y-axis home switch
    //  @param switchZ     Z-axis home switch
    // ----------------------------------------------------------
    HomeOperation(MotorController* motor,
                  OpenLoopStepper* zScrews[4],
                  Microswitch*     switchX,
                  Microswitch*     switchY,
                  Microswitch*     switchZ);

    void run() override;

private:
    void homeZ();
    void homeY();
    void homeX();

    bool driveUntilSwitch(Microswitch* sw, float speedMMS,
                          uint32_t timeoutMs);

    MotorController* motor_;
    OpenLoopStepper* zScrews_[4];
    Microswitch*     switchX_;
    Microswitch*     switchY_;
    Microswitch*     switchZ_;

    // TODO: Confirm direction signs for your switch placement
    static constexpr float    HOMING_SPEED_MMS   = Constants::Gantry::HOMING_SPEED_MMS;
    static constexpr float    BACKOFF_MM          = 3.0f;           // TODO: Tune backoff distance after switch trips
    static constexpr uint32_t HOMING_TIMEOUT_MS  = 30000;          // 30s max per axis
};
