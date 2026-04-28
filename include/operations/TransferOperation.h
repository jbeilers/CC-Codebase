#pragma once

#include "operations/Operation.h"
#include <cstdint>

// ============================================================
//  TransferOperation.h  —  Transfer one or more small objects
//                          between petri dishes.
//
//  SEQUENCE (per object)
//  ---------------------
//    1. Acquire ResourceMutex tokens for all actuators in use.
//    2. Move gantry to source dish position (CoreXY).
//    3. Lower Z tool head (lead screws) to pick height.
//    4. Activate pick mechanism (STS3215 or MG996R depending
//       on tool config).
//    5. Raise Z to travel height.
//    6. Optionally scan with line laser + TSL1401 to confirm
//       object was picked (ccdTargetDetected).
//    7. Move gantry to destination dish position.
//    8. Lower Z to place height.
//    9. Activate place mechanism (release object).
//   10. Raise Z to travel height.
//   11. Log a StateSnapshot entry.
//   12. Release all actuator tokens.
//   13. Repeat for remaining objects (transferCount_).
//
//  POSITIONS
//  ---------
//  Source and destination are Cartesian (X, Y) coordinates
//  in mm, measured from the home corner.
//  Z heights are in mm from the Z home position.
//
//  TODO: Confirm pick/place mechanism — which servo actuates
//        the gripper/pipette/vacuum? Set PICK_SERVO_IDX and
//        PICK_ANGLE_DEG / RELEASE_ANGLE_DEG accordingly.
//
//  TODO: Confirm Z heights for your petri dish geometry.
//        TRAVEL_Z_MM, PICK_Z_MM, PLACE_Z_MM are placeholders.
//
//  TODO: If multiple objects are transferred in one session,
//        define source/destination arrays or a grid offset.
// ============================================================

class MotorController;
class OpenLoopStepper;
class ServoController;
class LaserSensor;
class LineLaser;

class TransferOperation : public Operation {
public:
    // ----------------------------------------------------------
    //  @param motor        CoreXY gantry controller
    //  @param zScrews      4x NEMA 17 lead screw steppers
    //  @param servos       ServoController singleton (for pick/place)
    //  @param laser        TSL1401 sensor (for pick confirmation)
    //  @param lineLaser    5V line laser module
    //  @param srcX / srcY  Source petri dish position (mm)
    //  @param dstX / dstY  Destination petri dish position (mm)
    //  @param count        Number of objects to transfer
    // ----------------------------------------------------------
    TransferOperation(MotorController* motor,
                      OpenLoopStepper* zScrews[4],
                      ServoController* servos,
                      LaserSensor*     laser,
                      LineLaser*       lineLaser,
                      float srcX, float srcY,
                      float dstX, float dstY,
                      uint8_t count = 1);

    void run() override;

private:
    // ----------------------------------------------------------
    //  Sub-steps (each returns false if shouldAbort())
    // ----------------------------------------------------------
    bool moveGantryTo(float x, float y, uint32_t timeoutMs = 10000);
    bool moveZTo(float z_mm, uint32_t timeoutMs = 10000);
    bool activatePick();
    bool activatePlace();
    bool confirmPick();   // Returns true if CCD detects object

    // ----------------------------------------------------------
    //  Z motion helpers — drives all 4 lead screws together
    // ----------------------------------------------------------
    void driveZ(float speedMMS);
    void stopZ();

    MotorController* motor_;
    OpenLoopStepper* zScrews_[4];
    ServoController* servos_;
    LaserSensor*     laser_;
    LineLaser*       lineLaser_;

    float   srcX_, srcY_;
    float   dstX_, dstY_;
    uint8_t transferCount_;

    // Z height constants (mm from Z home) — TODO: fill from measured geometry
    static constexpr float TRAVEL_Z_MM = -1.0f;   // TODO: Safe travel height above dishes
    static constexpr float PICK_Z_MM   = -1.0f;   // TODO: Height at which the pick occurs
    static constexpr float PLACE_Z_MM  = -1.0f;   // TODO: Height at which the place occurs

    // Servo config for pick/place — TODO: confirm tool mechanism
    static constexpr uint8_t PICK_SERVO_IDX    = 0;       // TODO: Which MG996R index actuates the gripper
    static constexpr float   PICK_ANGLE_DEG    = -1.0f;   // TODO: Servo angle to close/grip
    static constexpr float   RELEASE_ANGLE_DEG = -1.0f;   // TODO: Servo angle to open/release

    static constexpr uint32_t Z_TIMEOUT_MS     = 15000;   // Max time for a Z move
    static constexpr uint32_t SETTLE_MS        = 200;     // Wait after Z arrives before actuating
    static constexpr uint32_t PICK_DWELL_MS    = 500;     // Time to hold pick position
    static constexpr uint32_t PLACE_DWELL_MS   = 500;     // Time to hold place position
};
