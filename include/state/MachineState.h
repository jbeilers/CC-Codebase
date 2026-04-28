#pragma once

#include <cstdint>
#include <TeensyThreads.h>
#include "core/ResourceMutex.h"

// ============================================================
//  MachineState.h  —  Single source of truth for the runtime
//                     state of all axes, actuators, and the
//                     gantry position.
//
//  RULES:
//    - No control logic here. Read/write only.
//    - All writes must go through the setters to maintain
//      thread safety via the internal mutex.
//    - The control thread writes; serial/UI threads read.
// ============================================================

// ----------------------------------------------------------
//  Per-stepper state
// ----------------------------------------------------------
struct StepperState {
    float    positionMM   = 0.0f;   // Current position in mm
    float    velocityMMS  = 0.0f;   // Current velocity in mm/s
    bool     isMoving     = false;
    bool     isHomed      = false;
};

// ----------------------------------------------------------
//  Per-servo state
// ----------------------------------------------------------
struct ServoState {
    float    angleDeg     = 0.0f;   // Current angle in degrees
    bool     isMoving     = false;
};

// ----------------------------------------------------------
//  Per-DC motor state
// ----------------------------------------------------------
struct DCMotorState {
    float    dutyCycle    = 0.0f;   // 0.0 – 1.0 normalized
    bool     isRunning    = false;
    bool     dirForward   = true;   // true = forward, false = reverse
};

// ----------------------------------------------------------
//  Gantry (CoreXY) state
//  Positions are in mm from the home corner.
// ----------------------------------------------------------
struct GantryState {
    float    x            = 0.0f;
    float    y            = 0.0f;
    float    xVelocity    = 0.0f;   // mm/s
    float    yVelocity    = 0.0f;   // mm/s
    bool     isMoving     = false;
    bool     isHomed      = false;
};

// ----------------------------------------------------------
//  Microswitch trip context
//  Tells the system whether a switch trip is expected (homing)
//  or unexpected (mid-task boundary violation → fault).
// ----------------------------------------------------------
enum class SwitchTripContext : uint8_t {
    NONE,       // Switch not tripped
    HOMING,     // Trip expected — used to set axis origin
    FAULT       // Trip unexpected — gantry left allowed frame
};

// ----------------------------------------------------------
//  Per-axis switch state
//  Each axis has a min (home) switch.
//  TODO: Add a MAX switch entry per axis if hard limit switches
//        are added to the far end of each axis travel.
// ----------------------------------------------------------
struct SwitchState {
    bool              tripped = false;
    SwitchTripContext context = SwitchTripContext::NONE;
};

// ----------------------------------------------------------
//  Sensor / switch state
// ----------------------------------------------------------
struct SensorState {
    SwitchState  switchX;               // Gantry X-axis home/limit switch
    SwitchState  switchY;               // Gantry Y-axis home/limit switch
    SwitchState  switchZ;               // Z-axis tool home/limit switch
    bool         laserOn           = false;

    // TSL1401 CCD sensor — updated by LaserSensor::syncState()
    uint8_t      ccdCenter         = 64;    // Center pixel index (0–127); 64 = optical center
    uint8_t      ccdThreshold      = 0;     // Last dynamic threshold (normalized 0–255)
    bool         ccdTargetDetected = false; // True when a valid dark region is detected

    // TODO: Add laserLinePosMM once mounting geometry and pixel→mm calibration confirmed
};

// ----------------------------------------------------------
//  System-level state
// ----------------------------------------------------------
enum class SystemStatus : uint8_t {
    IDLE,
    HOMING,
    RUNNING,
    PAUSED,
    FAULT,
    E_STOP
};

// ----------------------------------------------------------
//  MachineState  —  Aggregates all sub-states.
//
//  Usage (from any thread):
//    MachineState& state = MachineState::instance();
//    GantryState g = state.getGantry();
//    state.setGantry(g);
// ----------------------------------------------------------
class MachineState {
public:

    // Singleton accessor — one instance shared across all threads
    static MachineState& instance() {
        static MachineState inst;
        return inst;
    }

    // Disallow copy
    MachineState(const MachineState&)            = delete;
    MachineState& operator=(const MachineState&) = delete;

    // ----------------------------------------------------------
    //  Gantry
    // ----------------------------------------------------------
    GantryState getGantry() const {
        mutex_.lock();
        GantryState s = gantry_;
        mutex_.unlock();
        return s;
    }
    void setGantry(const GantryState& s) {
        mutex_.lock();
        gantry_ = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  CoreXY Steppers (2x closed-loop)
    //  Index 0 = Motor A, Index 1 = Motor B
    // ----------------------------------------------------------
    StepperState getGantryStepper(uint8_t idx) const {
        mutex_.lock();
        StepperState s = gantryStepper_[idx < 2 ? idx : 0];
        mutex_.unlock();
        return s;
    }
    void setGantryStepper(uint8_t idx, const StepperState& s) {
        mutex_.lock();
        if (idx < 2) gantryStepper_[idx] = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  Lead Screw Steppers (4x NEMA 17)
    //  Index 0–1 = Device 1 screws, Index 2–3 = Device 2 screws
    //  TODO: Update index comments once devices are named
    // ----------------------------------------------------------
    StepperState getLeadScrew(uint8_t idx) const {
        mutex_.lock();
        StepperState s = leadScrew_[idx < 4 ? idx : 0];
        mutex_.unlock();
        return s;
    }
    void setLeadScrew(uint8_t idx, const StepperState& s) {
        mutex_.lock();
        if (idx < 4) leadScrew_[idx] = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  STS3215 Smart Servo (1x)
    // ----------------------------------------------------------
    ServoState getSTS3215() const {
        mutex_.lock();
        ServoState s = sts3215_;
        mutex_.unlock();
        return s;
    }
    void setSTS3215(const ServoState& s) {
        mutex_.lock();
        sts3215_ = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  MG996R Servos (4x)
    //  Index 0–3
    // ----------------------------------------------------------
    ServoState getMG996R(uint8_t idx) const {
        mutex_.lock();
        ServoState s = mg996r_[idx < 4 ? idx : 0];
        mutex_.unlock();
        return s;
    }
    void setMG996R(uint8_t idx, const ServoState& s) {
        mutex_.lock();
        if (idx < 4) mg996r_[idx] = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  DC Motors (2x)
    //  Index 0 = Motor A, Index 1 = Motor B
    // ----------------------------------------------------------
    DCMotorState getDCMotor(uint8_t idx) const {
        mutex_.lock();
        DCMotorState s = dcMotor_[idx < 2 ? idx : 0];
        mutex_.unlock();
        return s;
    }
    void setDCMotor(uint8_t idx, const DCMotorState& s) {
        mutex_.lock();
        if (idx < 2) dcMotor_[idx] = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  Sensors / Switches
    // ----------------------------------------------------------
    SensorState getSensors() const {
        mutex_.lock();
        SensorState s = sensors_;
        mutex_.unlock();
        return s;
    }
    void setSensors(const SensorState& s) {
        mutex_.lock();
        sensors_ = s;
        mutex_.unlock();
    }

    // Aliases used by LaserSensor / LineLaser
    SensorState getSensorState() const { return getSensors(); }
    void setSensorState(const SensorState& s) { setSensors(s); }

    // ----------------------------------------------------------
    //  System status
    // ----------------------------------------------------------
    SystemStatus getStatus() const {
        mutex_.lock();
        SystemStatus s = status_;
        mutex_.unlock();
        return s;
    }
    // Alias used by LineLaser and other device drivers
    SystemStatus getSystemStatus() const { return getStatus(); }

    void setStatus(SystemStatus s) {
        mutex_.lock();
        status_ = s;
        mutex_.unlock();
    }

    // ----------------------------------------------------------
    //  E-stop — callable from any thread, highest priority.
    //  Sets status to E_STOP immediately.
    // ----------------------------------------------------------
    void eStop() {
        mutex_.lock();
        status_ = SystemStatus::E_STOP;
        mutex_.unlock();
        // Drop all actuator tokens so no device stays locked out
        // after an emergency stop.
        ResourceMutex::instance().releaseAll();
    }

private:
    MachineState() = default;

    mutable Threads::Mutex mutex_;

    GantryState   gantry_;
    StepperState  gantryStepper_[2];
    StepperState  leadScrew_[4];
    ServoState    sts3215_;
    ServoState    mg996r_[4];
    DCMotorState  dcMotor_[2];
    SensorState   sensors_;
    SystemStatus  status_ = SystemStatus::IDLE;
};