#include "state/StateSnapshot.h"
#include "core/ResourceMutex.h"
#include <ArduinoJson.h>

// ============================================================
//  StateSnapshot.cpp
// ============================================================

// ----------------------------------------------------------
//  Static storage for the ring-buffer log
// ----------------------------------------------------------
SnapshotLogEntry StateSnapshot::log_[SNAPSHOT_LOG_CAPACITY];
uint8_t                 StateSnapshot::logHead_ = 0;
uint8_t                 StateSnapshot::logSize_ = 0;

// ----------------------------------------------------------
StateSnapshot StateSnapshot::capture() {
    StateSnapshot snap;
    auto& ms = MachineState::instance();

    snap.gantry        = ms.getGantry();
    snap.gantryStepper[0] = ms.getGantryStepper(0);
    snap.gantryStepper[1] = ms.getGantryStepper(1);
    for (uint8_t i = 0; i < 4; i++) snap.leadScrew[i] = ms.getLeadScrew(i);
    snap.sts3215       = ms.getSTS3215();
    for (uint8_t i = 0; i < 4; i++) snap.mg996r[i] = ms.getMG996R(i);
    snap.dcMotor[0]    = ms.getDCMotor(0);
    snap.dcMotor[1]    = ms.getDCMotor(1);
    snap.sensors       = ms.getSensors();
    snap.status        = ms.getStatus();
    snap.timestampMs   = millis();

    return snap;
}

// ----------------------------------------------------------
//  Convert SystemStatus enum to a short string for JSON.
// ----------------------------------------------------------
static const char* statusStr(SystemStatus s) {
    switch (s) {
        case SystemStatus::IDLE:    return "IDLE";
        case SystemStatus::HOMING:  return "HOMING";
        case SystemStatus::RUNNING: return "RUNNING";
        case SystemStatus::PAUSED:  return "PAUSED";
        case SystemStatus::FAULT:   return "FAULT";
        case SystemStatus::E_STOP:  return "E_STOP";
        default:                    return "UNKNOWN";
    }
}

// ----------------------------------------------------------
void StateSnapshot::toJson(JsonVariant doc) const {
    doc["ts"]  = timestampMs;
    doc["sys"] = statusStr(status);

    // Gantry
    JsonObject g = doc.createNestedObject("gantry");
    g["x"]      = gantry.x;
    g["y"]      = gantry.y;
    g["vx"]     = gantry.xVelocity;
    g["vy"]     = gantry.yVelocity;
    g["moving"] = gantry.isMoving;
    g["homed"]  = gantry.isHomed;

    // Sensors
    JsonObject sen = doc.createNestedObject("sensors");
    sen["swX"]        = sensors.switchX.tripped;
    sen["swY"]        = sensors.switchY.tripped;
    sen["swZ"]        = sensors.switchZ.tripped;
    sen["laser"]      = sensors.laserOn;
    sen["ccdCenter"]  = sensors.ccdCenter;
    sen["ccdDetected"]= sensors.ccdTargetDetected;

    // DC motors (compact array)
    JsonArray motors = doc.createNestedArray("dcMotor");
    for (uint8_t i = 0; i < 2; i++) {
        JsonObject m = motors.createNestedObject();
        m["duty"] = dcMotor[i].dutyCycle;
        m["run"]  = dcMotor[i].isRunning;
        m["fwd"]  = dcMotor[i].dirForward;
    }

    // Current draw from ResourceMutex
    doc["ampDraw"] = ResourceMutex::instance().currentDrawAmps();
}

// ----------------------------------------------------------
void StateSnapshot::logEntry() {
    SnapshotLogEntry& e    = log_[logHead_];
    e.snap         = capture();
    e.timestampMs  = e.snap.timestampMs;

    logHead_ = (logHead_ + 1) % SNAPSHOT_LOG_CAPACITY;
    if (logSize_ < SNAPSHOT_LOG_CAPACITY) logSize_++;
}

// ----------------------------------------------------------
uint8_t StateSnapshot::logCount() {
    return logSize_;
}

// ----------------------------------------------------------
const SnapshotLogEntry& StateSnapshot::logAt(uint8_t index) {
    // index 0 = oldest.  The oldest entry in the ring is at
    // (logHead_ - logSize_ + CAPACITY) % CAPACITY.
    uint8_t base = static_cast<uint8_t>(
        (logHead_ - logSize_ + SNAPSHOT_LOG_CAPACITY) % SNAPSHOT_LOG_CAPACITY
    );
    uint8_t real = (base + index) % SNAPSHOT_LOG_CAPACITY;
    return log_[real];
}

// ----------------------------------------------------------
void StateSnapshot::clearLog() {
    logHead_ = 0;
    logSize_ = 0;
}