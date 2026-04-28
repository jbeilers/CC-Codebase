#pragma once

#include <cstdint>
#include <Arduino.h>

#include "ArduinoJson/Variant/JsonVariant.hpp"
#include "state/MachineState.h"

// ============================================================
//  StateSnapshot.h  —  Immutable point-in-time copy of the
//                      full machine state.
//
//  PURPOSE
//  -------
//  MachineState is a live, mutex-protected singleton.  Any
//  code that needs to read multiple fields atomically — such
//  as a serial telemetry reply, a session log entry, or a
//  pre/post-operation audit — would otherwise need to hold
//  the MachineState mutex across several reads, or risk
//  reading a partially-updated state.
//
//  StateSnapshot captures all fields under one mutex lock and
//  provides an immutable value type that can be passed freely
//  between threads with no further synchronisation.
//
//  USAGE
//  -----
//    // Take a snapshot from any thread:
//    StateSnapshot snap = StateSnapshot::capture();
//
//    // Read fields safely:
//    float x   = snap.gantry.x;
//    bool homed = snap.gantry.isHomed;
//
//    // Serialise to JSON for logging or serial reply:
//    JsonDocument doc;
//    snap.toJson(doc);
//    serializeJson(doc, Serial);
//    Serial.println();
//
//  LOGGING
//  -------
//  A lightweight session log is maintained in
//  StateSnapshot::log[].  Each entry records a snapshot
//  and the millis() timestamp it was taken.  The log wraps
//  around when full (ring buffer semantics).
//
//  Log an entry:
//    StateSnapshot::logEntry();          // Captures current state
//
//  Iterate log:
//    for (uint8_t i = 0; i < StateSnapshot::logCount(); i++) {
//        const LogEntry& e = StateSnapshot::logAt(i);
//        // e.snap, e.timestampMs
//    }
//
//  TODO: Set LOG_CAPACITY to suit available RAM.
//        At ~100 bytes per snapshot, 64 entries ≈ 6.4 kB.
// ============================================================

// Maximum number of log entries kept in RAM.
// Oldest entries are overwritten when the ring is full.
constexpr uint8_t SNAPSHOT_LOG_CAPACITY = 64;

// ============================================================
//  StateSnapshot  —  Value type, freely copyable.
// ============================================================
struct StateSnapshot {

    // ----------------------------------------------------------
    //  All sub-state fields — mirrors MachineState layout.
    // ----------------------------------------------------------
    GantryState   gantry;
    StepperState  gantryStepper[2];
    StepperState  leadScrew[4];
    ServoState    sts3215;
    ServoState    mg996r[4];
    DCMotorState  dcMotor[2];
    SensorState   sensors;
    SystemStatus  status;

    // millis() at the moment of capture
    uint32_t      timestampMs = 0;

    // ----------------------------------------------------------
    //  Capture the current MachineState into a new snapshot.
    //  Acquires the MachineState mutex for the minimum time
    //  necessary to read each sub-state atomically.
    // ----------------------------------------------------------
    static StateSnapshot capture();

    // ----------------------------------------------------------
    //  Serialise this snapshot into an ArduinoJson document.
    //  The caller is responsible for creating the document and
    //  sending/storing it afterwards.
    //
    //  Recommended document size: StaticJsonDocument<512>
    //  (gantry + sensors + dcMotor array + ampDraw fits in ~400 bytes)
    //
    //  Usage:
    //    StaticJsonDocument<512> doc;
    //    snap.toJson(doc.as<JsonVariant>());
    //    serializeJson(doc, Serial);
    //
    //  JSON shape:
    //  {
    //    "ts": 12345,
    //    "sys": "RUNNING",
    //    "gantry": { "x":10.0, "y":5.0, "vx":0.0, "vy":0.0,
    //                "moving":true, "homed":true },
    //    "sensors": {
    //      "swX":false, "swY":false, "swZ":false,
    //      "laser":true,
    //      "ccdCenter":63, "ccdDetected":true
    //    },
    //    "dcMotor": [ {"duty":0.5,"run":true}, {...} ],
    //    "ampDraw": 2.1
    //  }
    // ----------------------------------------------------------
    void toJson(ArduinoJson::JsonVariant doc) const;

    // ----------------------------------------------------------
    //  Log ring-buffer interface.
    //  LogEntry is defined below (after StateSnapshot is complete).
    // ----------------------------------------------------------

    // Capture the current state and append to the ring log.
    static void logEntry();

    // Number of entries currently in the log (0–SNAPSHOT_LOG_CAPACITY).
    static uint8_t logCount();

    // Access log entry by logical index: 0 = oldest, count-1 = newest.
    // Returns a reference into the internal ring buffer — valid until
    // clearLog() or another logEntry() call wraps around to this slot.
    static const struct SnapshotLogEntry& logAt(uint8_t index);

    // Clear all log entries.
    static void clearLog();

private:
    // ----------------------------------------------------------
    //  Ring-buffer log storage — static, lives in BSS segment.
    //  Defined as SnapshotLogEntry* to avoid the incomplete-type
    //  problem; storage is in StateSnapshot.cpp.
    // ----------------------------------------------------------
    static struct SnapshotLogEntry log_[SNAPSHOT_LOG_CAPACITY];
    static uint8_t                 logHead_;   // Next write index
    static uint8_t                 logSize_;   // Valid entries (≤ capacity)
};

// ============================================================
//  SnapshotLogEntry  —  Defined AFTER StateSnapshot is complete
//                       so StateSnapshot is no longer an
//                       incomplete type when used as a field.
//
//  Named SnapshotLogEntry (not StateSnapshot::LogEntry) to keep
//  the definition at file scope where the full type is available.
//
//  Usage:
//    const SnapshotLogEntry& e = StateSnapshot::logAt(i);
//    float x = e.snap.gantry.x;
// ============================================================
struct SnapshotLogEntry {
    StateSnapshot snap;
    uint32_t      timestampMs = 0;
};