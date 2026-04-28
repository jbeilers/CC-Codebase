#pragma once

#include <Arduino.h>
#include "pins/PinMap.h"
#include "state/MachineState.h"

// ============================================================
//  LineLaser.h  —  5V line laser module control.
//
//  This module is a simple digital output — HIGH = laser on,
//  LOW = laser off.
//
//  IMPORTANT — Safety notes:
//    - Never leave the laser on when the gantry is idle or when
//      the system is in FAULT / E_STOP state.
//    - The laser should only be enabled when a capture is
//      actively being taken by LaserSensor.
//    - autoSafetyOff_ (enabled by default) will automatically
//      turn the laser off if the system transitions to FAULT
//      or E_STOP on the next syncSafety() call.
//
//  Usage:
//    LineLaser laser;
//    laser.begin();
//    laser.on();           // Enable before LaserSensor::capture()
//    sensor.capture();
//    laser.off();          // Disable immediately after
//    laser.syncSafety();   // Call each control tick for auto-off
// ============================================================

class LineLaser {
public:
    static LineLaser& instance() {
        static LineLaser inst;
        return inst;
    }

    explicit LineLaser(int pin = Pins::LineLaser::ENABLE,
                       bool autoSafetyOff = true);

    // ----------------------------------------------------------
    //  Call once in setup() to configure the pin.
    //  Laser starts OFF.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Turn the laser on.
    //  Has no effect if the system is in FAULT or E_STOP and
    //  autoSafetyOff_ is true.
    // ----------------------------------------------------------
    void on();

    // ----------------------------------------------------------
    //  Turn the laser off.
    // ----------------------------------------------------------
    void off();

    // ----------------------------------------------------------
    //  Toggle the laser state.
    // ----------------------------------------------------------
    void toggle();

    // ----------------------------------------------------------
    //  Returns true if the laser is currently on.
    // ----------------------------------------------------------
    bool isOn() const { return active_; }

    // ----------------------------------------------------------
    //  Safety check: turns the laser off if the system has
    //  entered FAULT or E_STOP.
    //  Call each control loop tick when autoSafetyOff_ is true.
    // ----------------------------------------------------------
    void syncSafety();

private:
    int  pin_;
    bool active_         = false;
    bool autoSafetyOff_  = true;
};
