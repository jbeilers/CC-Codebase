#pragma once

#include <Arduino.h>
#include "state/MachineState.h"
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  Microswitch.h  —  Debounced mechanical switch reader.
//
//  Each Microswitch instance owns one physical pin and one
//  axis's SwitchState in MachineState. It operates in two
//  modes depending on SystemStatus:
//
//    HOMING  → A trip is expected. On trip, sets SwitchState
//              context to HOMING so HomeOperation can zero
//              the axis and stop the motor.
//
//    RUNNING → A trip is unexpected. On trip, sets context
//              to FAULT and calls MachineState::eStop()
//              immediately.
//
//  Wiring assumption: Normally-open, INPUT_PULLUP.
//    Pin reads HIGH when open (not tripped).
//    Pin reads LOW  when closed (tripped).
//  TODO: If any switch is wired normally-closed, invert the
//        logic by setting invertLogic = true in the constructor.
// ============================================================

// Which axis this switch belongs to — used for MachineState writes
enum class SwitchAxis : uint8_t {
    X,
    Y,
    Z
};

class Microswitch {
public:
    // ----------------------------------------------------------
    //  Constructor
    //  @param pin         GPIO pin number (from PinMap)
    //  @param axis        Which axis this switch guards
    //  @param invertLogic Set true if wired normally-closed
    // ----------------------------------------------------------
    explicit Microswitch(int         pin,
                         SwitchAxis  axis,
                         bool        invertLogic = false);

    // ----------------------------------------------------------
    //  Call once in setup() to configure the pin mode
    // ----------------------------------------------------------
    void begin() const;

    // ----------------------------------------------------------
    //  Call on every control loop tick (or from a polling thread).
    //  Reads the pin, debounces, and updates MachineState.
    //  Also triggers eStop() if an unexpected trip is detected.
    // ----------------------------------------------------------
    void update();

    // ----------------------------------------------------------
    //  Returns true if the switch is currently tripped
    //  (after debounce). Does not update state — call update() first.
    // ----------------------------------------------------------
    [[nodiscard]] bool isTripped() const;

    // ----------------------------------------------------------
    //  Manually clear the switch state — call this after homing
    //  has consumed the trip event and moved the axis off the switch.
    // ----------------------------------------------------------
    void clearTrip();

    // ----------------------------------------------------------
    //  Returns the axis this switch belongs to
    // ----------------------------------------------------------
    [[nodiscard]] SwitchAxis axis() const { return axis_; }

private:
    // Resolves which SwitchState in MachineState to write to
    void writeSwitchState(bool tripped, SwitchTripContext context);

    int         pin_;
    SwitchAxis  axis_;
    bool        invertLogic_;

    // Debounce state
    bool        lastRawState_    = false;   // Last raw pin read
    bool        stableState_     = false;   // Debounced stable state
    uint32_t    lastChangeTimeMs_ = 0;      // Time of last raw state change (ms)
};
