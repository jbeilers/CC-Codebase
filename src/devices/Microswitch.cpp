#include "devices/Microswitch.h"

// ============================================================
//  Microswitch.cpp
// ============================================================

Microswitch::Microswitch(int pin, SwitchAxis axis, bool invertLogic)
    : pin_(pin)
    , axis_(axis)
    , invertLogic_(invertLogic)
{}

// ----------------------------------------------------------
void Microswitch::begin() const {
    pinMode(pin_, INPUT_PULLUP);
}

// ----------------------------------------------------------
void Microswitch::update() {
    // Raw read — LOW means tripped for normally-open + pullup wiring
    bool rawRead = (digitalRead(pin_) == LOW);
    if (invertLogic_) rawRead = !rawRead;

    const uint32_t now = millis();

    // Reset debounce timer on any raw state change
    if (rawRead != lastRawState_) {
        lastRawState_     = rawRead;
        lastChangeTimeMs_ = now;
        return;     // Don't commit until signal is stable
    }

    // State hasn't changed — check if it has been stable long enough
    if ((now - lastChangeTimeMs_) < Constants::Switches::DEBOUNCE_MS) {
        return;     // Still within debounce window
    }

    // Stable state confirmed — check if anything changed
    if (rawRead == stableState_) {
        return;     // No change, nothing to do
    }

    stableState_ = rawRead;

    if (!stableState_) {
        // Switch just released — clear trip state
        writeSwitchState(false, SwitchTripContext::NONE);
        return;
    }

    // ----------------------------------------------------------
    //  Switch just tripped — determine context from SystemStatus
    // ----------------------------------------------------------
    SystemStatus status = MachineState::instance().getStatus();

    if (status == SystemStatus::HOMING) {
        // Expected trip — signal HomeOperation to zero the axis
        writeSwitchState(true, SwitchTripContext::HOMING);

    } else if (status == SystemStatus::RUNNING ||
               status == SystemStatus::PAUSED) {
        // Unexpected trip — gantry has left its allowed frame
        writeSwitchState(true, SwitchTripContext::FAULT);
        MachineState::instance().eStop();

    } else {
        // IDLE, FAULT, E_STOP, etc. — record the trip but don't escalate
        writeSwitchState(true, SwitchTripContext::NONE);
    }
}

// ----------------------------------------------------------
bool Microswitch::isTripped() const {
    return stableState_;
}

// ----------------------------------------------------------
void Microswitch::clearTrip() {
    stableState_ = false;
    writeSwitchState(false, SwitchTripContext::NONE);
}

// ----------------------------------------------------------
void Microswitch::writeSwitchState(bool tripped, SwitchTripContext context) {
    SensorState sensors = MachineState::instance().getSensors();

    SwitchState sw;
    sw.tripped = tripped;
    sw.context = context;

    switch (axis_) {
        case SwitchAxis::X: sensors.switchX = sw; break;
        case SwitchAxis::Y: sensors.switchY = sw; break;
        case SwitchAxis::Z: sensors.switchZ = sw; break;
    }

    MachineState::instance().setSensors(sensors);
}
