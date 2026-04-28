#include "devices/LineLaser.h"

// ============================================================
//  LineLaser.cpp
// ============================================================

LineLaser::LineLaser(int pin, bool autoSafetyOff)
    : pin_(pin)
    , autoSafetyOff_(autoSafetyOff)
{}

// ----------------------------------------------------------
void LineLaser::begin() {
    if (pin_ < 0) return;   // Pin not yet assigned in PinMap — no-op
    pinMode(pin_, OUTPUT);
    off();                   // Always start with laser off
}

// ----------------------------------------------------------
void LineLaser::on() {
    if (pin_ < 0) return;

    // Safety interlock: refuse to enable during fault states
    if (autoSafetyOff_) {
        SystemStatus status = MachineState::instance().getSystemStatus();
        if (status == SystemStatus::FAULT || status == SystemStatus::E_STOP) {
            return;
        }
    }

    digitalWrite(pin_, HIGH);
    active_ = true;
}

// ----------------------------------------------------------
void LineLaser::off() {
    if (pin_ < 0) return;
    digitalWrite(pin_, LOW);
    active_ = false;
}

// ----------------------------------------------------------
void LineLaser::toggle() {
    if (active_) off();
    else         on();
}

// ----------------------------------------------------------
void LineLaser::syncSafety() {
    if (!autoSafetyOff_ || !active_) return;

    SystemStatus status = MachineState::instance().getSystemStatus();
    if (status == SystemStatus::FAULT || status == SystemStatus::E_STOP) {
        off();
    }
}
