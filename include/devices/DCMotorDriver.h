#pragma once

#include <Arduino.h>
#include <cstdint>
#include "state/MachineState.h"
#include "pins/PinMap.h"
#include "math/Constants.h"

// ============================================================
//  DCMotorDriver.h  —  L298N dual H-bridge wrapper.
//
//  Controls two DC motors (Motor A and Motor B) through
//  one L298N driver board.
//
//  Wiring modes (selected automatically via PinMap ENA/ENB value):
//
//  EN-pin PWM  (ENA/ENB >= 0):
//    IN pins set constant direction; ENA/ENB carry the PWM signal.
//    Requires jumper removed and ENA/ENB wired to Teensy PWM pins.
//
//  IN-pin PWM  (ENA/ENB == -1, jumpers left on):
//    ENA/ENB held HIGH by jumper (always enabled).
//    Speed is achieved by PWM-ing the active IN pin while holding
//    the inactive IN pin LOW.  Brake and coast work as normal.
//
//  Truth table — IN-pin PWM forward at duty D:
//    IN1 = LOW (constant),  IN2 = PWM(D)   → variable forward
//  Truth table — IN-pin PWM reverse at duty D:
//    IN1 = PWM(D),          IN2 = LOW       → variable reverse
//  Brake:  IN1=HIGH, IN2=HIGH  (both constant HIGH)
//  Coast:  IN1=LOW,  IN2=LOW   (both constant LOW)
// ============================================================

// Motor index
enum class DCMotorIndex : uint8_t {
    A = 0,
    B = 1
};

class DCMotorDriver {
public:
    static DCMotorDriver& instance() {
        static DCMotorDriver inst;
        return inst;
    }

    // Disallow copy
    DCMotorDriver(const DCMotorDriver&)            = delete;
    DCMotorDriver& operator=(const DCMotorDriver&) = delete;

    // ----------------------------------------------------------
    //  Call once in setup() to configure all pin modes.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Set motor speed and direction.
    //  @param motor     DCMotorIndex::A or DCMotorIndex::B
    //  @param duty      Speed as 0.0–1.0 normalized duty cycle.
    //                   Negative values drive in reverse.
    //                   0.0 = coast (not brake).
    // ----------------------------------------------------------
    void set(DCMotorIndex motor, float duty);

    // ----------------------------------------------------------
    //  Actively brake a motor (both IN pins HIGH).
    //  Causes rapid deceleration via back-EMF.
    // ----------------------------------------------------------
    void brake(DCMotorIndex motor);

    // ----------------------------------------------------------
    //  Coast a motor (both IN pins LOW).
    //  Motor decelerates freely.
    // ----------------------------------------------------------
    void coast(DCMotorIndex motor);

    // ----------------------------------------------------------
    //  Stop both motors immediately (coast).
    // ----------------------------------------------------------
    void stopAll();

    // ----------------------------------------------------------
    //  Sync both motor states to MachineState.
    //  Call after any set/brake/coast call.
    // ----------------------------------------------------------
    void syncState();

private:
    DCMotorDriver() = default;

    // ----------------------------------------------------------
    //  Apply direction pins for a given motor
    // ----------------------------------------------------------
    void applyDirection(DCMotorIndex motor, bool forward);

    // ----------------------------------------------------------
    //  Apply PWM duty.
    //  EN-pin mode: writes to ENA/ENB.
    //  IN-pin mode: PWMs the active IN pin, holds inactive LOW.
    //  Reads forward_[idx] to know which IN pin is active.
    // ----------------------------------------------------------
    void applyPWM(DCMotorIndex motor, float duty);

    // Cached state for MachineState sync
    float duty_[2]       = {0.0f, 0.0f};
    bool  forward_[2]    = {true, true};
    bool  running_[2]    = {false, false};
};
