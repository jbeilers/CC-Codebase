#include "devices/DCMotorDriver.h"

// ============================================================
//  DCMotorDriver.cpp
// ============================================================

void DCMotorDriver::begin() {
    // Motor A
    pinMode(Pins::DCMotors::MOTOR_A_IN1, OUTPUT);
    pinMode(Pins::DCMotors::MOTOR_A_IN2, OUTPUT);

    // Pre-initialise every IN pin's timer submodule at 50 Hz — the same
    // frequency the Arduino Servo library uses.  Without this, the first
    // analogWrite() call on any of these pins (when a motor command arrives)
    // initialises the submodule at Teensy's default ~4882 Hz.  If that pin
    // happens to share a FlexPWM submodule with a servo pin, the 50 Hz servo
    // signal is corrupted and the servo stops responding.  Setting the
    // frequency here (after ServoController::begin() has already run) locks
    // shared submodules at 50 Hz so subsequent analogWrite duty-cycle updates
    // never change the frequency.
    analogWriteFrequency(Pins::DCMotors::MOTOR_A_IN1, 50);
    analogWriteFrequency(Pins::DCMotors::MOTOR_A_IN2, 50);
    analogWrite(Pins::DCMotors::MOTOR_A_IN1, 0);
    analogWrite(Pins::DCMotors::MOTOR_A_IN2, 0);

    if (Pins::DCMotors::MOTOR_A_ENA >= 0) {
        pinMode(Pins::DCMotors::MOTOR_A_ENA, OUTPUT);
        analogWriteFrequency(Pins::DCMotors::MOTOR_A_ENA,
                             Constants::DCMotors::PWM_FREQ_HZ);
    }

    // Motor B
    pinMode(Pins::DCMotors::MOTOR_B_IN3, OUTPUT);
    pinMode(Pins::DCMotors::MOTOR_B_IN4, OUTPUT);

    analogWriteFrequency(Pins::DCMotors::MOTOR_B_IN3, 50);
    analogWriteFrequency(Pins::DCMotors::MOTOR_B_IN4, 50);
    analogWrite(Pins::DCMotors::MOTOR_B_IN3, 0);
    analogWrite(Pins::DCMotors::MOTOR_B_IN4, 0);

    if (Pins::DCMotors::MOTOR_B_ENB >= 0) {
        pinMode(Pins::DCMotors::MOTOR_B_ENB, OUTPUT);
        analogWriteFrequency(Pins::DCMotors::MOTOR_B_ENB,
                             Constants::DCMotors::PWM_FREQ_HZ);
    }

    stopAll();
}

// ----------------------------------------------------------
void DCMotorDriver::set(DCMotorIndex motor, float duty) {
    uint8_t idx = static_cast<uint8_t>(motor);

    if (duty >  Constants::DCMotors::MAX_DUTY) duty =  Constants::DCMotors::MAX_DUTY;
    if (duty < -Constants::DCMotors::MAX_DUTY) duty = -Constants::DCMotors::MAX_DUTY;

    bool forward = (duty >= 0.0f);
    float absDuty = fabsf(duty);

    forward_[idx] = forward;
    duty_[idx]    = absDuty;
    running_[idx] = (absDuty > 0.0f);

    applyDirection(motor, forward);
    applyPWM(motor, absDuty);
}

// ----------------------------------------------------------
void DCMotorDriver::brake(DCMotorIndex motor) {
    uint8_t idx = static_cast<uint8_t>(motor);
    running_[idx] = false;
    duty_[idx]    = 0.0f;

    if (motor == DCMotorIndex::A) {
        digitalWrite(Pins::DCMotors::MOTOR_A_IN1, HIGH);
        digitalWrite(Pins::DCMotors::MOTOR_A_IN2, HIGH);
    } else {
        digitalWrite(Pins::DCMotors::MOTOR_B_IN3, HIGH);
        digitalWrite(Pins::DCMotors::MOTOR_B_IN4, HIGH);
    }
}

// ----------------------------------------------------------
void DCMotorDriver::coast(DCMotorIndex motor) {
    uint8_t idx = static_cast<uint8_t>(motor);
    running_[idx] = false;
    duty_[idx]    = 0.0f;

    if (motor == DCMotorIndex::A) {
        digitalWrite(Pins::DCMotors::MOTOR_A_IN1, LOW);
        digitalWrite(Pins::DCMotors::MOTOR_A_IN2, LOW);
        if (Pins::DCMotors::MOTOR_A_ENA >= 0)
            analogWrite(Pins::DCMotors::MOTOR_A_ENA, 0);
    } else {
        digitalWrite(Pins::DCMotors::MOTOR_B_IN3, LOW);
        digitalWrite(Pins::DCMotors::MOTOR_B_IN4, LOW);
        if (Pins::DCMotors::MOTOR_B_ENB >= 0)
            analogWrite(Pins::DCMotors::MOTOR_B_ENB, 0);
    }
}

// ----------------------------------------------------------
void DCMotorDriver::stopAll() {
    coast(DCMotorIndex::A);
    coast(DCMotorIndex::B);
}

// ----------------------------------------------------------
void DCMotorDriver::syncState() {
    for (uint8_t i = 0; i < 2; i++) {
        DCMotorState s;
        s.dutyCycle  = duty_[i];
        s.isRunning  = running_[i];
        s.dirForward = forward_[i];
        MachineState::instance().setDCMotor(i, s);
    }
}

// ----------------------------------------------------------
void DCMotorDriver::applyDirection(DCMotorIndex motor, bool forward) {
    // Sets the inactive IN pin LOW; the active IN pin is set HIGH here
    // but will be immediately overridden by analogWrite in applyPWM
    // when in IN-pin PWM mode.
    if (motor == DCMotorIndex::A) {
        digitalWrite(Pins::DCMotors::MOTOR_A_IN1, forward ? LOW  : HIGH);
        digitalWrite(Pins::DCMotors::MOTOR_A_IN2, forward ? HIGH : LOW);
    } else {
        digitalWrite(Pins::DCMotors::MOTOR_B_IN3, forward ? LOW  : HIGH);
        digitalWrite(Pins::DCMotors::MOTOR_B_IN4, forward ? HIGH : LOW);
    }
}

// ----------------------------------------------------------
void DCMotorDriver::applyPWM(DCMotorIndex motor, float duty) {
    uint8_t idx = static_cast<uint8_t>(motor);
    int pwmVal  = static_cast<int>(duty * 255.0f);
    bool fwd    = forward_[idx];

    if (motor == DCMotorIndex::A) {
        if (Pins::DCMotors::MOTOR_A_ENA >= 0) {
            // EN-pin mode: direction set by applyDirection; speed on ENA
            analogWrite(Pins::DCMotors::MOTOR_A_ENA, pwmVal);
        } else {
            // IN-pin PWM mode: applyDirection already set the inactive pin LOW.
            // Override the active IN pin with the PWM duty cycle.
            // forward → IN2 is active (IN1 already LOW from applyDirection)
            // reverse → IN1 is active (IN2 already LOW from applyDirection)
            if (fwd)
                analogWrite(Pins::DCMotors::MOTOR_A_IN2, pwmVal);
            else
                analogWrite(Pins::DCMotors::MOTOR_A_IN1, pwmVal);
        }
    } else {
        if (Pins::DCMotors::MOTOR_B_ENB >= 0) {
            analogWrite(Pins::DCMotors::MOTOR_B_ENB, pwmVal);
        } else {
            if (fwd)
                analogWrite(Pins::DCMotors::MOTOR_B_IN4, pwmVal);
            else
                analogWrite(Pins::DCMotors::MOTOR_B_IN3, pwmVal);
        }
    }
}
