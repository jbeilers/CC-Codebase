// ============================================================
//  main.cpp  —  Teensy 4.1 CoreXY Petri Dish Transfer System
//
//  Responsibilities:
//    - Instantiate every device object (steppers, servos, etc.)
//    - Wire device objects into the control and serial callbacks
//    - Start ThreadManager (which starts all threads)
//    - loop() is intentionally minimal — all work is threaded
//
//  CHECKLIST BEFORE FIRST POWER-ON:
//    [ ] Fill all -1 values in PinMap.h
//    [ ] Fill all -1 values in Constants.h
//    [ ] Confirm motor wiring directions (invertDir flags below)
//    [ ] Confirm STS3215 servo ID in Constants::Servos::STS3215_SERVO_ID
//    [ ] Run homing before any transfer session
// ============================================================

#include <Arduino.h>

// Core infrastructure
#include "core/ThreadManager.h"
#include "core/SerialComm.h"
#include "core/ResourceMutex.h"

// Math / config
#include "pins/PinMap.h"
#include "math/Constants.h"
#include "math/Kinematics.h"

// Device drivers
#include "devices/OpenLoopStepper.h"
#include "devices/ServoController.h"
#include "devices/DCMotorDriver.h"
#include "devices/LaserSensor.h"
#include "devices/LineLaser.h"
#include "devices/Microswitch.h"

// Control stack
#include "control/MotorController.h"

// State
#include "state/MachineState.h"
#include "state/StateSnapshot.h"

// Operations
#include "operations/HomeOperation.h"
#include "operations/TransferOperation.h"
#include "operations/CalibrationOperation.h"

// ============================================================
//  Device instantiation
//  All objects are file-scope so they outlive setup/loop and
//  are accessible to lambdas registered with ThreadManager.
// ============================================================

// ----------------------------------------------------------
//  CoreXY gantry — open-loop steppers (encoder removed).
//  invertDir: flip to true for whichever motor runs the wrong
//  way on first power-on.  Check one axis at a time.
// ----------------------------------------------------------
static OpenLoopStepper motorA(
    Pins::Gantry::MOTOR_A_STEP,
    Pins::Gantry::MOTOR_A_DIR,
    Constants::Gantry::STEPS_PER_MM,
    /*stateIndex=*/ 0,
    /*invertDir=*/  true     // Both motors inverted — belt routing makes forward = wrong direction
);

static OpenLoopStepper motorB(
    Pins::Gantry::MOTOR_B_STEP,
    Pins::Gantry::MOTOR_B_DIR,
    Constants::Gantry::STEPS_PER_MM,
    /*stateIndex=*/ 1,
    /*invertDir=*/  true     // Flip independently if one axis is still wrong after upload
);

// ----------------------------------------------------------
//  Lead screw steppers — 4x NEMA 17 (open-loop)
//  Indexes 0–3 map to MachineState::leadScrew[0–3].
// ----------------------------------------------------------
static OpenLoopStepper zScrew0(
    Pins::LeadScrews::DEVICE1_SCREW1_STEP,
    Pins::LeadScrews::DEVICE1_SCREW1_DIR,
    Constants::LeadScrews::STEPS_PER_MM,
    /*stateIndex=*/ 0
);
static OpenLoopStepper zScrew1(
    Pins::LeadScrews::DEVICE1_SCREW2_STEP,
    Pins::LeadScrews::DEVICE1_SCREW2_DIR,
    Constants::LeadScrews::STEPS_PER_MM,
    /*stateIndex=*/ 1
);
static OpenLoopStepper zScrew2(
    Pins::LeadScrews::DEVICE2_SCREW1_STEP,
    Pins::LeadScrews::DEVICE2_SCREW1_DIR,
    Constants::LeadScrews::STEPS_PER_MM,
    /*stateIndex=*/ 2
);
static OpenLoopStepper zScrew3(
    Pins::LeadScrews::DEVICE2_SCREW2_STEP,
    Pins::LeadScrews::DEVICE2_SCREW2_DIR,
    Constants::LeadScrews::STEPS_PER_MM,
    /*stateIndex=*/ 3
);

// Convenience array for passing to Operations
static OpenLoopStepper* zScrews[4] = {&zScrew0, &zScrew1, &zScrew2, &zScrew3};

// ----------------------------------------------------------
//  Microswitches — X, Y, Z home/limit switches
//  normallyOpen=true means unpressed = HIGH (INPUT_PULLUP).
//  Flip to false if your switches are wired normally-closed.
// ----------------------------------------------------------
static Microswitch switchX(Pins::Switches::SWITCH_X, SwitchAxis::X, /*normallyOpen=*/ true);
static Microswitch switchY(Pins::Switches::SWITCH_Y, SwitchAxis::Y, /*normallyOpen=*/ true);
static Microswitch switchZ(Pins::Switches::SWITCH_Z, SwitchAxis::Z, /*normallyOpen=*/ true);

// ----------------------------------------------------------
//  Control stack
// ----------------------------------------------------------
static MotorController motorController;

// ----------------------------------------------------------
//  Sensor hardware
// ----------------------------------------------------------
static LaserSensor& laserSensor  = LaserSensor::instance();
static LineLaser&   lineLaser    = LineLaser::instance();

// ----------------------------------------------------------
//  Global motor controller pointer — used by Operation base
//  class helpers (waitForMotionComplete, etc.)
// ----------------------------------------------------------
MotorController* g_motorController = &motorController;

// ----------------------------------------------------------
//  Calibration data — persists for the lifetime of a session
// ----------------------------------------------------------
static CalibData calibData;

// ============================================================
//  Static callback functions registered with ThreadManager.
//  Must be plain functions (no captures) because ThreadManager
//  stores them as void(*)() with -fno-exceptions / no std::function.
// ============================================================
static void controlCallback() {
    switchX.update();
    switchY.update();
    switchZ.update();

    bool motionDone = motorController.update();
    if (motionDone &&
        MachineState::instance().getStatus() == SystemStatus::RUNNING) {
        MachineState::instance().setStatus(SystemStatus::IDLE);
    }

    laserSensor.syncState();
    lineLaser.syncSafety();
}

static void serialCallback() {
    SerialComm::instance().receive();
}

// ============================================================
//  setup()
// ============================================================
void setup() {
    // USB serial — wait up to 3s for host connection before proceeding
    Serial.begin(Constants::Serial::BAUD_RATE);
    uint32_t waitStart = millis();
    while (!Serial && (millis() - waitStart) < 3000) {}

    // ---- Initialise device drivers ----
    motorA.begin();   // StepperDriver::begin() — configures STEP/DIR pins
    motorB.begin();
    zScrew0.begin();
    zScrew1.begin();
    zScrew2.begin();
    zScrew3.begin();

    switchX.begin();
    switchY.begin();
    switchZ.begin();

    ServoController::instance().begin();
    DCMotorDriver::instance().begin();

    laserSensor.begin();
    lineLaser.begin();

    // ---- Initialise control stack ----
    motorController.begin(motorA, motorB);

    // ---- Initialise serial communication layer ----
    SerialComm::instance().setMotorController(&motorController);
    SerialComm::instance().setLeadScrews(zScrews);
    SerialComm::instance().setServoController(&ServoController::instance());
    SerialComm::instance().setMicroswitches(&switchX, &switchY, &switchZ);
    SerialComm::instance().begin();

    // ---- Register callbacks ----
    ThreadManager::instance().setControlCallback(controlCallback);
    ThreadManager::instance().setSerialCallback(serialCallback);

    // ---- Start all threads ----
    ThreadManager::instance().begin();

    // ---- Initial status log ----
    MachineState::instance().setStatus(SystemStatus::IDLE);
    StateSnapshot::logEntry();
}

// ============================================================
//  loop()
//  Intentionally minimal — all real work is threaded.
//  loop() runs on the Arduino main thread, which has the
//  lowest effective priority under TeensyThreads.
//
//  Use this only for non-critical background tasks that don't
//  belong in any specific thread, such as periodic diagnostics.
// ============================================================
void loop() {
    // Yield to TeensyThreads so worker/control threads can run.
    threads.yield();

    delay(100);
}