#pragma once

#include "devices/StepperDriver.h"
#include "math/Constants.h"

// ============================================================
//  OpenLoopStepper.h  —  NEMA 17 open-loop stepper driver.
//
//  Thin subclass of StepperDriver — adds no extra hardware
//  but carries its own MachineState index so it can write
//  its position back to the correct LeadScrew slot.
//
//  Usage:
//    OpenLoopStepper screw(
//        Pins::LeadScrews::DEVICE1_SCREW1_STEP,
//        Pins::LeadScrews::DEVICE1_SCREW1_DIR,
//        Constants::LeadScrews::STEPS_PER_MM,
//        0   // MachineState leadScrew index
//    );
//    screw.begin();
//    screw.setDirection(true);
//    screw.step();           // Call at desired rate from control loop
//    screw.syncState();      // Push position to MachineState
// ============================================================

class OpenLoopStepper : public StepperDriver {
public:
    // ----------------------------------------------------------
    //  Constructor
    //  @param stepPin        GPIO pin → TB6600 PUL+
    //  @param dirPin         GPIO pin → TB6600 DIR+
    //  @param stepsPerMm     From Constants::LeadScrews::STEPS_PER_MM
    //  @param stateIndex     Index into MachineState::leadScrew[] (0–3)
    //  @param invertDir      Set true if motor is wired in reverse
    // ----------------------------------------------------------
    OpenLoopStepper(int     stepPin,
                    int     dirPin,
                    float   stepsPerMm,
                    uint8_t stateIndex,
                    bool    invertDir = false);

    // ----------------------------------------------------------
    //  Write current position and motion state back to
    //  MachineState::leadScrew[stateIndex].
    //  Call once per control loop tick after step().
    // ----------------------------------------------------------
    void syncState(bool isMoving = false);

private:
    uint8_t stateIndex_;
};
