#include "devices/OpenLoopStepper.h"

// ============================================================
//  OpenLoopStepper.cpp
// ============================================================

OpenLoopStepper::OpenLoopStepper(int     stepPin,
                                 int     dirPin,
                                 float   stepsPerMm,
                                 uint8_t stateIndex,
                                 bool    invertDir)
    : StepperDriver(stepPin, dirPin, stepsPerMm, invertDir)
    , stateIndex_(stateIndex)
{}

// ----------------------------------------------------------
void OpenLoopStepper::syncState(bool isMoving) {
    StepperState s;
    s.positionMM  = getPositionMM();
    s.velocityMMS = 0.0f;   // TODO: Compute velocity from step rate if needed
    s.isMoving    = isMoving;
    s.isHomed     = (positionSteps_ != 0 || isMoving);
    MachineState::instance().setLeadScrew(stateIndex_, s);
}
