#include "operations/Operation.h"
#include "state/MachineState.h"
#include "control/MotorController.h"
#include <TeensyThreads.h>

// ============================================================
//  Operation.cpp
// ============================================================

// External reference — MotorController is instantiated in main.cpp
// and accessed here by pointer. Set via Operation::setMotorController().
// Declared extern so operations can call isIdle() without coupling
// to main.cpp's include chain.
extern MotorController* g_motorController;

// ----------------------------------------------------------
bool Operation::waitForMotionComplete(uint32_t timeoutMs) {
    uint32_t elapsed = 0;
    const uint32_t POLL_MS = 5;

    while (true) {
        if (shouldAbort())                        return false;
        if (!g_motorController->isMoving())        return true;

        threads.delay(POLL_MS);
        elapsed += POLL_MS;

        if (timeoutMs > 0 && elapsed >= timeoutMs) return false;
    }
}

// ----------------------------------------------------------
void Operation::waitMs(uint32_t ms) {
    const uint32_t SLICE_MS = 5;
    uint32_t remaining = ms;
    while (remaining > 0 && !shouldAbort()) {
        uint32_t slice = (remaining < SLICE_MS) ? remaining : SLICE_MS;
        threads.delay(slice);
        remaining -= slice;
    }
}

// ----------------------------------------------------------
bool Operation::shouldAbort() const {
    if (abortRequested_) return true;
    SystemStatus s = MachineState::instance().getStatus();
    return (s == SystemStatus::FAULT || s == SystemStatus::E_STOP);
}
