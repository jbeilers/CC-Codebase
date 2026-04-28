#pragma once

#include <cstdint>

// ============================================================
//  Operation.h  —  Abstract base class for all multi-step
//                  machine operations.
//
//  An Operation is a Task whose work function consists of a
//  sequence of sub-steps (moves, waits, sensor reads, etc.)
//  executed in order.
//
//  Operations are constructed, pushed onto the TaskQueue via
//  a Task wrapper, and executed by a worker thread.
//
//  Concrete operations override run() and implement their
//  step sequence as a blocking procedure (since they run on
//  a dedicated worker thread).
//
//  All operations should:
//    - Check MachineState::getStatus() before every step.
//    - Abort cleanly (set status FAULT if needed) on failure.
//    - Report completion or failure via the status enum.
// ============================================================

enum class OperationStatus : uint8_t {
    NOT_STARTED,
    RUNNING,
    COMPLETE,
    FAILED,
    ABORTED
};

class Operation {
public:
    virtual ~Operation() = default;

    // ----------------------------------------------------------
    //  Execute the operation. Called by the worker thread.
    //  Implementations must be blocking and self-contained.
    //  Returns when operation is complete, failed, or aborted.
    // ----------------------------------------------------------
    virtual void run() = 0;

    // ----------------------------------------------------------
    //  Request graceful abort. run() should check this flag
    //  between steps and exit early if true.
    // ----------------------------------------------------------
    virtual void abort() { abortRequested_ = true; }

    // ----------------------------------------------------------
    //  Returns true once the operation has finished (any outcome).
    // ----------------------------------------------------------
    bool isDone() const {
        return status_ == OperationStatus::COMPLETE
            || status_ == OperationStatus::FAILED
            || status_ == OperationStatus::ABORTED;
    }

    OperationStatus getStatus() const { return status_; }

    const char* getName() const { return name_; }

protected:
    explicit Operation(const char* name)
        : status_(OperationStatus::NOT_STARTED)
        , name_(name)
    {}

    // ----------------------------------------------------------
    //  Helpers for use inside run()
    // ----------------------------------------------------------

    // Block until the MotorController reports idle,
    // checking abort flag and system status each tick.
    // @param timeoutMs  Max wait time (0 = infinite)
    // Returns false on timeout or abort.
    bool waitForMotionComplete(uint32_t timeoutMs = 0);

    // Block for a fixed duration, respecting abort flag.
    void waitMs(uint32_t ms);

    // Check whether abort has been requested or system is in fault.
    // Returns true if the operation should stop immediately.
    bool shouldAbort() const;

    bool            abortRequested_ = false;
    OperationStatus status_         = OperationStatus::NOT_STARTED;
    const char*     name_;
};
