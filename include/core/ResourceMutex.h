#pragma once

#include <cstdint>
#include <initializer_list>
#include <TeensyThreads.h>
#include "math/Constants.h"

// ============================================================
//  ResourceMutex.h  —  Amperage-aware actuator gating.
//
//  PURPOSE
//  -------
//  Prevents combinations of motors/servos from running
//  simultaneously when their combined current draw would
//  exceed the PSU budget (Constants::Power::MAX_TOTAL_AMPS).
//
//  Each actuator has a rated peak current draw defined in
//  Constants::Power.  Before any device driver activates a
//  motor or servo, it must acquire a token via tryAcquire().
//  When the actuator is done, it releases the token.
//
//  ACTUATORS
//  ---------
//  Each member of the Actuator enum maps to exactly one
//  physical device.  The two CoreXY steppers are listed
//  separately because they may (rarely) be acquired and
//  released independently (e.g. for single-axis homing).
//  In normal operation MotorController acquires both together
//  via tryAcquireAll({GANTRY_A, GANTRY_B}).
//
//  USAGE — single actuator
//  ------------------------
//    if (ResourceMutex::instance().tryAcquire(Actuator::STS3215)) {
//        servoController.moveSTS3215(angle);
//        ResourceMutex::instance().release(Actuator::STS3215);
//    } else {
//        // Budget exceeded — defer or abort
//    }
//
//  USAGE — RAII guard (preferred, exception-safe)
//  -----------------------------------------------
//    {
//        ActuatorGuard guard({Actuator::GANTRY_A, Actuator::GANTRY_B});
//        if (!guard.acquired()) { return; }   // Budget exceeded
//        motorController.moveTo(x, y);
//    }   // <-- guard destructor releases both tokens automatically
//
//  USAGE — atomic multi-acquire
//  -----------------------------
//    All-or-nothing: if any one actuator in the set would push
//    the draw over budget, the entire group is rejected and
//    nothing is acquired.
//    if (ResourceMutex::instance().tryAcquireAll(
//            {Actuator::MG996R_0, Actuator::MG996R_1})) { ... }
//
//  THREAD SAFETY
//  -------------
//  All public methods are protected by an internal Threads::Mutex.
//  Safe to call from the serial thread, worker pool, or control
//  thread simultaneously.
//
//  NOTES
//  -----
//  - If Constants::Power::MAX_TOTAL_AMPS is still -1 (not yet
//    set), the budget check is SKIPPED and tryAcquire always
//    returns true.  Fill in the constant before final deployment.
//  - Current draw values for each actuator are looked up from
//    Constants::Power at compile time via amperageFor().
//    No heap allocation occurs.
// ============================================================

// ----------------------------------------------------------
//  Actuator identifiers — one per physical device.
// ----------------------------------------------------------
enum class Actuator : uint8_t {
    GANTRY_A    = 0,   // CoreXY closed-loop stepper, Motor A
    GANTRY_B    = 1,   // CoreXY closed-loop stepper, Motor B
    LEADSCREW_0 = 2,   // NEMA 17 lead screw stepper, index 0
    LEADSCREW_1 = 3,   // NEMA 17 lead screw stepper, index 1
    LEADSCREW_2 = 4,   // NEMA 17 lead screw stepper, index 2
    LEADSCREW_3 = 5,   // NEMA 17 lead screw stepper, index 3
    STS3215     = 6,   // Feetech STS3215 smart servo
    MG996R_0    = 7,   // MG996R standard servo, index 0
    MG996R_1    = 8,   // MG996R standard servo, index 1
    MG996R_2    = 9,   // MG996R standard servo, index 2
    MG996R_3    = 10,  // MG996R standard servo, index 3
    DC_MOTOR_0  = 11,  // DC motor via L298N, index 0
    DC_MOTOR_1  = 12,  // DC motor via L298N, index 1
    COUNT       = 13   // Sentinel — do not use as an actuator
};

constexpr uint8_t ACTUATOR_COUNT = static_cast<uint8_t>(Actuator::COUNT);

// ----------------------------------------------------------
//  Forward declaration for the RAII guard
// ----------------------------------------------------------
class ResourceMutex;

// ============================================================
//  ActuatorGuard  —  RAII wrapper for safe multi-acquire.
//
//  Acquires all listed actuators atomically in the constructor.
//  Releases all on destruction, even if an exception unwinds
//  the stack (not that Arduino code uses exceptions, but good
//  practice).
//
//  Usage:
//    ActuatorGuard guard({Actuator::GANTRY_A, Actuator::GANTRY_B});
//    if (!guard.acquired()) return;
//    // ... use actuators ...
//    // released automatically when guard goes out of scope
// ============================================================
class ActuatorGuard {
public:
    // ----------------------------------------------------------
    //  Attempt to acquire all actuators in the initialiser list.
    //  If budget would be exceeded, nothing is acquired.
    // ----------------------------------------------------------
    ActuatorGuard(std::initializer_list<Actuator> actuators);

    // ----------------------------------------------------------
    //  Single-actuator convenience constructor.
    // ----------------------------------------------------------
    explicit ActuatorGuard(Actuator a);

    // ----------------------------------------------------------
    //  Releases all held tokens on destruction.
    // ----------------------------------------------------------
    ~ActuatorGuard();

    // Non-copyable; moveable would be complex — just forbid both
    ActuatorGuard(const ActuatorGuard&)            = delete;
    ActuatorGuard& operator=(const ActuatorGuard&) = delete;

    // ----------------------------------------------------------
    //  True if all requested actuators were successfully acquired.
    //  Check this immediately after construction.
    // ----------------------------------------------------------
    bool acquired() const { return acquired_; }

private:
    // Compact bitmask of which actuators this guard holds.
    // Bit n = 1 if Actuator(n) is held.
    uint16_t heldMask_  = 0;
    bool     acquired_  = false;
};

// ============================================================
//  ResourceMutex  —  Singleton amperage budget tracker.
// ============================================================
class ResourceMutex {
public:
    // Singleton accessor
    static ResourceMutex& instance() {
        static ResourceMutex inst;
        return inst;
    }

    ResourceMutex(const ResourceMutex&)            = delete;
    ResourceMutex& operator=(const ResourceMutex&) = delete;

    // ----------------------------------------------------------
    //  Attempt to acquire a single actuator token.
    //
    //  Returns true  — token granted; caller may activate the
    //                   actuator.
    //  Returns false — budget would be exceeded; actuator must
    //                   not be activated.
    //
    //  If the actuator is already held (double-acquire), this
    //  returns true without re-charging the budget.  Callers
    //  should track their own hold state to avoid this.
    // ----------------------------------------------------------
    bool tryAcquire(Actuator a);

    // ----------------------------------------------------------
    //  Attempt to acquire all actuators in a set atomically.
    //  If ANY one would exceed the budget, NONE are acquired.
    //  Returns true only if the full set was granted.
    // ----------------------------------------------------------
    bool tryAcquireAll(std::initializer_list<Actuator> actuators);

    // ----------------------------------------------------------
    //  Release a previously acquired actuator token.
    //  Safe to call even if the actuator is not currently held
    //  (no-op in that case).
    // ----------------------------------------------------------
    void release(Actuator a);

    // ----------------------------------------------------------
    //  Release a set of actuators at once (used by ActuatorGuard).
    // ----------------------------------------------------------
    void releaseAll(std::initializer_list<Actuator> actuators);

    // ----------------------------------------------------------
    //  Current total estimated draw from all held actuators (A).
    // ----------------------------------------------------------
    float currentDrawAmps() const;

    // ----------------------------------------------------------
    //  Remaining headroom before the budget is exhausted (A).
    //  Returns a large value if MAX_TOTAL_AMPS is not yet set.
    // ----------------------------------------------------------
    float remainingAmps() const;

    // ----------------------------------------------------------
    //  True if the given actuator token is currently held.
    // ----------------------------------------------------------
    bool isHeld(Actuator a) const;

    // ----------------------------------------------------------
    //  Diagnostic: bitmask of all currently held actuators.
    //  Bit n = 1 means Actuator(n) is active.
    // ----------------------------------------------------------
    uint16_t heldMask() const;

    // ----------------------------------------------------------
    //  Emergency release — drops all tokens immediately.
    //  Call from eStop() to ensure no actuator stays blocked
    //  in an acquired state after a fault.
    // ----------------------------------------------------------
    void releaseAll();

private:
    ResourceMutex() = default;

    // ----------------------------------------------------------
    //  Rated peak current draw for each actuator (amps).
    //  Indexed by static_cast<uint8_t>(Actuator).
    //  Populated from Constants::Power at compile time.
    // ----------------------------------------------------------
    static constexpr float amperageFor(Actuator a) {
        switch (a) {
            case Actuator::GANTRY_A:    return Constants::Power::GANTRY_MOTOR_AMPS;
            case Actuator::GANTRY_B:    return Constants::Power::GANTRY_MOTOR_AMPS;
            case Actuator::LEADSCREW_0: return Constants::Power::LEADSCREW_MOTOR_AMPS;
            case Actuator::LEADSCREW_1: return Constants::Power::LEADSCREW_MOTOR_AMPS;
            case Actuator::LEADSCREW_2: return Constants::Power::LEADSCREW_MOTOR_AMPS;
            case Actuator::LEADSCREW_3: return Constants::Power::LEADSCREW_MOTOR_AMPS;
            case Actuator::STS3215:     return Constants::Power::STS3215_AMPS;
            case Actuator::MG996R_0:    return Constants::Power::MG996R_AMPS;
            case Actuator::MG996R_1:    return Constants::Power::MG996R_AMPS;
            case Actuator::MG996R_2:    return Constants::Power::MG996R_AMPS;
            case Actuator::MG996R_3:    return Constants::Power::MG996R_AMPS;
            case Actuator::DC_MOTOR_0:  return Constants::Power::DC_MOTOR_AMPS;
            case Actuator::DC_MOTOR_1:  return Constants::Power::DC_MOTOR_AMPS;
            default:                    return 0.0f;
        }
    }

    // ----------------------------------------------------------
    //  Internal acquire/release helpers — callers must hold
    //  mutex_ before calling.
    // ----------------------------------------------------------
    bool tryAcquireLocked(Actuator a);
    void releaseLocked(Actuator a);

    // ----------------------------------------------------------
    //  State
    // ----------------------------------------------------------
    mutable Threads::Mutex mutex_;

    // Bitmask of currently held actuators (bit n = Actuator n)
    uint16_t heldMask_    = 0;

    // Running total of current draw for all held actuators
    float    totalAmps_   = 0.0f;
};
