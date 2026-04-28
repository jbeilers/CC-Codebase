#include "core/ResourceMutex.h"

// ============================================================
//  ResourceMutex.cpp
// ============================================================

// ----------------------------------------------------------
//  Internal helpers (must be called with mutex_ held)
// ----------------------------------------------------------

bool ResourceMutex::tryAcquireLocked(Actuator a) {
    const uint8_t  bit  = static_cast<uint8_t>(a);
    const uint16_t mask = static_cast<uint16_t>(1u << bit);

    // Already held — idempotent (no double-charge)
    if (heldMask_ & mask) return true;

    // Budget guard — skip if MAX_TOTAL_AMPS not yet configured
    const float budget = Constants::Power::MAX_TOTAL_AMPS;
    if (budget > 0.0f) {
        const float cost = amperageFor(a);
        if (totalAmps_ + cost > budget) return false;
        totalAmps_ += cost;
    }

    heldMask_ |= mask;
    return true;
}

void ResourceMutex::releaseLocked(Actuator a) {
    const uint8_t  bit  = static_cast<uint8_t>(a);
    const uint16_t mask = static_cast<uint16_t>(1u << bit);

    if (!(heldMask_ & mask)) return;   // Not held — no-op

    heldMask_ &= ~mask;

    // Subtract this actuator's draw, guarded against underflow
    const float cost = amperageFor(a);
    totalAmps_ -= cost;
    if (totalAmps_ < 0.0f) totalAmps_ = 0.0f;
}

// ----------------------------------------------------------
//  Public interface
// ----------------------------------------------------------

bool ResourceMutex::tryAcquire(Actuator a) {
    mutex_.lock();
    bool ok = tryAcquireLocked(a);
    mutex_.unlock();
    return ok;
}

bool ResourceMutex::tryAcquireAll(std::initializer_list<Actuator> actuators) {
    mutex_.lock();

    // Phase 1: check whether the entire set fits in the budget.
    const float budget = Constants::Power::MAX_TOTAL_AMPS;
    if (budget > 0.0f) {
        float prospective = totalAmps_;
        for (Actuator a : actuators) {
            const uint8_t  bit  = static_cast<uint8_t>(a);
            const uint16_t mask = static_cast<uint16_t>(1u << bit);
            // Only count actuators not already held
            if (!(heldMask_ & mask)) {
                prospective += amperageFor(a);
            }
        }
        if (prospective > budget) {
            mutex_.unlock();
            return false;
        }
    }

    // Phase 2: all fit — acquire the whole set
    for (Actuator a : actuators) {
        tryAcquireLocked(a);   // Cannot fail now; budget pre-checked
    }

    mutex_.unlock();
    return true;
}

void ResourceMutex::release(Actuator a) {
    mutex_.lock();
    releaseLocked(a);
    mutex_.unlock();
}

void ResourceMutex::releaseAll(std::initializer_list<Actuator> actuators) {
    mutex_.lock();
    for (Actuator a : actuators) {
        releaseLocked(a);
    }
    mutex_.unlock();
}

void ResourceMutex::releaseAll() {
    mutex_.lock();
    heldMask_  = 0;
    totalAmps_ = 0.0f;
    mutex_.unlock();
}

float ResourceMutex::currentDrawAmps() const {
    mutex_.lock();
    float v = totalAmps_;
    mutex_.unlock();
    return v;
}

float ResourceMutex::remainingAmps() const {
    const float budget = Constants::Power::MAX_TOTAL_AMPS;
    if (budget <= 0.0f) return 9999.0f;   // Not configured — treat as unlimited
    mutex_.lock();
    float v = budget - totalAmps_;
    mutex_.unlock();
    return v;
}

bool ResourceMutex::isHeld(Actuator a) const {
    const uint16_t mask = static_cast<uint16_t>(1u << static_cast<uint8_t>(a));
    mutex_.lock();
    bool v = (heldMask_ & mask) != 0;
    mutex_.unlock();
    return v;
}

uint16_t ResourceMutex::heldMask() const {
    mutex_.lock();
    uint16_t v = heldMask_;
    mutex_.unlock();
    return v;
}

// ============================================================
//  ActuatorGuard
// ============================================================

ActuatorGuard::ActuatorGuard(std::initializer_list<Actuator> actuators) {
    auto& rm = ResourceMutex::instance();
    acquired_ = rm.tryAcquireAll(actuators);
    if (acquired_) {
        for (Actuator a : actuators) {
            heldMask_ |= static_cast<uint16_t>(1u << static_cast<uint8_t>(a));
        }
    }
}

ActuatorGuard::ActuatorGuard(Actuator a) {
    auto& rm = ResourceMutex::instance();
    acquired_ = rm.tryAcquire(a);
    if (acquired_) {
        heldMask_ = static_cast<uint16_t>(1u << static_cast<uint8_t>(a));
    }
}

ActuatorGuard::~ActuatorGuard() {
    if (!acquired_ || heldMask_ == 0) return;

    auto& rm = ResourceMutex::instance();
    // Release every bit set in heldMask_
    for (uint8_t bit = 0; bit < ACTUATOR_COUNT; ++bit) {
        if (heldMask_ & (1u << bit)) {
            rm.release(static_cast<Actuator>(bit));
        }
    }
}
