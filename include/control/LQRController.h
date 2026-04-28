#pragma once

#include "math/Constants.h"

// ============================================================
//  LQRController.h  —  Single-axis discrete LQR with integral
//                      anti-windup.
//
//  Augmented state:  [ error,  error_dot,  ∫error ]
//
//  Control law:
//    u = −K[0]·e − K[1]·ė − K[2]·∫e
//
//  where:
//    e      = ref_pos − est_pos          (position error, mm)
//    ė      = ref_vel − est_vel          (velocity error, mm/s)
//    ∫e     = running integral of e·dt   (mm·s)
//
//  The gain vector K[3] is loaded from Constants::LQR::K[0][0..2].
//  Fill in the gains offline:
//    Python: K, S, E = control.dlqr(F_aug, B_aug, Q_aug, R_aug)
//    MATLAB: [K, S, E] = dlqr(F_aug, B_aug, Q_aug, R_aug)
//
//  One instance is created per Cartesian axis (X and Y).
//  MotorController holds two instances.
//
//  Anti-windup:
//    The integrator is clamped to ±INTEGRAL_CLAMP.
//    TODO: Set INTEGRAL_CLAMP in Constants or as a constructor arg
//          once the physical range of motion is known.
//
//  Usage:
//    lqrX_.setReference(ref_pos, ref_vel);
//    float u_x = lqrX_.update(est_pos, est_vel, dt);
// ============================================================

class LQRController {
public:
    // ----------------------------------------------------------
    //  integralClamp — maximum magnitude of the integral term
    //  in mm·s. Prevents windup during large errors or saturation.
    //  TODO: Tune this based on physical travel limits.
    // ----------------------------------------------------------
    explicit LQRController(float integralClamp = Constants::LQR::INTEGRAL_CLAMP);

    // ----------------------------------------------------------
    //  Set the desired reference position and velocity for this
    //  axis. Call once per trajectory step before update().
    // ----------------------------------------------------------
    void setReference(float refPos, float refVel);

    // ----------------------------------------------------------
    //  Compute the control output (mm/s² acceleration command)
    //  for one axis.
    //
    //  estPos — Kalman-filtered position estimate (mm)
    //  estVel — Kalman-filtered velocity estimate (mm/s)
    //  dt     — elapsed time since last call (seconds)
    //
    //  Returns signed acceleration command (mm/s²).
    //  Caller adds feed-forward term before converting to steps.
    // ----------------------------------------------------------
    float update(float estPos, float estVel, float dt);

    // ----------------------------------------------------------
    //  Reset integrator and last error (call after homing or
    //  after a large position discontinuity).
    // ----------------------------------------------------------
    void reset();

    // ----------------------------------------------------------
    //  Diagnostics — expose internal state for telemetry
    // ----------------------------------------------------------
    float getError()    const { return lastError_; }
    float getIntegral() const { return integral_; }

private:
    float Kp_;             // Gain on position error
    float Kd_;             // Gain on velocity error
    float Ki_;             // Gain on integral of position error
    float integralClamp_;  // Anti-windup bound

    float refPos_   = 0.0f;
    float refVel_   = 0.0f;
    float integral_ = 0.0f;
    float lastError_= 0.0f;
};