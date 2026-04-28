#include "control/LQRController.h"
#include <cmath>

// ============================================================
//  LQRController.cpp
// ============================================================

LQRController::LQRController(float integralClamp)
    : Kp_(Constants::LQR::K[0][0])
    , Kd_(Constants::LQR::K[0][1])
    , Ki_(Constants::LQR::K[0][2])
    , integralClamp_(integralClamp)
{}

// ----------------------------------------------------------
void LQRController::setReference(float refPos, float refVel) {
    refPos_ = refPos;
    refVel_ = refVel;
}

// ----------------------------------------------------------
float LQRController::update(float estPos, float estVel, float dt) {
    // Position and velocity errors
    const float e  = refPos_ - estPos;
    const float de = refVel_ - estVel;

    // Integrate position error with anti-windup clamp
    integral_ += e * dt;
    if (integral_ >  integralClamp_) integral_ =  integralClamp_;
    if (integral_ < -integralClamp_) integral_ = -integralClamp_;

    lastError_ = e;

    // LQR control law:  u = Kp·e + Kd·ė + Ki·∫e
    // (Signs are positive here because K stores magnitudes;
    //  the controller stabilises by commanding accel toward reference.)
    return Kp_ * e + Kd_ * de + Ki_ * integral_;
}

// ----------------------------------------------------------
void LQRController::reset() {
    integral_  = 0.0f;
    lastError_ = 0.0f;
    refPos_    = 0.0f;
    refVel_    = 0.0f;
}
