#include "math/Kinematics.h"

// ============================================================
//  Kinematics.cpp
// ============================================================

Vec2 Kinematics::motorToCartesian(float a_counts, float b_counts) {
    const float spm = Constants::Gantry::STEPS_PER_MM;
    Vec2 cart;
    cart(0, 0) = (a_counts + b_counts) / (2.0f * spm);   // x_mm
    cart(1, 0) = (a_counts - b_counts) / (2.0f * spm);   // y_mm
    return cart;
}

Vec2 Kinematics::cartesianToMotor(float x_mm, float y_mm) {
    const float spm = Constants::Gantry::STEPS_PER_MM;
    Vec2 motor;
    motor(0, 0) = (x_mm + y_mm) * spm;   // a_steps
    motor(1, 0) = (x_mm - y_mm) * spm;   // b_steps
    return motor;
}

Vec2 Kinematics::cartesianVelToMotorVel(float vx_mms, float vy_mms) {
    const float spm = Constants::Gantry::STEPS_PER_MM;
    Vec2 mv;
    mv(0, 0) = (vx_mms + vy_mms) * spm;   // va steps/s
    mv(1, 0) = (vx_mms - vy_mms) * spm;   // vb steps/s
    return mv;
}

Vec2 Kinematics::motorVelToCartesianVel(float va_steps_s, float vb_steps_s) {
    const float spm = Constants::Gantry::STEPS_PER_MM;
    Vec2 cv;
    cv(0, 0) = (va_steps_s + vb_steps_s) / (2.0f * spm);   // vx mm/s
    cv(1, 0) = (va_steps_s - vb_steps_s) / (2.0f * spm);   // vy mm/s
    return cv;
}

Vec2 Kinematics::clampToWorkspace(float x_mm, float y_mm) {
    Vec2 c;
    c(0, 0) = x_mm < 0.0f ? 0.0f :
              (x_mm > Constants::Gantry::X_TRAVEL_MM ? Constants::Gantry::X_TRAVEL_MM : x_mm);
    c(1, 0) = y_mm < 0.0f ? 0.0f :
              (y_mm > Constants::Gantry::Y_TRAVEL_MM ? Constants::Gantry::Y_TRAVEL_MM : y_mm);
    return c;
}

bool Kinematics::inWorkspace(float x_mm, float y_mm) {
    return (x_mm >= 0.0f && x_mm <= Constants::Gantry::X_TRAVEL_MM &&
            y_mm >= 0.0f && y_mm <= Constants::Gantry::Y_TRAVEL_MM);
}
