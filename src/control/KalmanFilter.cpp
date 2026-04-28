#include "control/KalmanFilter.h"

// ============================================================
//  KalmanFilter.cpp
// ============================================================

void KalmanFilter::begin(float dt) {
    dt_ = dt;

    // ---- Observation matrix H (constant) ----
    // Maps [ x, vx, y, vy ] → [ x_enc, y_enc ]
    H_.zero();
    H_(0, 0) = 1.0f;   // x_pos
    H_(1, 2) = 1.0f;   // y_pos

    // ---- Process noise covariance Q (diagonal) ----
    // Tune via Constants::Kalman::Q_POS and Q_VEL.
    // Larger Q  → filter trusts the encoder more (faster but noisier).
    // Smaller Q → filter trusts the model more (smoother but slower to correct).
    Q_.zero();
    Q_(0, 0) = Constants::Kalman::Q_POS;
    Q_(1, 1) = Constants::Kalman::Q_VEL;
    Q_(2, 2) = Constants::Kalman::Q_POS;
    Q_(3, 3) = Constants::Kalman::Q_VEL;

    // ---- Measurement noise covariance R (diagonal) ----
    // Tune via Constants::Kalman::R_ENC.
    // Measure encoder jitter variance with gantry stationary and belt tensioned.
    R_.zero();
    R_(0, 0) = Constants::Kalman::R_ENC;
    R_(1, 1) = Constants::Kalman::R_ENC;

    // ---- Initial covariance P0 ----
    // Large initial uncertainty — the filter will converge quickly.
    P0_.zero();
    P0_(0, 0) = Constants::Kalman::P0_POS;
    P0_(1, 1) = Constants::Kalman::P0_VEL;
    P0_(2, 2) = Constants::Kalman::P0_POS;
    P0_(3, 3) = Constants::Kalman::P0_VEL;

    // ---- Build F and B for nominal dt ----
    buildProcessModel(dt_);

    // ---- Initial state and covariance ----
    x_.zero();
    P_ = P0_;

    ready_ = true;
}

// ----------------------------------------------------------
void KalmanFilter::buildProcessModel(float dt) {
    // State transition  F
    F_.setIdentity();
    F_(0, 1) = dt;   // x  += vx * dt
    F_(2, 3) = dt;   // y  += vy * dt

    // Control input  B
    B_.zero();
    B_(0, 0) = 0.5f * dt * dt;   // x  += ½ ax dt²
    B_(1, 0) = dt;                // vx += ax dt
    B_(2, 1) = 0.5f * dt * dt;   // y  += ½ ay dt²
    B_(3, 1) = dt;                // vy += ay dt
}

// ----------------------------------------------------------
void KalmanFilter::predict(const Vec2& u, float dt) {
    if (!ready_) return;

    // Rebuild process model if dt has changed noticeably
    // (handles jitter in the control loop timer)
    if (fabsf(dt - dt_) > 1e-6f) {
        dt_ = dt;
        buildProcessModel(dt_);
    }

    // State prediction:  x̂⁻ = F·x̂ + B·u
    x_ = F_ * x_ + B_ * u;

    // Covariance prediction:  P⁻ = F·P·Fᵀ + Q
    P_ = F_ * P_ * F_.transposed() + Q_;
}

// ----------------------------------------------------------
void KalmanFilter::update(const Vec2& z) {
    if (!ready_) return;

    // Innovation:  y = z − H·x̂⁻
    Vec2 y = z - H_ * x_;

    // Innovation covariance:  S = H·P⁻·Hᵀ + R
    Mat2x2 S = H_ * P_ * H_.transposed() + R_;

    // Kalman gain:  K = P⁻·Hᵀ·S⁻¹
    Mat2x2 S_inv;
    if (!S.inverse2x2(S_inv)) {
        // S is singular — skip update, keep prediction.
        // This should not happen with a valid R matrix.
        return;
    }
    // K_gain : 4×2 = P(4×4) * Hᵀ(4×2) * S_inv(2×2)
    Mat4x2 K_gain = P_ * H_.transposed() * S_inv;

    // State update:   x̂ = x̂⁻ + K·y
    x_ = x_ + K_gain * y;

    // Covariance update (Joseph form — numerically stable):
    //   P = (I − K·H)·P⁻·(I − K·H)ᵀ + K·R·Kᵀ
    Mat4x4 I_KH;
    I_KH.setIdentity();
    I_KH = I_KH - K_gain * H_;
    P_ = I_KH * P_ * I_KH.transposed() + K_gain * R_ * K_gain.transposed();
}

// ----------------------------------------------------------
void KalmanFilter::reset(float x_mm, float y_mm) {
    x_.zero();
    x_(0, 0) = x_mm;
    x_(2, 0) = y_mm;
    P_ = P0_;
}
