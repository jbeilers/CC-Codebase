#pragma once

#include "math/Matrix.h"
#include "math/Constants.h"

// ============================================================
//  KalmanFilter.h  —  Discrete linear Kalman filter for the
//                     CoreXY gantry.
//
//  State vector  x̂ = [ x_pos,  x_vel,  y_pos,  y_vel ]ᵀ
//                     (all in mm and mm/s, Cartesian frame)
//
//  Input vector  u  = [ a_x,  a_y ]ᵀ  (commanded accel mm/s²)
//
//  Measurement   z  = [ x_enc,  y_enc ]ᵀ  (mm, Cartesian)
//                     Converted from motor-space encoder counts
//                     by the caller via Kinematics::motorToCartesian.
//
//  Process model (ZOH constant-acceleration discretisation):
//    F = [ 1  dt   0   0  ]      B = [ dt²/2    0    ]
//        [ 0   1   0   0  ]          [ dt       0    ]
//        [ 0   0   1  dt  ]          [  0     dt²/2  ]
//        [ 0   0   0   1  ]          [  0       dt   ]
//
//  Observation:  H = [ 1  0  0  0 ]
//                    [ 0  0  1  0 ]
//
//  Noise covariances built from Constants::Kalman.
//
//  Usage (from MotorController::update each tick):
//    kf.predict(u_accel, dt);
//    kf.update(z_encoder_mm);   // if encoder reading is fresh
//    Vec4 state = kf.getState();
// ============================================================

// Extra matrix type aliases used by the Kalman equations
using Mat4x2 = Matrix<4, 2>;   // B matrix and gain K_gain
using Mat2x4 = Matrix<2, 4>;   // H matrix, H*P

class KalmanFilter {
public:
    KalmanFilter() = default;

    // ----------------------------------------------------------
    //  Call once in setup() before any predict/update calls.
    //  Builds F, B, H, Q, R from Constants; sets P = P0.
    // ----------------------------------------------------------
    void begin(float dt = Constants::Timing::CONTROL_LOOP_DT);

    // ----------------------------------------------------------
    //  Prediction step — call every control tick.
    //  u = [a_x, a_y] commanded acceleration (mm/s²).
    //  dt — actual elapsed seconds since last call.
    // ----------------------------------------------------------
    void predict(const Vec2& u, float dt);

    // ----------------------------------------------------------
    //  Correction step — call when a fresh encoder reading
    //  is available.
    //  z = [x_enc_mm, y_enc_mm] in Cartesian mm.
    // ----------------------------------------------------------
    void update(const Vec2& z);

    // ----------------------------------------------------------
    //  State accessors (all in mm / mm·s⁻¹)
    // ----------------------------------------------------------
    const Vec4& getState()  const { return x_; }
    float getX()  const { return x_(0, 0); }
    float getVx() const { return x_(1, 0); }
    float getY()  const { return x_(2, 0); }
    float getVy() const { return x_(3, 0); }

    // ----------------------------------------------------------
    //  Reset the filter (e.g. after homing completes).
    //  Sets position; zeroes velocity; resets P to P0.
    // ----------------------------------------------------------
    void reset(float x_mm, float y_mm);

    // ----------------------------------------------------------
    //  Diagnostics — covariance trace (total uncertainty)
    // ----------------------------------------------------------
    float covarianceTrace() const {
        return P_(0,0) + P_(1,1) + P_(2,2) + P_(3,3);
    }

private:
    // Build (or rebuild) F_ and B_ for the given dt.
    void buildProcessModel(float dt);

    Vec4   x_;          // State estimate [ x, vx, y, vy ]
    Mat4x4 P_;          // Estimate error covariance
    Mat4x4 F_;          // State transition (rebuilt each predict if dt varies)
    Mat4x2 B_;          // Control input matrix
    Mat2x4 H_;          // Observation matrix (constant)
    Mat4x4 Q_;          // Process noise covariance (constant)
    Mat2x2 R_;          // Measurement noise covariance (constant)
    Mat4x4 P0_;         // Initial covariance (kept for reset())

    float  dt_  = Constants::Timing::CONTROL_LOOP_DT;
    bool   ready_ = false;
};
