#pragma once

#include <cmath>
#include <cstdint>
#include "math/Matrix.h"
#include "math/Constants.h"

// ============================================================
//  TrajectoryPlanner.h  —  Jerk-limited (S-curve) 2D trajectory
//                          with ZVD input shaping.
//
//  OVERVIEW
//  --------
//  Moves the gantry from its current position to a target
//  position along a straight line in Cartesian space.
//  The scalar path distance is profiled with a 7-phase
//  S-curve that respects:
//    V_max   — peak cruise speed      (Constants::Trajectory::MAX_VEL_MMS)
//    A_max   — peak acceleration      (Constants::Trajectory::MAX_ACCEL_MMS2)
//    J_max   — peak jerk              (Constants::Trajectory::MAX_JERK_MMS3)
//
//  The scalar profile is then projected onto the X and Y axes
//  using the unit vector of the move.
//
//  ZVD INPUT SHAPING
//  -----------------
//  A Zero-Vibration-Derivative (ZVD) input shaper is applied
//  to the acceleration signal to suppress residual vibration.
//  The shaper convolves the acceleration command with a 3-impulse
//  sequence:
//    { (A1, 0),  (A2, T/2),  (A3, T) }
//  where T = 1/fn is the period of the dominant resonance and
//  the amplitudes A1/A2/A3 are derived from the damping ratio ζ.
//
//  Coefficients are computed at begin() from:
//    Constants::Trajectory::NATURAL_FREQ_HZ
//    Constants::Trajectory::DAMPING_RATIO
//
//  The shaper introduces a time delay of T seconds.
//  If ZVD shaping is not needed, call begin() and set
//  enableZVD(false).  The delay buffer is still allocated but
//  not applied.
//
//  OUTPUT (per tick via step())
//  ----------------------------
//    TrajectoryState::pos     [x_ref, y_ref]     mm
//    TrajectoryState::vel     [vx_ref, vy_ref]   mm/s
//    TrajectoryState::accel   [ax_ref, ay_ref]   mm/s²
//    TrajectoryState::done    true when move is complete
//
//  USAGE
//  -----
//    planner.begin();
//    planner.moveTo(target_x, target_y, current_x, current_y);
//    // Each control tick:
//    TrajectoryState ts = planner.step(dt);
//    if (ts.done) { ... }
// ============================================================

// Maximum ZVD delay buffer length in control ticks.
// At 1 kHz and fn ≥ 2 Hz, T ≤ 0.5 s → 500 ticks.
// Increase if natural frequency is very low.
constexpr uint16_t ZVD_BUFFER_LEN = 512;

struct TrajectoryState {
    Vec2  pos;          // Reference position  [x, y]  (mm)
    Vec2  vel;          // Reference velocity  [vx, vy] (mm/s)
    Vec2  accel;        // Reference accel     [ax, ay] (mm/s²)
    bool  done = true;  // True when move is fully settled
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner() = default;

    // ----------------------------------------------------------
    //  Call once in setup().
    //  Computes ZVD coefficients from Constants.
    //  If natural freq or damping are -1 (not yet set), ZVD
    //  is automatically disabled.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Queue a new move to (target_x, target_y) starting from
    //  (current_x, current_y).
    //  Calling this mid-move overrides the active move.
    //  Units: mm (Cartesian).
    // ----------------------------------------------------------
    void moveTo(float targetX, float targetY,
                float currentX, float currentY);

    // ----------------------------------------------------------
    //  Advance the trajectory by dt seconds.
    //  Returns the current reference TrajectoryState.
    //  Call each control tick from MotorController::update().
    // ----------------------------------------------------------
    TrajectoryState step(float dt);

    // ----------------------------------------------------------
    //  True if the planner is idle (no active move, or done).
    // ----------------------------------------------------------
    bool isDone() const { return done_; }

    // ----------------------------------------------------------
    //  Enable or disable ZVD shaping at runtime.
    //  Disabling clears the delay buffer.
    // ----------------------------------------------------------
    void enableZVD(bool en);
    bool isZVDEnabled() const { return zvdEnabled_; }

    // ----------------------------------------------------------
    //  Immediately abort the current move and hold position.
    // ----------------------------------------------------------
    void stop();

private:
    // ----------------------------------------------------------
    //  7-phase S-curve planning
    // ----------------------------------------------------------

    // Compute phase durations for a distance `dist` (scalar mm)
    void planSCurve(float dist);

    // Evaluate scalar s-curve at accumulated path time t_
    // Fills scalar_pos_, scalar_vel_, scalar_accel_
    void evalSCurve(float t);

    // Phase durations (seconds)
    float tj1_  = 0.0f;    // Phase 1: jerk up   (0 → a_max)
    float ta_   = 0.0f;    // Phase 2: const accel
    float tj2_  = 0.0f;    // Phase 3: jerk down (a_max → 0)
    float tv_   = 0.0f;    // Phase 4: cruise
    float tj3_  = 0.0f;    // Phase 5: jerk down (0 → -a_max)
    float td_   = 0.0f;    // Phase 6: const decel
    float tj4_  = 0.0f;    // Phase 7: jerk up   (-a_max → 0)
    float totalTime_ = 0.0f;

    // Intermediate boundary velocities
    float v_lim_ = 0.0f;   // Actual peak velocity (may be < V_max)
    float a_lim_ = 0.0f;   // Actual peak accel    (may be < A_max)

    // Current scalar path state (distance along path)
    float t_        = 0.0f;    // Accumulated time in current move
    float scalar_pos_   = 0.0f;
    float scalar_vel_   = 0.0f;
    float scalar_accel_ = 0.0f;

    // Path geometry (unit vector of the move)
    float ux_    = 0.0f;    // Unit vector X component
    float uy_    = 0.0f;    // Unit vector Y component
    float dist_  = 0.0f;    // Total path distance (mm)

    // Start position (used to compute absolute reference)
    Vec2  startPos_;

    bool  done_ = true;

    // ----------------------------------------------------------
    //  ZVD input shaper
    // ----------------------------------------------------------
    void zvdInit();
    float zvdPush(float accel_sample);   // Returns shaped accel

    bool  zvdEnabled_ = false;

    // 3-impulse ZVD parameters
    float zvdA1_ = 0.0f, zvdA2_ = 0.0f, zvdA3_ = 0.0f;
    int   zvdDelay1_ = 0;  // Delay for A2 in ticks
    int   zvdDelay2_ = 0;  // Delay for A3 in ticks

    // Circular delay buffer for raw acceleration signal
    float  zvdBuf_[ZVD_BUFFER_LEN] = {0.0f};
    int    zvdHead_ = 0;    // Write pointer
};
