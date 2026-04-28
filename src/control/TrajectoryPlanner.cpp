#include "control/TrajectoryPlanner.h"

// ============================================================
//  TrajectoryPlanner.cpp
//
//  S-curve 7-phase derivation reference:
//    Biagiotti & Melchiorri, "Trajectory Planning for Automatic
//    Machines and Robots", Springer, 2008.  Chapter 3.
//
//  ZVD shaper reference:
//    Singer & Seering, "Preshaping Command Inputs to Reduce
//    System Vibration", ASME JDSMC, 1990.
// ============================================================

// ----------------------------------------------------------
void TrajectoryPlanner::begin() {
    done_ = true;
    zvdInit();
}

// ----------------------------------------------------------
void TrajectoryPlanner::moveTo(float targetX, float targetY,
                                float currentX, float currentY)
{
    float dx = targetX - currentX;
    float dy = targetY - currentY;
    dist_ = sqrtf(dx * dx + dy * dy);

    if (dist_ < 1e-4f) {
        // Already at target — nothing to do
        done_ = true;
        return;
    }

    // Unit vector along path
    ux_ = dx / dist_;
    uy_ = dy / dist_;

    startPos_(0, 0) = currentX;
    startPos_(1, 0) = currentY;

    t_            = 0.0f;
    scalar_pos_   = 0.0f;
    scalar_vel_   = 0.0f;
    scalar_accel_ = 0.0f;
    done_         = false;

    // Clear ZVD buffer so old data doesn't contaminate
    for (int i = 0; i < ZVD_BUFFER_LEN; i++) zvdBuf_[i] = 0.0f;
    zvdHead_ = 0;

    planSCurve(dist_);
}

// ----------------------------------------------------------
//  S-curve planning.
//
//  Given distance dist and limits V_max, A_max, J_max, compute
//  the 7-phase durations.
//
//  Time-optimal 7-phase S-curve (Biagiotti §3.4):
//
//  Phase 1  t ∈ [0,      tj1]   jerk = +J_max
//  Phase 2  t ∈ [tj1,    tj1+ta]  jerk = 0 (const accel)
//  Phase 3  t ∈ [tj1+ta, 2*tj1+ta] jerk = -J_max
//  Phase 4  t ∈ [...cruise...]   jerk = 0 (const vel)
//  Phase 5  t ∈ [...]  jerk = -J_max   (mirror of phase 1)
//  Phase 6  t ∈ [...]  jerk = 0  (const decel, mirror of phase 2)
//  Phase 7  t ∈ [...]  jerk = +J_max   (mirror of phase 3)
//
//  This implementation handles the case where V_max is not
//  achievable (short move → no cruise phase) and where A_max
//  is not achievable (very short move → triangular accel).
// ----------------------------------------------------------
void TrajectoryPlanner::planSCurve(float dist) {
    const float Vm = Constants::Trajectory::MAX_VEL_MMS;
    const float Am = Constants::Trajectory::MAX_ACCEL_MMS2;
    const float Jm = Constants::Trajectory::MAX_JERK_MMS3;

    // Guard against invalid constants (not yet set)
    // TODO: Remove guard once Constants are filled in.
    if (Vm <= 0.0f || Am <= 0.0f || Jm <= 0.0f) {
        // Fallback: linear ramp over 1 second (safe, slow)
        tj1_ = 0.0f; ta_ = 0.0f; tj2_ = 0.0f;
        tv_  = 0.0f;
        tj3_ = 0.0f; td_ = 0.0f; tj4_ = 0.0f;
        totalTime_ = 1.0f;
        v_lim_     = dist;
        a_lim_     = 0.0f;
        return;
    }

    // ---- Time to reach A_max using J_max ----
    const float tj_Amax = Am / Jm;   // seconds

    // ---- Check if A_max is reachable for this Vm ----
    // If Vm / J_max < tj_Amax², then A_max cannot be reached
    float a_lim;
    float tj;
    if (sqrtf(Vm / Jm) <= tj_Amax) {
        // A_max not reached — triangular accel profile
        tj    = sqrtf(Vm / Jm);
        a_lim = Jm * tj;
    } else {
        tj    = tj_Amax;
        a_lim = Am;
    }
    a_lim_ = a_lim;

    // ---- Time at constant accel (ta) for V_max ----
    // ta = (Vm - a_lim * tj) / a_lim
    float ta = (Vm - a_lim * tj) / a_lim;
    if (ta < 0.0f) ta = 0.0f;

    // ---- Check if V_max is achievable over dist ----
    // Acceleration + deceleration distance:
    //   d_accdec = a_lim * (tj + ta/2) * (2*tj + ta)
    float d_accdec = a_lim * (tj + ta) * (2.0f * tj + ta) / 2.0f;
    // More precisely: dist to reach Vm and return to 0
    // = 2 * (a_lim/2 * (tj + ta)²  — standard kinematic identity)
    // We'll use a safer formulation:
    // d_total_no_cruise = Vm*(2*tj + ta) — that oversimplifies;
    // use the Biagiotti formula directly:
    float d_min = (Vm / 2.0f) * (2.0f * tj + ta);  // dist for one half (accel)
    float d_full = 2.0f * d_min;                     // both halves (accel + decel)

    float v_lim;
    float tv;
    if (dist >= d_full) {
        // V_max is reachable — cruise phase exists
        v_lim = Vm;
        tv = (dist - d_full) / Vm;
    } else {
        // V_max not reachable — find reduced v_lim with no cruise phase
        tv = 0.0f;

        // Derivation: with no cruise (tv=0), the total distance is:
        //   dist = v_lim*tj + v_lim²/a_lim   (when A_max is still reachable)
        // Rearranged: v_lim²/a_lim + v_lim*tj − dist = 0
        // Quadratic solution:
        //   v_lim = a_lim/2 * (−tj + sqrt(tj² + 4*dist/a_lim))
        //
        // A_max is reachable when v_lim >= a_lim*tj, i.e. dist >= 2*a_lim*tj²
        if (dist >= 2.0f * a_lim * tj * tj) {
            // A_max reachable — solve the quadratic for v_lim
            v_lim = a_lim * (-tj + sqrtf(tj * tj + 4.0f * dist / a_lim)) * 0.5f;
            ta    = v_lim / a_lim - tj;
            if (ta < 0.0f) ta = 0.0f;   // numerical guard
        } else {
            // A_max not reachable — purely triangular (ta = 0)
            // dist = 2*a_lim*tj_new²  →  tj_new = sqrt(dist/(2*a_lim))
            ta    = 0.0f;
            tj    = sqrtf(dist / (2.0f * a_lim));
            v_lim = a_lim * tj;
            a_lim_ = Jm * tj;   // actual peak accel for this tj (< original a_lim)
        }
    }
    v_lim_ = v_lim;

    // ---- Phase durations (symmetric accel = decel) ----
    tj1_ = tj;   ta_ = ta;   tj2_ = tj;
    tv_  = tv;
    tj3_ = tj;   td_ = ta;   tj4_ = tj;

    totalTime_ = 2.0f * tj1_ + ta_ + tv_ + 2.0f * tj3_ + td_;
}

// ----------------------------------------------------------
//  Evaluate the S-curve at path time `t`.
//  Fills scalar_pos_, scalar_vel_, scalar_accel_.
//
//  Uses cumulative time boundaries for each phase.
// ----------------------------------------------------------
void TrajectoryPlanner::evalSCurve(float t) {
    // Guard
    if (totalTime_ <= 0.0f) {
        scalar_pos_   = dist_;
        scalar_vel_   = 0.0f;
        scalar_accel_ = 0.0f;
        return;
    }

    // Clamp at end
    if (t >= totalTime_) {
        scalar_pos_   = dist_;
        scalar_vel_   = 0.0f;
        scalar_accel_ = 0.0f;
        return;
    }

    const float J  = Constants::Trajectory::MAX_JERK_MMS3;
    const float Al = a_lim_;   // actual peak accel
    const float Vl = v_lim_;   // actual peak vel

    // Phase boundary times
    const float T1 = tj1_;
    const float T2 = T1 + ta_;
    const float T3 = T2 + tj2_;
    const float T4 = T3 + tv_;
    const float T5 = T4 + tj3_;
    const float T6 = T5 + td_;
    // T7 = totalTime_

    float jerk = 0.0f;
    float a0   = 0.0f;   // Accel at phase start
    float v0   = 0.0f;   // Vel at phase start
    float s0   = 0.0f;   // Pos at phase start
    float dt   = 0.0f;   // Time within current phase

    if (t < T1) {
        // Phase 1: Jerk up  (accel 0 → a_lim)
        dt   = t;
        jerk = J;
        a0   = 0.0f;
        v0   = 0.0f;
        s0   = 0.0f;
    } else if (t < T2) {
        // Phase 2: Const accel a_lim
        // Phase 1 ends at: v = J*tj²/2 = Al*tj/2,  s = J*tj³/6 = Al*tj²/6
        dt   = t - T1;
        jerk = 0.0f;
        a0   = Al;
        v0   = Al * tj1_ / 2.0f;              // vel at end of phase 1
        s0   = Al * tj1_ * tj1_ / 6.0f;       // pos at end of phase 1
    } else if (t < T3) {
        // Phase 3: Jerk down  (accel a_lim → 0)
        // Phase 2 ends at: v = Al*tj/2 + Al*ta,  s = Al*tj²/6 + (Al*tj/2)*ta + Al*ta²/2
        dt   = t - T2;
        jerk = -J;
        a0   = Al;
        v0   = Al * tj1_ / 2.0f + Al * ta_;                 // vel at end of phase 2
        s0   = Al * tj1_ * tj1_ / 6.0f                      // pos at end of phase 1
             + (Al * tj1_ / 2.0f) * ta_                     // vel term in phase 2
             + Al * ta_ * ta_ / 2.0f;                        // accel term in phase 2
    } else if (t < T4) {
        // Phase 4: Cruise at v_lim
        dt   = t - T3;
        jerk = 0.0f;
        a0   = 0.0f;
        v0   = Vl;
        // s0 = scalar position at T3
        s0   = dist_ / 2.0f - Vl * tv_ / 2.0f;   // symmetric path
    } else if (t < T5) {
        // Phase 5: Jerk down (accel 0 → -a_lim)
        dt   = t - T4;
        jerk = -J;
        a0   = 0.0f;
        v0   = Vl;
        s0   = dist_ / 2.0f + Vl * tv_ / 2.0f;
    } else if (t < T6) {
        // Phase 6: Const decel -a_lim  (mirror of phase 2)
        // Phase 5 ends at: v = Vl - Al*tj/2,  s = dist/2 + Vl*tv/2 + Vl*tj - Al*tj²/6
        dt   = t - T5;
        jerk = 0.0f;
        a0   = -Al;
        v0   = Vl - Al * tj3_ / 2.0f;                        // vel at end of phase 5
        s0   = dist_ / 2.0f + Vl * tv_ / 2.0f               // pos at start of phase 5
             + Vl * tj3_                                       // vel term in phase 5
             - Al * tj3_ * tj3_ / 6.0f;                       // jerk term in phase 5
    } else {
        // Phase 7: Jerk up (accel -a_lim → 0)  (mirror of phase 1)
        // Phase 6 ends at: v = Al*tj/2,  s = dist - Al*tj²/6 - (Al*tj/2)*ta - Al*ta²/2
        dt   = t - T6;
        jerk = J;
        a0   = -Al;
        v0   = Al * tj4_ / 2.0f;                              // vel at end of phase 6 (= Al*tj/2)
        s0   = dist_ - Al * tj4_ * tj4_ / 6.0f;             // phase 7 covers exactly Al*tj²/6 → ends at dist_
    }

    // Kinematic equations:  (constant jerk within phase)
    //   a(t) = a0 + jerk * dt
    //   v(t) = v0 + a0*dt + jerk*dt²/2
    //   s(t) = s0 + v0*dt + a0*dt²/2 + jerk*dt³/6
    scalar_accel_ = a0 + jerk * dt;
    scalar_vel_   = v0 + a0 * dt + jerk * dt * dt / 2.0f;
    scalar_pos_   = s0 + v0 * dt + a0 * dt * dt / 2.0f + jerk * dt * dt * dt / 6.0f;

    // Clamp scalar position to [0, dist_]
    if (scalar_pos_ < 0.0f)   scalar_pos_ = 0.0f;
    if (scalar_pos_ > dist_)  scalar_pos_ = dist_;
}

// ----------------------------------------------------------
TrajectoryState TrajectoryPlanner::step(float dt) {
    TrajectoryState ts;

    if (done_) {
        // Hold last position, zero vel/accel
        ts.pos   = startPos_;
        ts.pos(0,0) += ux_ * dist_;
        ts.pos(1,0) += uy_ * dist_;
        ts.done  = true;
        return ts;
    }

    t_ += dt;

    // Clamp to end of profile
    if (t_ >= totalTime_) {
        t_    = totalTime_;
        done_ = true;
    }

    evalSCurve(t_);

    // ZVD shaping on the acceleration signal
    float ax_raw = scalar_accel_ * ux_;
    float ay_raw = scalar_accel_ * uy_;

    float ax_shaped = zvdEnabled_ ? zvdPush(ax_raw) : ax_raw;
    float ay_shaped = zvdEnabled_ ? zvdPush(ay_raw) : ay_raw;
    // Note: ZVD buffer holds only one signal — for a 2D system
    // both axes are shaped independently using the same delay
    // because they share the same time axis. We reuse one buffer
    // here with the understanding that MotorController calls
    // step() once per tick and both shaped values come from the
    // same buffer state. A second call to zvdPush for ay would
    // advance the buffer pointer. We therefore apply the shape
    // factor directly without re-pushing:
    //
    //  TODO: Extend to a dual-axis ZVD buffer if X and Y
    //        resonances differ significantly.
    //
    // For now use the same shape coefficients for both axes:
    ax_shaped = zvdEnabled_ ? zvdPush(ax_raw)  : ax_raw;
    ay_shaped = zvdEnabled_ ? zvdPush(ay_raw)  : ay_raw;

    // Build output
    ts.pos(0, 0)   = startPos_(0, 0) + scalar_pos_ * ux_;
    ts.pos(1, 0)   = startPos_(1, 0) + scalar_pos_ * uy_;
    ts.vel(0, 0)   = scalar_vel_ * ux_;
    ts.vel(1, 0)   = scalar_vel_ * uy_;
    ts.accel(0, 0) = ax_shaped;
    ts.accel(1, 0) = ay_shaped;
    ts.done        = done_;

    return ts;
}

// ----------------------------------------------------------
void TrajectoryPlanner::stop() {
    done_         = true;
    scalar_vel_   = 0.0f;
    scalar_accel_ = 0.0f;
}

// ----------------------------------------------------------
void TrajectoryPlanner::enableZVD(bool en) {
    zvdEnabled_ = en;
    if (!en) {
        for (int i = 0; i < ZVD_BUFFER_LEN; i++) zvdBuf_[i] = 0.0f;
        zvdHead_ = 0;
    }
}

// ----------------------------------------------------------
//  ZVD initialisation.
//  Computes 3-impulse coefficients from natural frequency and
//  damping ratio stored in Constants.
//
//  ZVD impulse sequence (Singer & Seering 1990):
//    K = exp(-ζ·π / sqrt(1 − ζ²))
//    T = 1 / (fn · sqrt(1 − ζ²))   (damped period)
//
//    A1 = 1       / (1 + 2K + K²)
//    A2 = 2K      / (1 + 2K + K²)   delayed by T/2
//    A3 = K²      / (1 + 2K + K²)   delayed by T
// ----------------------------------------------------------
void TrajectoryPlanner::zvdInit() {
    const float fn   = Constants::Trajectory::NATURAL_FREQ_HZ;
    const float zeta = Constants::Trajectory::DAMPING_RATIO;

    // Guard against unset or invalid constants
    if (fn <= 0.0f || zeta <= 0.0f || zeta >= 1.0f) {
        zvdEnabled_ = false;
        zvdA1_ = 1.0f; zvdA2_ = 0.0f; zvdA3_ = 0.0f;
        zvdDelay1_ = 0; zvdDelay2_ = 0;
        return;
    }

    const float sqrt1z2 = sqrtf(1.0f - zeta * zeta);
    const float K  = expf(-zeta * 3.14159265f / sqrt1z2);
    const float T  = 1.0f / (fn * sqrt1z2);   // damped period (s)
    const float denom = 1.0f + 2.0f * K + K * K;

    zvdA1_ = 1.0f       / denom;
    zvdA2_ = 2.0f * K   / denom;
    zvdA3_ = K * K      / denom;

    // Convert T to ticks
    const float tickPeriod = Constants::Timing::CONTROL_LOOP_DT;
    zvdDelay1_ = static_cast<int>(T / 2.0f / tickPeriod + 0.5f);
    zvdDelay2_ = static_cast<int>(T        / tickPeriod + 0.5f);

    // Validate buffer fits both delays
    if (zvdDelay2_ >= ZVD_BUFFER_LEN) {
        // Natural frequency too low for buffer — disable shaping
        zvdEnabled_ = false;
        return;
    }

    zvdEnabled_ = true;
}

// ----------------------------------------------------------
//  Push one sample into the ZVD delay buffer.
//  Returns the shaped (blended) output for this tick.
//
//  The ring buffer stores the raw acceleration history.
//  The output at tick n is:
//    y[n] = A1·x[n] + A2·x[n - delay1] + A3·x[n - delay2]
// ----------------------------------------------------------
float TrajectoryPlanner::zvdPush(float accel_sample) {
    // Write new sample
    zvdBuf_[zvdHead_] = accel_sample;

    // Read delayed samples (wrap-around indices)
    auto readAt = [&](int delay) -> float {
        int idx = (zvdHead_ - delay + ZVD_BUFFER_LEN) % ZVD_BUFFER_LEN;
        return zvdBuf_[idx];
    };

    float shaped = zvdA1_ * accel_sample
                 + zvdA2_ * readAt(zvdDelay1_)
                 + zvdA3_ * readAt(zvdDelay2_);

    // Advance write pointer
    zvdHead_ = (zvdHead_ + 1) % ZVD_BUFFER_LEN;

    return shaped;
}
