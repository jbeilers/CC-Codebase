#pragma once

#include "math/Matrix.h"
#include "math/Constants.h"

// ============================================================
//  FeedForward.h  —  Inverse dynamics feed-forward for the
//                    CoreXY gantry.
//
//  Model:  F_ff = m·a_ref + b·v_ref
//
//  where:
//    m        — moving carriage mass (kg), from Constants
//    a_ref    — reference acceleration from TrajectoryPlanner (mm/s²)
//    b        — viscous friction coefficient (N/(mm/s)), from Constants
//    v_ref    — reference velocity from TrajectoryPlanner (mm/s)
//
//  Output units are mm/s² (acceleration command) — the same units
//  as the LQR output.  MotorController sums LQR + FF before
//  converting to step rate.
//
//  The friction term (b·v_ref) compensates for belt/rail drag.
//  If Constants::Gantry::FRICTION_COEFF == 0, this term is zero.
//  Identify b by commanding constant velocities and measuring the
//  steady-state LQR output at each velocity — b ≈ u_steady / v.
//
//  Usage:
//    Vec2 ff = feedForward_.compute(ref_accel, ref_vel);
//    // ff in mm/s² — add to LQR output before step conversion
// ============================================================

class FeedForward {
public:
    FeedForward() = default;

    // ----------------------------------------------------------
    //  Compute feed-forward acceleration command.
    //  ref_accel — [ax, ay] from TrajectoryPlanner (mm/s²)
    //  ref_vel   — [vx, vy] from TrajectoryPlanner (mm/s)
    //  Returns   — [ff_x, ff_y] in mm/s²
    // ----------------------------------------------------------
    Vec2 compute(const Vec2& ref_accel, const Vec2& ref_vel) const;

    // ----------------------------------------------------------
    //  Override mass at runtime (e.g. after weighing the tool
    //  payload).  Defaults to Constants::Gantry::MOVING_MASS_KG.
    // ----------------------------------------------------------
    void setMass(float kg) { mass_kg_ = kg; }

    // ----------------------------------------------------------
    //  Override friction coefficient at runtime (identify via
    //  constant-velocity sweep). Defaults to
    //  Constants::Gantry::FRICTION_COEFF.
    // ----------------------------------------------------------
    void setFriction(float b) { friction_b_ = b; }

private:
    float mass_kg_    = Constants::Gantry::MOVING_MASS_KG;
    float friction_b_ = Constants::Gantry::FRICTION_COEFF;
};
