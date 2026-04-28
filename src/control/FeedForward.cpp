#include "control/FeedForward.h"

// ============================================================
//  FeedForward.cpp
// ============================================================

Vec2 FeedForward::compute(const Vec2& ref_accel, const Vec2& ref_vel) const {
    Vec2 ff;

    // Inertia term: F = m·a
    // Units: kg × mm/s² → we keep everything in mm/s² since
    //        LQR gains were designed in that unit space too.
    //        The mass scaling cancels in the gain design.
    // TODO: If LQR is designed in force units (N) rather than
    //       acceleration units, multiply through by mass here
    //       and scale LQR gains accordingly.
    ff(0, 0) = mass_kg_ * ref_accel(0, 0)   // x inertia
             + friction_b_ * ref_vel(0, 0);  // x friction

    ff(1, 0) = mass_kg_ * ref_accel(1, 0)   // y inertia
             + friction_b_ * ref_vel(1, 0);  // y friction

    return ff;
}
