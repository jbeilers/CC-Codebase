#pragma once

#include "math/Matrix.h"
#include "math/Constants.h"

// ============================================================
//  Kinematics.h  —  CoreXY forward and inverse kinematics.
//
//  CoreXY Kinematic Relationships
//  --------------------------------
//  Motor A  =  X + Y   (both belts contribute to both axes)
//  Motor B  =  X - Y
//
//  Inverse (Cartesian mm → motor steps):
//    a_steps = (x_mm + y_mm) × STEPS_PER_MM
//    b_steps = (x_mm - y_mm) × STEPS_PER_MM
//
//  Forward (motor encoder counts → Cartesian mm):
//    x_mm = (a_counts + b_counts) / (2 × STEPS_PER_MM)
//    y_mm = (a_counts - b_counts) / (2 × STEPS_PER_MM)
//
//  All methods are static — no instance needed.
// ============================================================

class Kinematics {
public:
    Kinematics() = delete;

    // ----------------------------------------------------------
    //  Motor encoder counts → Cartesian mm
    //  Returns Vec2 [ x_mm, y_mm ]
    // ----------------------------------------------------------
    static Vec2 motorToCartesian(float a_counts, float b_counts);

    // ----------------------------------------------------------
    //  Cartesian mm → motor step targets
    //  Returns Vec2 [ a_steps, b_steps ]
    // ----------------------------------------------------------
    static Vec2 cartesianToMotor(float x_mm, float y_mm);

    // ----------------------------------------------------------
    //  Cartesian velocity mm/s → motor velocity steps/s
    //  Returns Vec2 [ va_steps_s, vb_steps_s ]
    // ----------------------------------------------------------
    static Vec2 cartesianVelToMotorVel(float vx_mms, float vy_mms);

    // ----------------------------------------------------------
    //  Motor velocity steps/s → Cartesian velocity mm/s
    //  Returns Vec2 [ vx_mms, vy_mms ]
    // ----------------------------------------------------------
    static Vec2 motorVelToCartesianVel(float va_steps_s, float vb_steps_s);

    // ----------------------------------------------------------
    //  Clamp a Cartesian position to the machine workspace
    //  [0, X_TRAVEL_MM] × [0, Y_TRAVEL_MM].
    //  Returns clamped Vec2 [ x_mm, y_mm ]
    // ----------------------------------------------------------
    static Vec2 clampToWorkspace(float x_mm, float y_mm);

    // ----------------------------------------------------------
    //  Returns true if the given position is within workspace.
    // ----------------------------------------------------------
    static bool inWorkspace(float x_mm, float y_mm);
};
