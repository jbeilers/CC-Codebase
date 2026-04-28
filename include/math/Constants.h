#pragma once

#include <cstdint>

// ============================================================
//  Constants.h  —  All physical constants, tuning parameters,
//                  and control matrix definitions.
//
//  RULES:
//    - No logic, no includes of project files, no pin references.
//    - All values in SI units unless explicitly noted.
//    - TODO comments mark every value that needs confirmation.
// ============================================================

namespace Constants {

    // ----------------------------------------------------------
    //  System Timing
    // ----------------------------------------------------------
    namespace Timing {
        constexpr uint32_t CONTROL_LOOP_HZ      = 1000;                         // Kalman + LQR + FF update rate (Hz)
        constexpr float    CONTROL_LOOP_DT       = 1.0f / CONTROL_LOOP_HZ;      // Derived timestep (seconds)
        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;                         // HP PC connection check interval (ms)
        constexpr uint32_t SERIAL_TIMEOUT_MS     = 3000;                         // How long before connection deemed lost (ms) // TODO: Adjust based on observed PC response latency
    }

    // ----------------------------------------------------------
    //  Serial Communication
    // ----------------------------------------------------------
    namespace Serial {
        constexpr uint32_t BAUD_RATE = 115200;                                   // Must match platformio.ini SERIAL_BAUD
    }

    // ----------------------------------------------------------
    //  CoreXY Gantry — Geometry & Mechanics
    // ----------------------------------------------------------
    namespace Gantry {
        constexpr float BELT_PITCH_MM        = 2.0f;   // Set belt pitch in mm (e.g. 2.0f for GT2, 3.0f for GT3)
        constexpr int   MOTOR_STEPS_PER_REV  = 200;      // Steps/rev of 17HS24-2004D-E1K (typically 200)
        constexpr int   MICROSTEPPING        = 4;       // TODO: Set TB6600 microstepping setting (e.g. 1, 2, 4, 8, 16)
        constexpr int   PULLEY_TEETH         = 20;       // Number of teeth on the drive pulley

        // Derived — do not edit directly
        constexpr float STEPS_PER_MM = (MOTOR_STEPS_PER_REV * MICROSTEPPING)
                                       / (BELT_PITCH_MM * PULLEY_TEETH);         // Will auto-resolve once above are set

        constexpr float TRAVEL_X_MM  = 350.0f;          // Maximum X travel distance in mm
        constexpr float TRAVEL_Y_MM  = 250.0f;          // Maximum Y travel distance in mm
        constexpr float X_TRAVEL_MM  = TRAVEL_X_MM;     // Alias used by Kinematics::clampToWorkspace
        constexpr float Y_TRAVEL_MM  = TRAVEL_Y_MM;     // Alias used by Kinematics::clampToWorkspace

        constexpr float CARRIAGE_MASS_KG = 1.0f;       // TODO: Measured mass of the moving carriage + tool in kg
                                                        // NOTE: Keep at 1.0 for simulation — FeedForward outputs mass*accel,
                                                        //       but the sim plant integrates commands as direct acceleration (unit mass).
                                                        //       Set to real mass only when the hardware motor driver divides by mass.
        constexpr float MOVING_MASS_KG   = CARRIAGE_MASS_KG; // Alias used by FeedForward
        constexpr float FRICTION_COEFF   = 0.0f;       // TODO: Identify via constant-vel sweep (N or mm/s² equivalent)
                                                        // NOTE: Zero for simulation — not modelled in the LQR plant

        constexpr float MAX_SPEED_MM_S   = 30.0f;        // TODO: Increase gradually once step-loss is confirmed gone; was 80
        constexpr float MAX_ACCEL_MM_S2  = 60.0f;        // TODO: Increase gradually once step-loss is confirmed gone; was 150

        // Homing
        constexpr float HOMING_SPEED_MMS        = 10.0f;  // TODO: Crawl speed toward endstops during homing (mm/s); decrease if endstop overrun is observed

        // Soft / hard border limits
        constexpr float SOFT_LIMIT_MARGIN_MM     = 10.0f; // Begin decelerating when within this distance of any workspace border (mm)
        constexpr float HARD_LIMIT_MARGIN_MM     = 2.0f;  // E-stop if Kalman estimate drifts this far past any border (mm)

        // Closed-loop fault detection
        constexpr float FOLLOWING_ERROR_FAULT_MM = 2.0f; // TODO: Max allowed gap between commanded and encoder position (mm); triggers FAULT if exceeded (start ~2.0f)
    }

    // ----------------------------------------------------------
    //  Lead Screw Steppers — NEMA 17
    // ----------------------------------------------------------
    namespace LeadScrews {
        constexpr float LEAD_MM_PER_REV     = 8.0f;    // Lead of screw in mm/rev (e.g. 8.0f for T8 leadscrew)
        constexpr int   MOTOR_STEPS_PER_REV = 200;        // Steps/rev (typically 200 for NEMA 17)
        constexpr int   MICROSTEPPING       = 1;         // TODO: TB6600 microstepping setting

        // Derived
        constexpr float STEPS_PER_MM = (MOTOR_STEPS_PER_REV * MICROSTEPPING)
                                       / LEAD_MM_PER_REV;                        // TODO: Will auto-resolve once above are set

        constexpr float MAX_SPEED_MM_S  = 10.0f;        // TODO: Maximum lead screw speed in mm/s
        constexpr float MAX_ACCEL_MM_S2 = 20.0f;        // TODO: Maximum lead screw acceleration in mm/s²
    }

    // ----------------------------------------------------------
    //  Servos
    // ----------------------------------------------------------
    namespace Servos {

        // STS3215 Smart Servo (TTL UART)
        constexpr uint32_t STS3215_BAUD_RATE  = 1000000;                        // STS3215 factory default baud rate
        constexpr uint8_t  STS3215_SERVO_ID   = 1;                              // TODO: Set to the ID programmed into your STS3215 (default is usually 1)
        constexpr float    STS3215_MAX_TORQUE  = 2.0f;                         // TODO: Set operating torque limit (max rated: 30 kg*cm = ~2.94 N*m)

        // MG996R Standard Servo (PWM, 50 Hz)
        constexpr float MG996R_PWM_FREQ_HZ    = 50.0f;                          // Standard servo PWM frequency
        constexpr float MG996R_MIN_PULSE_US   = 500.0f;                         // TODO: Verify min pulse width in microseconds for your MG996R batch
        constexpr float MG996R_MAX_PULSE_US   = 2500.0f;                        // TODO: Verify max pulse width in microseconds for your MG996R batch
        constexpr float MG996R_MIN_ANGLE_DEG  = 0.0f;
        constexpr float MG996R_MAX_ANGLE_DEG  = 180.0f;
    }

    // ----------------------------------------------------------
    //  DC Motors (L298N)
    // ----------------------------------------------------------
    namespace DCMotors {
        constexpr float PWM_FREQ_HZ   = 1000.0f;        // TODO: Tune PWM frequency for your DC motors (1–20 kHz typical)
        constexpr float MAX_DUTY      = 1.0f;            // 0.0 – 1.0 normalized duty cycle
    }

    // ----------------------------------------------------------
    //  TSL1401 Line Sensor Array
    // ----------------------------------------------------------
    namespace LaserSensor {
        constexpr int   PIXEL_COUNT          = 128;      // Fixed: TSL1401 always has 128 pixels
        constexpr float CLK_FREQ_HZ          = 250000.0f;  // TODO: Tune (50 kHz – 8 MHz); raise if capture is slow, lower if pixels are noisy
        constexpr float INTEGRATION_TIME_US  = 50000.0f;   // TODO: Tune for your lighting; raise if laser peak is too dim, lower if saturating
        constexpr float PIXEL_PITCH_UM       = 63.5f;   // Fixed: TSL1401 pixel pitch is 63.5 µm

        // Data cleanup / filtering
        constexpr float NOISE_THRESHOLD      = 5.0f;   // TODO: Set ADC noise floor threshold (0–1023 range); calibrate empirically
        constexpr int   SMOOTHING_WINDOW     = 5;       // TODO: Set moving-average window size for laser line smoothing (e.g. 3–7)
    }

    // ----------------------------------------------------------
    //  Microswitch
    // ----------------------------------------------------------
    namespace Switches {
        constexpr uint32_t DEBOUNCE_MS = 20;             // TODO: Adjust if false triggers occur (increase) or response is sluggish (decrease)
    }

    // ----------------------------------------------------------
    //  Trajectory Planner
    // ----------------------------------------------------------
    namespace Trajectory {
        constexpr float MAX_JERK_MM_S3  = 500.0f;         // Scaled with reduced speed/accel (was 2000)
        constexpr float MAX_JERK_MMS3   = MAX_JERK_MM_S3; // Alias used by TrajectoryPlanner

        // Aliases matching Gantry limits, used by TrajectoryPlanner and MotorController
        constexpr float MAX_VEL_MMS     = Gantry::MAX_SPEED_MM_S;
        constexpr float MAX_ACCEL_MMS2  = Gantry::MAX_ACCEL_MM_S2;

        // ZVD Input Shaper coefficients
        // Compute offline: A1 = 1/(1+2K+K²), t1 = 0
        //                  A2 = 2K/(1+2K+K²), t2 = 0.5/fn
        //                  A3 = K²/(1+2K+K²), t3 = 1.0/fn
        // where K = e^(-zeta*pi/sqrt(1-zeta²)), fn = natural frequency (Hz)
        constexpr float ZVD_NATURAL_FREQ_HZ = 0.0f;      // 0 disables ZVD (zvdInit() guard rejects fn<=0).
                                                          // TODO: Set after measuring resonance on real hardware (tap test or step response, typically 10–50 Hz for belt gantry)
        constexpr float ZVD_DAMPING_RATIO   = 0.1f;    // TODO: Measure or estimate damping ratio (typical: 0.05 – 0.2)
        constexpr float NATURAL_FREQ_HZ     = ZVD_NATURAL_FREQ_HZ; // Alias used by TrajectoryPlanner
        constexpr float DAMPING_RATIO       = ZVD_DAMPING_RATIO;   // Alias used by TrajectoryPlanner
    }

    // ----------------------------------------------------------
    //  Kalman Filter
    //  State vector: [x, ẋ, y, ẏ]  (4x1)
    //  These are initial values — tune after observing real noise
    // ----------------------------------------------------------
    namespace Kalman {
        // Process noise covariance Q (how much we trust the model)
        constexpr float Q_POS  = 0.001f;                 // TODO: Set position process noise variance (start ~1e-4)
        constexpr float Q_VEL  = 0.02f;                 // TODO: Set velocity process noise variance (start ~1e-2)

        // Measurement noise covariance R (how much we trust the encoder)
        constexpr float R_ENC  = 0.01f;                 // TODO: Set encoder measurement noise variance (start ~1e-3; measure stationary encoder jitter)

        // Initial state estimate covariance P
        constexpr float P0_POS = 1.0f;                 // TODO: Initial position uncertainty (start ~1.0)
        constexpr float P0_VEL = 1.0f;                 // TODO: Initial velocity uncertainty (start ~1.0)
    }

    // ----------------------------------------------------------
    //  LQR Controller
    //  One LQRController instance per Cartesian axis (X and Y).
    //  Augmented state per axis: [ error,  error_dot,  ∫error ]
    //  Gain vector K is 1×3: [ Kp,  Kd,  Ki ]
    //
    //  Solve offline:
    //    Python:  K, S, E = control.dlqr(F_aug, B_aug, Q_aug, R_aug)
    //    MATLAB:  [K, S, E] = dlqr(F_aug, B_aug, Q_aug, R_aug)
    //
    //  Both axes share the same K (symmetric gantry assumption).
    //  If your X and Y axes have meaningfully different dynamics,
    //  define separate K_X and K_Y arrays.
    // ----------------------------------------------------------
    namespace LQR {
        // Gain vector [ Kp, Kd, Ki ] — replace -1.0f once dlqr is solved.
        // TODO: Run offline dlqr solve and fill in real values.

        // Integrator anti-windup clamp (mm·s).
        // Prevents integral wind-up during large errors or motor saturation.
        // TODO: Set to ~10–20% of total axis travel in mm·s once travel is known.

        constexpr float K[1][3] = {
            { 16.590945f, 6.556290f, 0.0f }
        };
        constexpr float INTEGRAL_CLAMP = 20.0f;

    }

    // ----------------------------------------------------------
    //  Resource / Amperage Budget
    //  Used by ResourceMutex to gate simultaneous actuator use.
    // ----------------------------------------------------------
    namespace Power {
        constexpr float MAX_TOTAL_AMPS      = 14.5f;    // TODO: Set total current budget from your PSU in amps
        constexpr float GANTRY_MOTOR_AMPS   = 2.0f;    // TODO: Measured/rated current draw per gantry stepper
        constexpr float LEADSCREW_MOTOR_AMPS= 1.0f;    // TODO: Measured/rated current draw per lead screw stepper
        constexpr float STS3215_AMPS        = 3.0f;    // TODO: Measured/rated peak current draw of STS3215
        constexpr float MG996R_AMPS         = 2.5f;    // TODO: Measured/rated peak current draw per MG996R (typ. 0.5–1.0A stall)
        constexpr float DC_MOTOR_AMPS       = 2.0f;    // TODO: Measured/rated current draw per DC motor
    }

} // namespace Constants