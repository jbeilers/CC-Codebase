// ============================================================
//  testing/test_control_sim.cpp
//
//  PURPOSE
//  -------
//  Validates the full control stack (TrajectoryPlanner + FeedForward +
//  KalmanFilter + LQRController) using a simulated point-mass plant.
//  No hardware required — runs entirely in software.
//
//  PLANT MODEL
//  -----------
//  Simple Euler-integrated double integrator per axis:
//    accel_cmd = feedforward + LQR_feedback
//    sim_vel  += accel_cmd * dt
//    sim_pos  += sim_vel   * dt
//
//  The simulated Cartesian position is fed back to the Kalman filter
//  as a noiseless encoder measurement each tick.  This lets you verify
//  that the trajectory profile shape and the LQR settling behaviour
//  are correct before connecting real hardware.
//
//  OUTPUT (CSV, printed at 100 Hz over USB Serial)
//  ------------------------------------------------
//  Columns: t_ms, ref_x, ref_y, sim_x, sim_y, est_x, est_y,
//           accel_x, accel_y, err_x, err_y
//
//    t_ms    — elapsed milliseconds since last 'G' command
//    ref_x/y — trajectory planner reference position (mm)
//    sim_x/y — simulated plant position (mm)
//    est_x/y — Kalman filter position estimate (mm)
//    accel_x/y — total acceleration command issued to plant (mm/s²)
//    err_x/y — position tracking error: ref - sim (mm)
//
//  Paste the CSV output into a spreadsheet or plot with Python/MATLAB
//  to inspect profile shape, overshoot, and settling time.
//
//  CONTROLS (single character over serial, no newline needed)
//  -----------------------------------------------------------
//    G  — start move to (SIM_TARGET_X, SIM_TARGET_Y)
//    R  — reset plant + filter to origin (0, 0); stops any active move
//    S  — stop and hold current position
//    Z  — toggle ZVD input shaper on/off
//    ?  — print this help
//
//  TUNING NOTES
//  ------------
//  The following Constants.h values drive the simulation:
//    Trajectory::MAX_VEL_MMS     — cruise speed (mm/s)
//    Trajectory::MAX_ACCEL_MMS2  — peak acceleration (mm/s²)
//    Trajectory::MAX_JERK_MMS3   — peak jerk (mm/s³)
//    LQR::K                      — gain vector [Kp, Kd, Ki]
//    LQR::INTEGRAL_CLAMP         — anti-windup bound
//    Gantry::MOVING_MASS_KG      — feedforward mass term
//    Gantry::FRICTION_COEFF      — feedforward friction term
//
//  Update the target position SIM_TARGET_X / SIM_TARGET_Y below to
//  test different move distances.
//
//  COMPILE
//  -------
//  platformio run -e teensy41_test
//  (main_test.cpp must forward to testControlSim_setup/loop)
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>
#include "control/TrajectoryPlanner.h"
#include "control/KalmanFilter.h"
#include "control/LQRController.h"
#include "control/FeedForward.h"

// ============================================================
//  Sim parameters — edit freely
// ============================================================

// Target position for 'G' command (mm, Cartesian)
static constexpr float SIM_TARGET_X = 250.0f;
static constexpr float SIM_TARGET_Y = 250.0f;

// Control loop period (must match Constants::Timing::CONTROL_LOOP_DT)
static constexpr float SIM_DT_S = 0.001f;   // 1 kHz

// CSV output decimation: print every N ticks → 1000/N Hz output rate
// At N=10 → 100 Hz output.  Reduce N for more resolution (more output).
static constexpr int CSV_PRINT_EVERY_N_TICKS = 10;

// ============================================================
//  Module instances — file scope
// ============================================================
static TrajectoryPlanner planner;
static KalmanFilter      kf;
static LQRController     lqrX;
static LQRController     lqrY;
static FeedForward       ff;

// ============================================================
//  Simulated plant state (point mass, Euler integration)
// ============================================================
static float simX   = 0.0f;
static float simY   = 0.0f;
static float simVx  = 0.0f;
static float simVy  = 0.0f;

// ============================================================
//  Runtime state
// ============================================================
static bool    running       = false;   // True while a move is in progress
static uint32_t tickCount    = 0;       // Ticks since last 'G' command
static uint32_t moveStartUs  = 0;       // micros() at last 'G' command
static bool    csvHeaderPrinted = false;

// ============================================================
//  Forward declarations
// ============================================================
static void doReset();
static void printHelp();
static void printCsvHeader();
static void printCsvRow(uint32_t t_ms,
                        float ref_x,  float ref_y,
                        float sim_x,  float sim_y,
                        float est_x,  float est_y,
                        float ax,     float ay,
                        float ref_vx, float ref_vy,
                        float sim_vx, float sim_vy);

// ============================================================
//  doReset — zero plant, filter, controllers, planner
// ============================================================
static void doReset() {
    simX  = 0.0f;  simY  = 0.0f;
    simVx = 0.0f;  simVy = 0.0f;

    kf.reset(0.0f, 0.0f);
    lqrX.reset();
    lqrY.reset();
    planner.stop();

    running    = false;
    tickCount  = 0;

    Serial.println(F("# RESET — plant and filter zeroed."));
}

// ============================================================
//  printHelp
// ============================================================
static void printHelp() {
    Serial.println(F("# =============================="));
    Serial.println(F("# Control Stack Simulation Test"));
    Serial.println(F("# ------------------------------"));
    Serial.println(F("#  G — move to target (SIM_TARGET_X, SIM_TARGET_Y)"));
    Serial.println(F("#  R — reset plant + filter to origin"));
    Serial.println(F("#  S — stop and hold"));
    Serial.println(F("#  Z — toggle ZVD shaper on/off"));
    Serial.println(F("#  ? — this help"));
    Serial.println(F("# ------------------------------"));
    Serial.print(F("# Target:   X="));  Serial.print(SIM_TARGET_X, 1);
    Serial.print(F(" mm  Y="));         Serial.print(SIM_TARGET_Y, 1);
    Serial.println(F(" mm"));
    Serial.print(F("# Max vel:  "));    Serial.print(Constants::Trajectory::MAX_VEL_MMS, 1);
    Serial.println(F(" mm/s"));
    Serial.print(F("# Max accel:"));    Serial.print(Constants::Trajectory::MAX_ACCEL_MMS2, 1);
    Serial.println(F(" mm/s²"));
    Serial.print(F("# Max jerk: "));    Serial.print(Constants::Trajectory::MAX_JERK_MMS3, 1);
    Serial.println(F(" mm/s³"));
    Serial.print(F("# LQR K:    ["));
    Serial.print(Constants::LQR::K[0][0], 3); Serial.print(F(", "));
    Serial.print(Constants::LQR::K[0][1], 3); Serial.print(F(", "));
    Serial.print(Constants::LQR::K[0][2], 3); Serial.println(F("]"));
    Serial.print(F("# ZVD:      "));
    Serial.println(planner.isZVDEnabled() ? F("enabled") : F("disabled"));
    Serial.println(F("# =============================="));
}

// ============================================================
//  printCsvHeader — printed once before data rows
// ============================================================
static void printCsvHeader() {
    Serial.println(F("t_ms,ref_x,ref_y,sim_x,sim_y,est_x,est_y,accel_x,accel_y,err_x,err_y,ref_vx,ref_vy,sim_vx,sim_vy"));
    csvHeaderPrinted = true;
}

// ============================================================
//  printCsvRow
// ============================================================
static void printCsvRow(uint32_t t_ms,
                        float ref_x,  float ref_y,
                        float sim_x,  float sim_y,
                        float est_x,  float est_y,
                        float ax,     float ay,
                        float ref_vx, float ref_vy,
                        float sim_vx, float sim_vy)
{
    Serial.print(t_ms);        Serial.print(',');
    Serial.print(ref_x, 4);    Serial.print(',');
    Serial.print(ref_y, 4);    Serial.print(',');
    Serial.print(sim_x, 4);    Serial.print(',');
    Serial.print(sim_y, 4);    Serial.print(',');
    Serial.print(est_x, 4);    Serial.print(',');
    Serial.print(est_y, 4);    Serial.print(',');
    Serial.print(ax, 4);       Serial.print(',');
    Serial.print(ay, 4);       Serial.print(',');
    Serial.print(ref_x - sim_x, 4);  Serial.print(',');
    Serial.print(ref_y - sim_y, 4);  Serial.print(',');
    Serial.print(ref_vx, 4);   Serial.print(',');
    Serial.print(ref_vy, 4);   Serial.print(',');
    Serial.print(sim_vx, 4);   Serial.print(',');
    Serial.println(sim_vy, 4);
}

// ============================================================
//  testControlSim_setup
// ============================================================
void testControlSim_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    kf.begin(SIM_DT_S);
    planner.begin();
    planner.enableZVD(false);   // Disabled until natural frequency is measured on hardware
    // lqrX and lqrY use Constants::LQR::K from their constructors

    printHelp();
    Serial.println(F("# Send 'G' to start a simulated move."));
}

// ============================================================
//  testControlSim_loop — called at ~1 kHz by loop()
// ============================================================
void testControlSim_loop() {

    // ---- Heartbeat: print "# alive" every 3 seconds while idle ----
    // Remove once serial input is confirmed working.
    if (!running) {
        static uint32_t lastHeartbeatMs = 0;
        uint32_t now = millis();
        if (now - lastHeartbeatMs >= 3000) {
            lastHeartbeatMs = now;
            Serial.println(F("# alive — send G/R/S/Z/?"));
        }
    }

    // ---- Handle serial commands ----
    if (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        switch (c) {
            case 'g': case 'G':
                doReset();
                planner.enableZVD(false);   // Force off — ZVD needs a measured fn before use
                planner.moveTo(SIM_TARGET_X, SIM_TARGET_Y, simX, simY);
                lqrX.reset();
                lqrY.reset();
                running       = true;
                tickCount     = 0;
                moveStartUs   = micros();
                printCsvHeader();   // Always print header so logger captures every run
                Serial.print(F("# GO (ZVD off) — moving to X="));
                Serial.print(SIM_TARGET_X, 1);
                Serial.print(F(" Y="));
                Serial.println(SIM_TARGET_Y, 1);
                break;

            case 'r': case 'R':
                doReset();
                csvHeaderPrinted = false;
                break;

            case 's': case 'S':
                planner.stop();
                running = false;
                lqrX.reset();
                lqrY.reset();
                Serial.println(F("# STOP"));
                break;

            case 'z': case 'Z': {
                bool en = !planner.isZVDEnabled();
                planner.enableZVD(en);
                Serial.print(F("# ZVD "));
                Serial.println(en ? F("enabled") : F("disabled"));
                break;
            }

            case '?':
                printHelp();
                break;

            default:
                // Echo unrecognised characters so we can see what's arriving.
                // Remove once input is confirmed working.
                if (c >= 0x20 && c < 0x7F) {   // printable ASCII only
                    Serial.print(F("# unknown: '"));
                    Serial.print(c);
                    Serial.println(F("'"));
                }
                break;
        }
    }

    if (!running) {
        delayMicroseconds(static_cast<unsigned int>(SIM_DT_S * 1e6f));
        return;
    }

    // ---- 1. Trajectory step ----
    TrajectoryState ts = planner.step(SIM_DT_S);

    // ---- 2. Feed-forward ----
    Vec2 ffCmd = ff.compute(ts.accel, ts.vel);

    // ---- 3. Set LQR references ----
    lqrX.setReference(ts.pos(0, 0), ts.vel(0, 0));
    lqrY.setReference(ts.pos(1, 0), ts.vel(1, 0));

    // ---- 4. Kalman: predict with feed-forward ----
    kf.predict(ffCmd, SIM_DT_S);

    // ---- 5. Kalman: correct with simulated plant position ----
    Vec2 z;
    z(0, 0) = simX;
    z(1, 0) = simY;
    kf.update(z);

    // ---- 6. LQR feedback ----
    float fbX = lqrX.update(kf.getX(), kf.getVx(), SIM_DT_S);
    float fbY = lqrY.update(kf.getY(), kf.getVy(), SIM_DT_S);

    // ---- 7. Total acceleration command ----
    float accelX = ffCmd(0, 0) + fbX;
    float accelY = ffCmd(1, 0) + fbY;

    // Clamp the 2D acceleration vector magnitude, not each axis independently.
    // Per-axis clamping breaks the X:Y ratio (e.g. X hits the limit while Y
    // does not), causing the plant to drift off the intended path direction.
    // Clamping the vector norm and rescaling both axes preserves the ratio.
    constexpr float MAX_A = Constants::Trajectory::MAX_ACCEL_MMS2;
    const float accelMag = sqrtf(accelX * accelX + accelY * accelY);
    if (accelMag > MAX_A) {
        const float scale = MAX_A / accelMag;
        accelX *= scale;
        accelY *= scale;
    }

    // ---- 8. Integrate plant (Euler, point mass) ----
    simVx += accelX * SIM_DT_S;
    simVy += accelY * SIM_DT_S;
    simX  += simVx  * SIM_DT_S;
    simY  += simVy  * SIM_DT_S;

    // ---- 9. CSV output (decimated) ----
    if (tickCount % CSV_PRINT_EVERY_N_TICKS == 0) {
        uint32_t t_ms = (micros() - moveStartUs) / 1000UL;
        printCsvRow(t_ms,
                    ts.pos(0, 0), ts.pos(1, 0),
                    simX,         simY,
                    kf.getX(),    kf.getY(),
                    accelX,       accelY,
                    ts.vel(0, 0), ts.vel(1, 0),
                    simVx,        simVy);
    }

    // ---- 10. Detect move completion ----
    if (ts.done) {
        // Keep printing for 1 more second of settling
        static uint32_t doneAt = 0;
        if (doneAt == 0) {
            doneAt = tickCount;
            Serial.println(F("# Trajectory complete — printing 1s settling data..."));
        }
        if (tickCount - doneAt >= 1000) {
            running = false;
            doneAt  = 0;
            Serial.println(F("# Done.  Send 'R' to reset or 'G' to repeat."));
        }
    }

    tickCount++;

    // ---- 11. Maintain 1 kHz loop rate ----
    delayMicroseconds(static_cast<unsigned int>(SIM_DT_S * 1e6f));
}

#endif // TEST_MODE
