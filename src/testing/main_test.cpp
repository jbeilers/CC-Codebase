// ============================================================
//  main_test.cpp  —  Entry point for all test builds.
//
//  Only one test function pair should be active at a time.
//  Comment out the active pair and uncomment the one you want.
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>

// ============================================================
//  Available tests — uncomment exactly ONE pair
// ============================================================

// ---- Control stack simulation (no hardware required) --------
// Runs Trajectory + Kalman + LQR + FeedForward with a simulated
// point-mass plant.  Outputs CSV over serial for offline plotting.
// extern void testControlSim_setup();
// extern void testControlSim_loop();

// ---- Combined hardware test (gantry + STS3215 servo) -----------
// H=home, G=gantry only, U=servo up, D=servo down, B=both, C=cal
// Servo commands gracefully skip if STS3215 is not connected.
// extern void testControlHw_setup();
// extern void testControlHw_loop();

// ---- Full transfer process simulation (10-step sequence) -------------------
// WASD+C to set reference, Q to E-stop, SPACE to advance each step.
// Includes open-loop gantry, TSL1401 scan, callus detection, grid transfer.
extern void testTransfer_setup();
extern void testTransfer_loop();

// ---- Comprehensive demonstration test (all 8 subsystems, mode-switchable) --
// ] / [ cycle modes, 1–8 jump directly.  No device classes — pure GPIO.
// Modes: D1 screws, servos 1&2, DC motors, gantry, camera, servos 3&4, STS3215, D2 screws.
// extern void testDemo_setup();
// extern void testDemo_loop();

// ---- Unified gantry test (open-loop + closed-loop, encoder readback) ---
// M = toggle mode, WASD = jog, P = position, R = reset, T = auto-test.
// Start in open-loop; press M to switch to closed-loop once wiring confirmed.
// extern void testGantry_setup();
// extern void testGantry_loop();

// ---- Raw encoder pin diagnostic (no library, no interrupts) ------------
// S = stream, C = change-count mode, R = single reading.
// Set PIN_A and PIN_B at the top of the file to test different pins.
// extern void testEncoderRaw_setup();
// extern void testEncoderRaw_loop();

// ---- CoreXY jog test (closed-loop only, legacy) -----
// Replaced by testGantry above — kept for reference.
// extern void testCoreXYJog_setup();
// extern void testCoreXYJog_loop();

// ---- CoreXY open-loop gantry test (legacy) ------
// Replaced by testGantry above — kept for reference.
// extern void testGantryOpenLoop_setup();
// extern void testGantryOpenLoop_loop();

// ---- Pre-flight hardware check ----------------------------------
// Auto-runs all subsystem checks (ping, ADC, encoder stability)
// then prompts for interactive motion tests (gantry jog, servos).
// Run this after any wiring change before initiating a transfer.
// extern void testHwCheck_setup();
// extern void testHwCheck_loop();

// ---- DC motor test (L298N dual H-bridge) --------------------
// W/S = Motor A fwd/rev, I/K = Motor B fwd/rev, T = auto-test.
// Fill in DCMotors::MOTOR_x_IN pins in PinMap.h first.
// extern void testDCMotors_setup();
// extern void testDCMotors_loop();

// ---- Lead-screw NEMA 17 test (TB6600 STEP/DIR) --------------
// 1-4 = select screw, A = all, U/D = up/down 15mm, T = auto-test.
// Set MS_SCREWx to match TB6600 DIP switch, fill LeadScrews pins.
// extern void testLeadScrews_setup();
// extern void testLeadScrews_loop();

// ---- Device 1 lift + push sequence --------------------------
// R = full sequence (lift → settle → push servo bar).
// H = reset (pull back → lower).  L/D/P/B = individual steps.
// Set LIFT_MM, SETTLE_MS, PUSH_DEG, SERVO_MASK in test file.
// extern void testDevice1Push_setup();
// extern void testDevice1Push_loop();

// ---- MG996R PWM servo test ----------------------------------
// 1-4 = select servo, A/D = min/max, C = centre, W/S = step,
// T = auto-sweep all wired servos.
// Fill in Servos::MG996R_x pins in PinMap.h first.
// extern void testMG996R_setup();
// extern void testMG996R_loop();

// ---- TSL1401 CCD + line laser test --------------------------
// R = single frame CSV, C = continuous mode, W = ASCII waveform.
// Laser fires automatically during every capture.
// Fill in LaserSensor::SI, CLK and LineLaser::ENABLE in PinMap.h.
// extern void testTSL1401_setup();
// extern void testTSL1401_loop();

// ============================================================

void setup() {
    // testControlSim_setup();
    // testControlHw_setup();
    testTransfer_setup();
    // testDemo_setup();
    // testGantry_setup();
    // testCoreXYJog_setup();
    // testGantryOpenLoop_setup();
    // testHwCheck_setup();
    // testDCMotors_setup();
    // testMG996R_setup();
    // testTSL1401_setup();
    // testLeadScrews_setup();
    // testDevice1Push_setup();
}

void loop() {
    testTransfer_loop();
    // testDemo_loop();
    // testGantry_loop();
    // testCoreXYJog_loop();
    // testGantryOpenLoop_loop();
    // testControlSim_loop();
    // testControlHw_loop();
    // testHwCheck_loop();
    // testDCMotors_loop();
    // testMG996R_loop();
    // testTSL1401_loop();
    // testLeadScrews_loop();
    // testDevice1Push_loop();
}

#endif // TEST_MODE
