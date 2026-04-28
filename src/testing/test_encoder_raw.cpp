// ============================================================
//  testing/test_encoder_raw.cpp
//
//  PURPOSE
//  -------
//  Raw pin-level encoder diagnostic. Reads encoder signal pins
//  directly via digitalRead() — no Encoder library, no interrupts,
//  no ClosedLoopStepper. Just bare GPIO.
//
//  Use this to determine whether:
//    1. The Teensy pin is reading correctly
//    2. The encoder channel is actually outputting a signal
//
//  PROCEDURE
//  ---------
//  1. Set PIN_A and PIN_B below to the pins you want to test.
//  2. Upload and open serial monitor at 115200 baud.
//  3. Press 'S' to start the live stream.
//  4. Slowly rotate the motor shaft by hand.
//
//  WHAT TO LOOK FOR
//  ----------------
//  - Both channels should toggle between 0 and 1 as you rotate.
//  - If a channel stays stuck at 0 or 1 regardless of rotation,
//    that channel is dead or not reaching the pin.
//  - Try changing PIN_A to a different Teensy pin and rewiring
//    the encoder signal to that pin — if it then toggles, the
//    original pin is faulty.
//  - If no pin ever reads the signal, the encoder channel itself
//    is dead.
//
//  CONTROLS
//  --------
//    S  — Toggle live stream on/off (prints every 50 ms)
//    R  — Single reading (one snapshot print)
//    C  — Count mode: prints a counter each time pin A changes state
//    ?  — Help
//
//  COMPILE
//  -------
//  Activate in main_test.cpp:
//    extern void testEncoderRaw_setup();
//    extern void testEncoderRaw_loop();
//  platformio run -e teensy41_test -t upload
// ============================================================

#ifdef TEST_MODE

#include <Arduino.h>

// ============================================================
//  CONFIGURATION — change these to test different pins
// ============================================================
static constexpr int PIN_A = 34;   // encoder A channel pin to test
static constexpr int PIN_B = 36;   // encoder B channel pin to test

// ============================================================
//  STATE
// ============================================================
static bool     g_stream      = false;
static uint32_t g_streamMs    = 0;
static constexpr uint32_t STREAM_INTERVAL_MS = 50;

// Change-count mode
static bool    g_countMode   = false;
static int     g_lastA       = -1;   // -1 = unread
static int     g_lastB       = -1;
static int32_t g_countA      = 0;
static int32_t g_countB      = 0;

// ============================================================
//  HELPERS
// ============================================================
static void printReading() {
    int a = digitalRead(PIN_A);
    int b = digitalRead(PIN_B);
    Serial.print(F("PIN_A("));
    Serial.print(PIN_A);
    Serial.print(F(")="));
    Serial.print(a);
    Serial.print(F("  PIN_B("));
    Serial.print(PIN_B);
    Serial.print(F(")="));
    Serial.println(b);
}

static void printHelp() {
    Serial.println(F("# ============================================="));
    Serial.println(F("# Raw Encoder Pin Diagnostic"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.print(F("#  Testing: PIN_A = ")); Serial.print(PIN_A);
    Serial.print(F(", PIN_B = ")); Serial.println(PIN_B);
    Serial.println(F("#  Rotate motor shaft slowly by hand."));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  S  Toggle live stream (every 50 ms)"));
    Serial.println(F("#  R  Single snapshot reading"));
    Serial.println(F("#  C  Toggle change-count mode"));
    Serial.println(F("#  ?  This help"));
    Serial.println(F("# ---------------------------------------------"));
    Serial.println(F("#  Expected: both pins alternate 0/1 with rotation"));
    Serial.println(F("#  Stuck at 0 or 1: channel dead or wiring fault"));
    Serial.println(F("# ============================================="));
}

// ============================================================
//  ENTRY POINTS
// ============================================================
void testEncoderRaw_setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(500);

    pinMode(PIN_A, INPUT);
    pinMode(PIN_B, INPUT);

    printHelp();
    Serial.println(F("# Press S to start stream, then rotate the shaft."));
}

void testEncoderRaw_loop() {
    // Live stream
    if (g_stream) {
        uint32_t now = millis();
        if (now - g_streamMs >= STREAM_INTERVAL_MS) {
            g_streamMs = now;
            printReading();
        }
    }

    // Change-count mode — detect any state change on either pin
    if (g_countMode) {
        int a = digitalRead(PIN_A);
        int b = digitalRead(PIN_B);
        if (g_lastA != -1 && a != g_lastA) {
            g_countA++;
            Serial.print(F("A changed → ")); Serial.print(a);
            Serial.print(F("  (total A transitions: ")); Serial.print(g_countA);
            Serial.println(F(")"));
        }
        if (g_lastB != -1 && b != g_lastB) {
            g_countB++;
            Serial.print(F("B changed → ")); Serial.print(b);
            Serial.print(F("  (total B transitions: ")); Serial.print(g_countB);
            Serial.println(F(")"));
        }
        g_lastA = a;
        g_lastB = b;
    }

    if (!Serial.available()) return;

    char c = static_cast<char>(Serial.read());
    switch (c) {
        case 's': case 'S':
            g_stream = !g_stream;
            g_streamMs = millis();
            Serial.println(g_stream ? F("# Stream ON") : F("# Stream OFF"));
            break;

        case 'r': case 'R':
            printReading();
            break;

        case 'c': case 'C':
            g_countMode = !g_countMode;
            g_lastA = -1; g_lastB = -1;
            g_countA = 0; g_countB = 0;
            Serial.println(g_countMode ? F("# Count mode ON — rotate shaft slowly")
                                       : F("# Count mode OFF"));
            break;

        case '?':
            printHelp();
            break;

        default:
            if (c >= 0x20 && c < 0x7F) {
                Serial.print(F("# unknown: '")); Serial.print(c); Serial.println(F("'"));
            }
            break;
    }
}

#endif // TEST_MODE
