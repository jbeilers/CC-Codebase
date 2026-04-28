#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <TeensyThreads.h>
#include <cstdint>
#include "core/TaskQueue.h"
#include "state/MachineState.h"
#include "math/Constants.h"

// Forward declarations — avoids circular includes with MotorController
class MotorController;
class OpenLoopStepper;
class ServoController;
class Microswitch;

// ============================================================
//  SerialComm.h  —  JSON serial communication layer.
//
//  Handles all USB serial I/O between the Teensy and HP PC.
//
//  Incoming message format (PC → Teensy):
//    One JSON object per line, newline-terminated.
//    Every message must have a "cmd" field.
//
//    Known commands:
//      {"cmd":"move",   "x":100.0, "y":50.0}
//      {"cmd":"home"}
//      {"cmd":"start"}
//      {"cmd":"stop"}
//      {"cmd":"estop"}
//      {"cmd":"status"}
//
//  Outgoing message format (Teensy → PC):
//    JSON response to each command, newline-terminated.
//    Every response has a "status" field: "ok" or "err",
//    and an optional "msg" field for detail.
//
//    Example responses:
//      {"status":"ok"}
//      {"status":"ok","x":100.0,"y":50.0,"sys":"RUNNING"}
//      {"status":"err","msg":"unknown command"}
//
//  Threading:
//    SerialComm is driven by the serial thread in ThreadManager.
//    receive() is called by that thread whenever Serial.available() > 0.
//    All writes to MachineState and TaskQueue are thread-safe.
// ============================================================

// Maximum size of one incoming JSON message in bytes
// TODO: Increase if commands grow to carry larger payloads
constexpr size_t SERIAL_RX_BUFFER_SIZE = 256;

// ArduinoJson document size for incoming and outgoing messages
// TODO: Increase if JSON payloads grow (add fields, nested objects, etc.)
constexpr size_t JSON_DOC_SIZE = 256;

class SerialComm {
public:
    static SerialComm& instance() {
        static SerialComm inst;
        return inst;
    }

    // Disallow copy
    SerialComm(const SerialComm&)            = delete;
    SerialComm& operator=(const SerialComm&) = delete;

    // ----------------------------------------------------------
    //  Register hardware references before calling begin().
    //  These are used by handleMove(), handleHome(), handleTransfer(),
    //  handleCalibrate(), and handleServo().
    //  Call from setup() after all drivers are initialised.
    // ----------------------------------------------------------
    void setMotorController(MotorController* mc) { motorController_ = mc; }
    void setLeadScrews(OpenLoopStepper** screws) { zScrews_ = screws; }
    void setServoController(ServoController* sc) { servoController_ = sc; }
    void setMicroswitches(Microswitch* x, Microswitch* y, Microswitch* z) {
        switchX_ = x; switchY_ = y; switchZ_ = z;
    }

    // ----------------------------------------------------------
    //  Call once in setup() to initialise the serial port.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Called by the serial thread whenever Serial.available() > 0.
    //  Reads bytes into the line buffer, and processes a complete
    //  message when a newline is received.
    // ----------------------------------------------------------
    void receive();

    // ----------------------------------------------------------
    //  Returns the millis() timestamp of the last successfully
    //  received message. Used by the heartbeat thread to detect
    //  a lost connection.
    // ----------------------------------------------------------
    uint32_t lastMessageMs() const;

private:
    SerialComm() = default;

    // ----------------------------------------------------------
    //  Parse and dispatch a complete JSON line.
    //  Called internally when a '\n' is received.
    // ----------------------------------------------------------
    void processLine(const char* line);

    // ----------------------------------------------------------
    //  Command handlers — each maps to one known "cmd" value.
    //  Each receives the full parsed JsonDocument for field access.
    // ----------------------------------------------------------
    void handleMove(JsonDocument& doc);
    void handleHome();
    void handleTransfer(JsonDocument& doc);   // {"cmd":"transfer","x":...,"y":...,"count":...}
    void handleCalibrate();                   // {"cmd":"calibrate"}
    void handleServo(JsonDocument& doc);      // {"cmd":"servo","id":...,"angle":...,"speed":...}
    void handleLeadScrew(JsonDocument& doc);  // {"cmd":"leadscrew","id":0-3,"dist_mm":...,"up":bool,"speed":...}
    void handleDCMotor(JsonDocument& doc);    // {"cmd":"dcmotor","id":0-1,"speed":0.0-1.0,"mode":"fwd|rev|stop|coast|brake"}
    void handleScanFrame();                   // {"cmd":"scan_frame"} — one capture + PX: line
    void handleSetOrigin();                   // {"cmd":"set_origin"} — reset gantry (0,0) to current position
    void handleSetServoHome();                // {"cmd":"set_servo_home"} — reset STS3215 zero to current angle
    void handlePingSts();                     // {"cmd":"ping_sts"} — ping STS3215, reply ok/err
    void handleStart();
    void handleStop();
    void handleEStop();
    void handleStatus();

    // ----------------------------------------------------------
    //  Response helpers — send a JSON reply over serial.
    // ----------------------------------------------------------
    void sendOk();
    void sendOkWithState();
    void sendError(const char* msg);

    // ----------------------------------------------------------
    //  Internal state
    // ----------------------------------------------------------
    char     rxBuf_[SERIAL_RX_BUFFER_SIZE];   // Accumulates bytes until '\n'
    uint16_t rxLen_       = 0;                // Current write position in rxBuf_
    bool     overflow_    = false;            // True if current line exceeded buffer

    mutable Threads::Mutex mutex_;
    volatile uint32_t lastMessageMs_ = 0;     // Timestamp of last good message

    // Hardware references — set via setters before begin()
    MotorController*  motorController_  = nullptr;
    OpenLoopStepper** zScrews_          = nullptr;
    ServoController*  servoController_  = nullptr;
    Microswitch*      switchX_          = nullptr;
    Microswitch*      switchY_          = nullptr;
    Microswitch*      switchZ_          = nullptr;

    // Origin offsets — set by handleSetOrigin(); applied in handleMove()
    float originX_ = 0.0f;
    float originY_ = 0.0f;

    // STS3215 origin — set by handleSetServoHome(); applied in handleServo() for id=4
    float stsHomeAngle_        = 0.0f;   // physical angle that maps to user angle 0
    float lastStsCommandedAngle_ = 0.0f; // last user-space angle sent to STS
};