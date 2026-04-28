#include "core/SerialComm.h"
#include "control/MotorController.h"
#include "devices/OpenLoopStepper.h"
#include "devices/ServoController.h"
#include "devices/Microswitch.h"
#include "devices/DCMotorDriver.h"
#include "core/ThreadManager.h"
#include "core/Task.h"
#include "operations/HomeOperation.h"
#include "operations/TransferOperation.h"
#include "operations/CalibrationOperation.h"
#include "devices/LaserSensor.h"
#include "devices/LineLaser.h"

// ============================================================
//  SerialComm.cpp
// ============================================================

void SerialComm::begin() {
    // Serial is already initialised and waited on in setup() before this is called.
    // Nothing to do here — kept as a hook for future per-module init if needed.
}

// ----------------------------------------------------------
//  receive() — called by the serial thread on every tick
//  where Serial.available() > 0.
//  Accumulates bytes into rxBuf_ until '\n', then dispatches.
// ----------------------------------------------------------
void SerialComm::receive() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (rxLen_ > 0) {
                if (!overflow_) {
                    rxBuf_[rxLen_] = '\0';
                    processLine(rxBuf_);
                } else {
                    // Line was too long — discard and report
                    sendError("message too long");
                }
                rxLen_    = 0;
                overflow_ = false;
            }
            continue;
        }

        if (rxLen_ >= SERIAL_RX_BUFFER_SIZE - 1) {
            overflow_ = true;
            continue;
        }

        rxBuf_[rxLen_++] = c;
    }
}

// ----------------------------------------------------------
//  processLine() — parse a complete null-terminated JSON line
//  and dispatch to the appropriate handler.
// ----------------------------------------------------------
void SerialComm::processLine(const char* line) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    DeserializationError err = deserializeJson(doc, line);

    if (err) {
        sendError("json parse error");
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
        sendError("missing cmd field");
        return;
    }

    // Update last-message timestamp (used by heartbeat thread)
    mutex_.lock();
    lastMessageMs_ = millis();
    mutex_.unlock();

    // Dispatch to handler
    if      (strcmp(cmd, "move")      == 0) handleMove(doc);
    else if (strcmp(cmd, "home")      == 0) handleHome();
    else if (strcmp(cmd, "transfer")  == 0) handleTransfer(doc);
    else if (strcmp(cmd, "calibrate") == 0) handleCalibrate();
    else if (strcmp(cmd, "servo")      == 0) handleServo(doc);
    else if (strcmp(cmd, "leadscrew") == 0) handleLeadScrew(doc);
    else if (strcmp(cmd, "dcmotor")   == 0) handleDCMotor(doc);
    else if (strcmp(cmd, "scan_frame")  == 0) handleScanFrame();
    else if (strcmp(cmd, "laser_on")    == 0) { LineLaser::instance().on();  sendOk(); }
    else if (strcmp(cmd, "laser_off")   == 0) { LineLaser::instance().off(); sendOk(); }
    else if (strcmp(cmd, "set_origin")  == 0) handleSetOrigin();
    else if (strcmp(cmd, "set_servo_home") == 0) handleSetServoHome();
    else if (strcmp(cmd, "ping_sts")    == 0) handlePingSts();
    else if (strcmp(cmd, "start")       == 0) handleStart();
    else if (strcmp(cmd, "stop")      == 0) handleStop();
    else if (strcmp(cmd, "estop")     == 0) handleEStop();
    else if (strcmp(cmd, "status")    == 0) handleStatus();
    else                                    sendError("unknown command");
}

// ----------------------------------------------------------
//  Command handlers
// ----------------------------------------------------------

void SerialComm::handleMove(JsonDocument& doc) {
    if (!doc["x"].is<float>() || !doc["y"].is<float>()) {
        sendError("move requires x and y fields");
        return;
    }
    if (!motorController_) {
        sendError("motor controller not registered");
        return;
    }

    float x = doc["x"].as<float>();
    float y = doc["y"].as<float>();

    // moveTo() is thread-safe and can be called from the serial thread directly.
    // It loads a new trajectory into TrajectoryPlanner; the control thread
    // at 1 kHz will execute it via MotorController::update().
    // Apply global origin offset so UI coordinates (0,0) = physical home
    motorController_->moveTo(x + originX_, y + originY_);
    MachineState::instance().setStatus(SystemStatus::RUNNING);
    sendOk();
}

void SerialComm::handleHome() {
    SystemStatus status = MachineState::instance().getStatus();
    if (status == SystemStatus::RUNNING || status == SystemStatus::HOMING) {
        sendError("cannot home while running");
        return;
    }
    if (!motorController_ || !zScrews_) {
        sendError("motor controller not registered");
        return;
    }

    if (!switchX_ || !switchY_ || !switchZ_) {
        sendError("microswitches not registered");
        return;
    }

    // Capture pointers for the lambda (safe: objects are file-scope in main.cpp)
    MotorController*  mc     = motorController_;
    OpenLoopStepper** screws = zScrews_;
    Microswitch*      swX    = switchX_;
    Microswitch*      swY    = switchY_;
    Microswitch*      swZ    = switchZ_;

    auto task = Task::make([mc, screws, swX, swY, swZ]() {
        HomeOperation op(mc, screws, swX, swY, swZ);
        MachineState::instance().setStatus(SystemStatus::HOMING);
        op.run();
    }, TaskPriority::LARGE, "home");

    ThreadManager::instance().queue().push(task);
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleTransfer(JsonDocument& doc) {
    SystemStatus status = MachineState::instance().getStatus();
    if (status == SystemStatus::RUNNING || status == SystemStatus::HOMING) {
        sendError("already running");
        return;
    }
    if (!MachineState::instance().getGantry().isHomed) {
        sendError("must home before transfer");
        return;
    }
    if (!motorController_ || !zScrews_ || !servoController_) {
        sendError("hardware not registered");
        return;
    }

    float   dstX  = doc["x"]     | 0.0f;
    float   dstY  = doc["y"]     | 0.0f;
    uint8_t count = doc["count"] | uint8_t(1);

    MotorController*  mc      = motorController_;
    OpenLoopStepper** screws  = zScrews_;
    ServoController*  servos  = servoController_;

    auto task = Task::make([mc, screws, servos, dstX, dstY, count]() {
        TransferOperation op(mc, screws, servos,
                             &LaserSensor::instance(), &LineLaser::instance(),
                             /*srcX=*/ 0.0f, /*srcY=*/ 0.0f,
                             dstX, dstY, count);
        op.run();
    }, TaskPriority::NORMAL, "transfer");

    ThreadManager::instance().queue().push(task);
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleCalibrate() {
    SystemStatus status = MachineState::instance().getStatus();
    if (status == SystemStatus::RUNNING || status == SystemStatus::HOMING) {
        sendError("already running");
        return;
    }
    if (!motorController_ || !zScrews_) {
        sendError("hardware not registered");
        return;
    }

    MotorController*  mc     = motorController_;
    OpenLoopStepper** screws = zScrews_;

    // Calibration data persists in a static so it survives the lambda lifetime.
    static CalibData calibData;

    auto task = Task::make([mc, screws]() {
        CalibrationOperation op(mc, screws,
                                &LaserSensor::instance(), &LineLaser::instance(),
                                &calibData,
                                /*laser=*/ true,
                                /*xy=*/    true,
                                /*z=*/     false);
        op.run();
    }, TaskPriority::NORMAL, "calibrate");

    ThreadManager::instance().queue().push(task);
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleServo(JsonDocument& doc) {
    if (!servoController_) {
        sendError("servo controller not registered");
        return;
    }

    uint8_t id    = doc["id"]    | uint8_t(0);
    float   angle = doc["angle"] | 0.0f;
    uint16_t speed = doc["speed"] | uint16_t(0);

    if (id == 4) {
        // STS3215 — all UART I/O runs on the worker thread so the serial
        // thread is never blocked waiting on STS timeouts.
        ServoController* sc      = servoController_;
        float            target  = angle + stsHomeAngle_;
        lastStsCommandedAngle_   = angle;

        auto task = Task::make([sc, target, speed]() {
            sc->sts3215EnableTorque(true);
            delay(10);   // Give servo time to enable torque before accepting a position command
            sc->sts3215SetPosition(target, speed);
        }, TaskPriority::NORMAL, "sts_move");

        ThreadManager::instance().queue().push(task);
    } else if (id < 4) {
        // MG996R PWM servo — instant, no blocking
        servoController_->mg996rSetAngle(id, angle);
    } else {
        sendError("invalid servo id (0-3 = MG996R, 4 = STS3215)");
        return;
    }
    sendOk();
}

// ----------------------------------------------------------
// ----------------------------------------------------------
void SerialComm::handleLeadScrew(JsonDocument& doc) {
    if (!zScrews_) {
        sendError("lead screws not registered");
        return;
    }

    uint8_t id      = doc["id"]      | uint8_t(0);
    float   distMM  = doc["dist_mm"] | 15.0f;
    bool    up      = doc["up"]      | true;
    float   speedMS = doc["speed"]   | 5.0f;   // mm/s

    if (id > 3) { sendError("leadscrew id must be 0-3"); return; }
    if (distMM <= 0.0f || distMM > 200.0f) { sendError("dist_mm out of range"); return; }

    OpenLoopStepper* screw = zScrews_[id];

    // Capture by value so the lambda is safe after this function returns
    auto task = Task::make([screw, distMM, up, speedMS]() {
        screw->setDirection(!up);   // Physical wiring is inverted; flip to match "up" intent

        float    stepsPerMM  = Constants::LeadScrews::STEPS_PER_MM;
        int32_t  totalSteps  = static_cast<int32_t>(distMM * stepsPerMM);
        uint32_t stepDelayUs = (speedMS > 0.0f)
            ? static_cast<uint32_t>(1000000.0f / (speedMS * stepsPerMM))
            : 500;

        for (int32_t i = 0; i < totalSteps; i++) {
            screw->step();
            delayMicroseconds(stepDelayUs);
        }
        screw->syncState(false);
    }, TaskPriority::NORMAL, "leadscrew");

    ThreadManager::instance().queue().push(task);
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleDCMotor(JsonDocument& doc) {
    uint8_t id        = doc["id"]    | uint8_t(0);
    float   speedPct  = doc["speed"] | 0.0f;
    const char* mode  = doc["mode"]  | "stop";

    if (id > 1) { sendError("dcmotor id must be 0 or 1"); return; }

    DCMotorIndex motor = (id == 0) ? DCMotorIndex::A : DCMotorIndex::B;
    auto& driver = DCMotorDriver::instance();

    if (strcmp(mode, "fwd")   == 0) { driver.set(motor,  speedPct); }
    else if (strcmp(mode, "rev")   == 0) { driver.set(motor, -speedPct); }
    else if (strcmp(mode, "brake") == 0) { driver.brake(motor); }
    else                                 { driver.coast(motor); }  // "stop" / "coast"

    driver.syncState();
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleScanFrame() {
    LineLaser::instance().on();

    // TSL1401 pipeline: capture() reads pixels from the PREVIOUS integration
    // period, while simultaneously starting a NEW one.  So we need two cycles:
    //   1st capture — reads stale (dark) data; starts fresh integration with laser ON
    //   wait        — sensor accumulates charge under laser illumination
    //   2nd capture — reads the laser-illuminated frame
    LaserSensor::instance().capture();   // dummy: flushes dark frame, arms integration
    delayMicroseconds(
        static_cast<uint32_t>(Constants::LaserSensor::INTEGRATION_TIME_US));
    LaserSensor::instance().capture();   // real: reads laser-illuminated pixels
    LaserSensor::instance().findCenter();
    LaserSensor::instance().syncState();
    LineLaser::instance().off();

    float currentY = MachineState::instance().getGantry().y;
    Serial.print(F("PX:"));
    Serial.print(currentY, 3);
    const uint16_t* px = LaserSensor::instance().getPixels();
    for (uint8_t i = 0; i < 128; i++) {
        Serial.print(',');
        Serial.print(px[i]);
    }
    Serial.println();
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handlePingSts() {
    if (!servoController_) {
        sendError("servo controller not registered");
        return;
    }
    ServoController* sc = servoController_;
    auto task = Task::make([sc]() {
        bool ok = sc->sts3215Ping();
        if (ok) {
            Serial.println(F("{\"status\":\"ok\",\"msg\":\"pong\"}"));
        } else {
            Serial.println(F("{\"status\":\"err\",\"msg\":\"STS3215 no response\"}"));
        }
    }, TaskPriority::NORMAL, "sts_ping");
    ThreadManager::instance().queue().push(task);
    // Respond immediately — actual ping result arrives asynchronously via serial
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleSetOrigin() {
    // Record current gantry position as the new (0,0) for all subsequent move commands.
    // All user-space coordinates received from the UI are added to this offset before
    // being passed to MotorController::moveTo().
    GantryState g = MachineState::instance().getGantry();
    originX_ = g.x;
    originY_ = g.y;
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleSetServoHome() {
    // Record the last commanded STS angle as the physical zero.
    // Future handleServo(id=4) calls will add stsHomeAngle_ before commanding the servo,
    // so user angle 0 always maps to the physical position that was set as home.
    stsHomeAngle_ = stsHomeAngle_ + lastStsCommandedAngle_;
    lastStsCommandedAngle_ = 0.0f;
    sendOk();
}

// ----------------------------------------------------------
void SerialComm::handleStart() {
    SystemStatus status = MachineState::instance().getStatus();

    if (status == SystemStatus::E_STOP) {
        sendError("clear estop before starting");
        return;
    }
    if (status == SystemStatus::RUNNING) {
        sendError("already running");
        return;
    }
    if (status != SystemStatus::IDLE && status != SystemStatus::PAUSED) {
        sendError("system not ready to start");
        return;
    }

    MachineState::instance().setStatus(SystemStatus::RUNNING);

    // TODO: Push a TransferOperation task onto ThreadManager::instance().queue()
    //       once TransferOperation is implemented.

    sendOk();
}

void SerialComm::handleStop() {
    SystemStatus status = MachineState::instance().getStatus();

    if (status != SystemStatus::RUNNING) {
        sendError("not currently running");
        return;
    }

    MachineState::instance().setStatus(SystemStatus::PAUSED);

    // TODO: Signal the active Operation to pause gracefully
    //       once Operation base class is implemented.

    sendOk();
}

void SerialComm::handleEStop() {
    MachineState::instance().eStop();
    sendOk();
}

void SerialComm::handleStatus() {
    sendOkWithState();
}

// ----------------------------------------------------------
//  Response helpers
// ----------------------------------------------------------

void SerialComm::sendOk() {
    Serial.println(F("{\"status\":\"ok\"}"));
}

void SerialComm::sendOkWithState() {
    // Use a larger document — we're now sending screws[4] and sw[3] too.
    StaticJsonDocument<512> doc;
    GantryState  g = MachineState::instance().getGantry();
    SystemStatus s = MachineState::instance().getStatus();
    SensorState  sens = MachineState::instance().getSensors();

    // "status" carries the system status string so bridge_node.py can read
    // data.get('status', 'IDLE') directly without looking at a separate field.
    switch (s) {
        case SystemStatus::IDLE:    doc["status"] = "IDLE";    break;
        case SystemStatus::HOMING:  doc["status"] = "HOMING";  break;
        case SystemStatus::RUNNING: doc["status"] = "RUNNING"; break;
        case SystemStatus::PAUSED:  doc["status"] = "PAUSED";  break;
        case SystemStatus::FAULT:   doc["status"] = "FAULT";   break;
        case SystemStatus::E_STOP:  doc["status"] = "E_STOP";  break;
        default:                    doc["status"] = "IDLE";    break;
    }

    doc["x"]      = g.x;
    doc["y"]      = g.y;
    doc["vx"]     = g.xVelocity;
    doc["vy"]     = g.yVelocity;
    doc["moving"] = g.isMoving;
    doc["homed"]  = g.isHomed;

    // Lead screw positions
    JsonArray screws = doc.createNestedArray("screws");
    for (uint8_t i = 0; i < 4; i++) {
        screws.add(MachineState::instance().getLeadScrew(i).positionMM);
    }

    // Limit switches [X, Y, Z]
    JsonArray sw = doc.createNestedArray("sw");
    sw.add(sens.switchX.tripped);
    sw.add(sens.switchY.tripped);
    sw.add(sens.switchZ.tripped);

    serializeJson(doc, Serial);
    Serial.println();   // Newline-terminate for the PC's readline
}

void SerialComm::sendError(const char* msg) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    doc["status"] = "err";
    doc["msg"]    = msg;
    serializeJson(doc, Serial);
    Serial.println();
}

// ----------------------------------------------------------
uint32_t SerialComm::lastMessageMs() const {
    mutex_.lock();
    uint32_t t = lastMessageMs_;
    mutex_.unlock();
    return t;
}