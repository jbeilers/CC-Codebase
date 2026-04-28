#pragma once

#include <TeensyThreads.h>
#include <cstdint>
#include "core/TaskQueue.h"
#include "core/Task.h"
#include "state/MachineState.h"
#include "math/Constants.h"

// ============================================================
//  ThreadManager.h  —  Central owner of all threads and the
//                      shared TaskQueue.
//
//  Threads managed:
//    - Control thread   : Runs Kalman → LQR → FF → dispatch
//                         at a fixed rate (Constants::Timing::CONTROL_LOOP_HZ).
//                         Highest priority. Implemented externally
//                         in MotorController — registered here.
//    - Serial thread    : Continuously listens for incoming
//                         serial messages and pushes Tasks.
//    - Heartbeat thread : Periodically checks HP PC connection
//                         liveness. Triggers FAULT if lost.
//    - Worker pool      : N worker threads that each pop and
//                         execute the next available Task.
//
//  Usage (in main.cpp):
//    ThreadManager::instance().begin();
// ============================================================

// TODO: Tune worker pool size based on observed task concurrency.
//       2 workers is a safe starting point for this system.
constexpr uint8_t WORKER_POOL_SIZE = 2;

// TODO: Tune stack sizes if stack overflows are observed.
//       Teensy 4.1 has 1MB RAM — these are conservative defaults.
constexpr int STACK_CONTROL   = 4096;   // Bytes — control loop stack
constexpr int STACK_SERIAL    = 2048;   // Bytes — serial listener stack
constexpr int STACK_HEARTBEAT = 1024;   // Bytes — heartbeat stack
constexpr int STACK_WORKER    = 2048;   // Bytes — each worker thread stack

class ThreadManager {
public:
    static ThreadManager& instance() {
        static ThreadManager inst;
        return inst;
    }

    // Disallow copy
    ThreadManager(const ThreadManager&)            = delete;
    ThreadManager& operator=(const ThreadManager&) = delete;

    // ----------------------------------------------------------
    //  Register the control loop callback before calling begin().
    //  The callback will be called at CONTROL_LOOP_HZ on the
    //  highest-priority thread.
    // ----------------------------------------------------------
    void setControlCallback(void (*fn)()) { controlCallback_ = fn; }

    // ----------------------------------------------------------
    //  Register the serial receive callback before calling begin().
    //  Called on every received message from the HP PC.
    // ----------------------------------------------------------
    void setSerialCallback(void (*fn)()) { serialCallback_ = fn; }

    // ----------------------------------------------------------
    //  Start all threads. Call once in setup() after all
    //  callbacks have been registered.
    //  Note: TeensyThreads has no priority API. The control thread
    //  is given a shorter time slice (1 tick) to approximate higher
    //  scheduling frequency relative to worker threads.
    // ----------------------------------------------------------
    void begin();

    // ----------------------------------------------------------
    //  Access the shared task queue.
    //  Device drivers and Operations push tasks here.
    // ----------------------------------------------------------
    TaskQueue& queue() { return queue_; }

    // ----------------------------------------------------------
    //  Returns true if all threads are alive and running.
    //  Called by the heartbeat thread to self-monitor.
    // ----------------------------------------------------------
    bool allThreadsAlive() const;

private:
    ThreadManager() = default;

    // ----------------------------------------------------------
    //  Thread entry points — static wrappers required by
    //  TeensyThreads since it doesn't accept member fn pointers.
    // ----------------------------------------------------------
    static void controlThreadEntry();
    static void serialThreadEntry();
    static void heartbeatThreadEntry();
    static void workerThreadEntry();

    // ----------------------------------------------------------
    //  Thread IDs — stored for liveness checks
    // ----------------------------------------------------------
    int controlThreadId_           = -1;
    int serialThreadId_            = -1;
    int heartbeatThreadId_         = -1;
    int workerThreadIds_[WORKER_POOL_SIZE] = {};

    // ----------------------------------------------------------
    //  Registered callbacks
    // ----------------------------------------------------------
    void (*controlCallback_)() = nullptr;
    void (*serialCallback_)()  = nullptr;

    // ----------------------------------------------------------
    //  Shared task queue — all workers pull from this
    // ----------------------------------------------------------
    TaskQueue queue_;
};