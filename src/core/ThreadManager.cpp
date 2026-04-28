#include "core/ThreadManager.h"
#include "core/SerialComm.h"

// ============================================================
//  ThreadManager.cpp
// ============================================================

// ----------------------------------------------------------
//  begin() — spawns all threads. Call once in setup().
// ----------------------------------------------------------
void ThreadManager::begin() {
    // Set tick resolution before spawning threads
    // 10µs tick gives sufficient resolution for the 1kHz control loop
    threads.setMicroTimer(10);

    // Control thread — give it a short time slice so it runs frequently.
    // TeensyThreads has no priority API; we compensate by giving the
    // control thread a smaller time slice than workers so it preempts sooner.
    controlThreadId_ = threads.addThread(controlThreadEntry, 0, STACK_CONTROL);
    threads.setTimeSlice(controlThreadId_, 1);  // 1 tick = 10µs per slice

    // Serial listener thread
    serialThreadId_ = threads.addThread(serialThreadEntry, 0, STACK_SERIAL);

    // Heartbeat thread
    heartbeatThreadId_ = threads.addThread(heartbeatThreadEntry, 0, STACK_HEARTBEAT);

    // Worker pool
    for (uint8_t i = 0; i < WORKER_POOL_SIZE; i++) {
        workerThreadIds_[i] = threads.addThread(workerThreadEntry, 0, STACK_WORKER);
    }
}

// ----------------------------------------------------------
//  allThreadsAlive() — checks that all thread IDs are still
//  in a running or sleeping state.
// ----------------------------------------------------------
bool ThreadManager::allThreadsAlive() const {
    auto isAlive = [](int id) -> bool {
        if (id < 0) return false;
        int state = threads.getState(id);
        return (state == Threads::RUNNING || state == Threads::SUSPENDED);
    };

    if (!isAlive(controlThreadId_))   return false;
    if (!isAlive(serialThreadId_))    return false;
    if (!isAlive(heartbeatThreadId_)) return false;

    for (uint8_t i = 0; i < WORKER_POOL_SIZE; i++) {
        if (!isAlive(workerThreadIds_[i])) return false;
    }

    return true;
}

// ----------------------------------------------------------
//  Control thread — runs the registered control callback at
//  a fixed rate defined by Constants::Timing::CONTROL_LOOP_HZ.
//  Bails immediately if E_STOP is active.
// ----------------------------------------------------------
void ThreadManager::controlThreadEntry() {
    ThreadManager& mgr = ThreadManager::instance();

    const uint32_t periodUs = 1000000UL / Constants::Timing::CONTROL_LOOP_HZ;

    while (true) {
        uint32_t startUs = micros();

        // Skip execution during E_STOP — motors should already be halted
        if (MachineState::instance().getStatus() != SystemStatus::E_STOP) {
            if (mgr.controlCallback_) {
                mgr.controlCallback_();
            }
        }

        // Fixed-rate sleep — yield remainder of period
        uint32_t elapsedUs = micros() - startUs;
        if (elapsedUs < periodUs) {
            threads.delay_us(periodUs - elapsedUs);
        }
        // TODO: If elapsed > periodUs, log a control loop overrun.
        //       Consistent overruns mean CONTROL_LOOP_HZ is too high
        //       or controlCallback_ is doing too much work per tick.
    }
}

// ----------------------------------------------------------
//  Serial thread — continuously polls USB serial for incoming
//  messages from the HP PC and invokes the registered callback.
//  Sleeps briefly when no data is available to yield CPU time.
// ----------------------------------------------------------
void ThreadManager::serialThreadEntry() {
    ThreadManager& mgr = ThreadManager::instance();

    while (true) {
        if (Serial.available() > 0) {
            if (mgr.serialCallback_) {
                mgr.serialCallback_();
            }
        } else {
            threads.delay(1);   // 1ms yield when idle
        }
    }
}

// ----------------------------------------------------------
//  Heartbeat thread — checks HP PC connection liveness by
//  monitoring the last received message timestamp.
//  If no message received within SERIAL_TIMEOUT_MS, sets FAULT.
// ----------------------------------------------------------
void ThreadManager::heartbeatThreadEntry() {
    // Timestamp of the last received serial message.
    // SerialComm must update this via MachineState or a shared
    // atomic when a message arrives.
    // TODO: Replace this stub with a real last-message timestamp
    //       once SerialComm is implemented. For now the thread
    //       exists and runs but does not yet trigger faults.

    while (true) {
        threads.delay(Constants::Timing::HEARTBEAT_INTERVAL_MS);

        SystemStatus status = MachineState::instance().getStatus();

        // Don't overwrite an existing E_STOP or FAULT
        if (status == SystemStatus::E_STOP ||
            status == SystemStatus::FAULT) {
            continue;
        }

        uint32_t lastMsg = SerialComm::instance().lastMessageMs();
        if (lastMsg > 0 &&
            (millis() - lastMsg) > Constants::Timing::SERIAL_TIMEOUT_MS) {
            MachineState::instance().setStatus(SystemStatus::FAULT);
        }
        // Note: lastMsg == 0 means no message has ever been received
        // (e.g. system just booted). We don't fault until the first
        // message has been seen, to allow connection setup time.
    }
}

// ----------------------------------------------------------
//  Worker thread — each worker loops forever, popping and
//  executing the next available Task from the shared queue.
//  Sleeps briefly when the queue is empty to yield CPU time.
// ----------------------------------------------------------
void ThreadManager::workerThreadEntry() {
    TaskQueue& queue = ThreadManager::instance().queue_;

    while (true) {
        std::shared_ptr<Task> task = queue.pop();

        if (task) {
            task->execute();
        } else {
            threads.delay(5);   // 5ms yield when queue is empty // TODO: Tune idle sleep duration — shorter = more responsive, longer = less CPU waste
        }
    }
}