#pragma once

#include <memory>
#include <queue>
#include <vector>
#include <TeensyThreads.h>
#include "Task.h"

// ============================================================
//  TaskQueue.h  —  Thread-safe priority queue of Tasks.
//
//  The highest-priority pending Task is always at the front.
//  The ThreadManager owns one TaskQueue and worker threads
//  call pop() to claim the next available Task.
// ============================================================

class TaskQueue {
public:
    TaskQueue()  = default;
    ~TaskQueue() = default;

    // Disallow copy
    TaskQueue(const TaskQueue&)            = delete;
    TaskQueue& operator=(const TaskQueue&) = delete;

    // ----------------------------------------------------------
    //  Push a new Task onto the queue.
    //  Thread-safe. Can be called from any thread.
    // ----------------------------------------------------------
    void push(std::shared_ptr<Task> task);

    // ----------------------------------------------------------
    //  Pop the highest-priority PENDING Task.
    //  Returns nullptr if the queue is empty or all tasks are
    //  non-pending (running, cancelled, etc.).
    //  Thread-safe.
    // ----------------------------------------------------------
    std::shared_ptr<Task> pop();

    // ----------------------------------------------------------
    //  Peek at the highest-priority task without removing it.
    //  Returns nullptr if queue is empty.
    //  Thread-safe.
    // ----------------------------------------------------------
    std::shared_ptr<Task> peek() const;

    // ----------------------------------------------------------
    //  Cancel all PENDING tasks in the queue.
    //  Does not affect currently RUNNING tasks.
    //  Thread-safe.
    // ----------------------------------------------------------
    void cancelAll();

    // ----------------------------------------------------------
    //  Returns the number of tasks currently in the queue
    //  (includes pending, running, and done tasks not yet reaped).
    //  Thread-safe.
    // ----------------------------------------------------------
    size_t size() const;

    // ----------------------------------------------------------
    //  Returns true if the queue contains no tasks at all.
    //  Thread-safe.
    // ----------------------------------------------------------
    bool empty() const;

private:
    // ----------------------------------------------------------
    //  Comparator: higher TaskPriority value = higher priority
    // ----------------------------------------------------------
    struct TaskComparator {
        bool operator()(const std::shared_ptr<Task>& a,
                        const std::shared_ptr<Task>& b) const {
            return static_cast<uint8_t>(a->priority)
                 < static_cast<uint8_t>(b->priority);
        }
    };

    using PriorityQueue = std::priority_queue<
        std::shared_ptr<Task>,
        std::vector<std::shared_ptr<Task>>,
        TaskComparator
    >;

    mutable PriorityQueue queue_;

    mutable Threads::Mutex mutex_;
};