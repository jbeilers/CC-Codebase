#include "core/TaskQueue.h"

// ============================================================
//  TaskQueue.cpp
// ============================================================

// ----------------------------------------------------------
void TaskQueue::push(std::shared_ptr<Task> task) {
    if (!task) return;
    mutex_.lock();
    queue_.push(std::move(task));
    mutex_.unlock();
}

// ----------------------------------------------------------
std::shared_ptr<Task> TaskQueue::pop() {
    mutex_.lock();

    // Drain any completed/cancelled tasks sitting at the top
    while (!queue_.empty()) {
        auto top = queue_.top();
        if (top->isDone() || top->isRunning()) {
            queue_.pop();   // Reap finished tasks
            continue;
        }
        if (top->isPending()) {
            queue_.pop();
            mutex_.unlock();
            return top;
        }
        queue_.pop();       // Catch-all for unexpected states
    }

    mutex_.unlock();
    return nullptr;
}

// ----------------------------------------------------------
std::shared_ptr<Task> TaskQueue::peek() const {
    mutex_.lock();
    if (queue_.empty()) {
        mutex_.unlock();
        return nullptr;
    }
    auto top = queue_.top();
    mutex_.unlock();
    return top;
}

// ----------------------------------------------------------
void TaskQueue::cancelAll() {
    mutex_.lock();

    // std::priority_queue has no iteration — rebuild after draining
    std::vector<std::shared_ptr<Task>> remaining;
    while (!queue_.empty()) {
        auto task = queue_.top();
        queue_.pop();
        task->cancel();     // No-op if already running/done
        remaining.push_back(task);
    }

    // Re-insert running tasks so they can finish normally
    // (running tasks ignore cancel() — they complete on their own)
    for (auto& t : remaining) {
        if (t->isRunning()) {
            queue_.push(t);
        }
        // Pending/done/cancelled tasks are dropped
    }

    mutex_.unlock();
}

// ----------------------------------------------------------
size_t TaskQueue::size() const {
    mutex_.lock();
    size_t s = queue_.size();
    mutex_.unlock();
    return s;
}

// ----------------------------------------------------------
bool TaskQueue::empty() const {
    mutex_.lock();
    bool e = queue_.empty();
    mutex_.unlock();
    return e;
}