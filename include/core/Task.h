#pragma once

#include <cstdint>
#include <atomic>
#include <memory>

// ============================================================
//  Task.h  —  Base unit of work for the task queue system.
//
//  A Task wraps a callable with a priority level and a
//  completion flag. The thread pool picks the highest-priority
//  available Task from the queue.
//
//  WorkFn is a plain void(void*) function pointer + a context
//  pointer, rather than std::function, because std::function
//  requires C++ exceptions (-fno-exceptions breaks it on ARM).
//
//  For lambda capture, wrap state in a heap-allocated struct
//  and pass it as ctx.  The Task takes ownership and deletes
//  it via the deleter_ pointer after execute() returns.
//
//  Convenience helper:  Task::make(lambda, priority, name)
//  handles the heap allocation automatically for lambdas.
//
//  Usage:
//    auto t = Task::make([&]() { motorController.moveTo(x,y); },
//                        TaskPriority::NORMAL, "move");
//    queue.push(t);
// ============================================================

enum class TaskPriority : uint8_t {
    SMALL      = 0,
    NORMAL   = 1,
    LARGE     = 2,
    CRITICAL = 3
};

enum class TaskStatus : uint8_t {
    PENDING,
    RUNNING,
    COMPLETE,
    FAILED,
    CANCELLED
};

struct Task {
    // Plain function pointer — takes an opaque context pointer.
    using WorkFn   = void (*)(void*);
    using DeleteFn = void (*)(void*);

    // ----------------------------------------------------------
    //  Low-level constructor — use Task::make() for lambdas.
    // ----------------------------------------------------------
    Task(WorkFn     work,
         void*      ctx,
         DeleteFn   deleter,
         TaskPriority  priority = TaskPriority::NORMAL,
         const char*   name     = "unnamed")
        : work_(work)
        , ctx_(ctx)
        , deleter_(deleter)
        , priority(priority)
        , name(name)
        , status(TaskStatus::PENDING)
    {}

    ~Task() {
        if (deleter_ && ctx_) deleter_(ctx_);
    }

    Task(const Task&)            = delete;
    Task& operator=(const Task&) = delete;

    // ----------------------------------------------------------
    //  make() — convenience factory for capturing lambdas.
    //  Allocates a copy of the lambda on the heap; the Task
    //  destructor deletes it via deleteHelper<Lambda>.
    //
    //  Uses static member function templates for invoke/delete
    //  instead of inline lambdas — arm-none-eabi-gcc 11 does not
    //  reliably convert captureless lambdas inside templates to
    //  plain function pointers.
    //
    //  Example:
    //    auto t = Task::make([x, y]() { mc.moveTo(x, y); },
    //                        TaskPriority::HIGH, "move");
    // ----------------------------------------------------------
    template<typename Lambda>
    static std::shared_ptr<Task> make(Lambda       fn,
                                      TaskPriority priority = TaskPriority::NORMAL,
                                      const char*  name     = "unnamed")
    {
        Lambda* heap = new Lambda(static_cast<Lambda&&>(fn));
        return std::make_shared<Task>(
            &invokeHelper<Lambda>,
            static_cast<void*>(heap),
            &deleteHelper<Lambda>,
            priority,
            name
        );
    }

    // ----------------------------------------------------------
    //  Execute — called by the worker thread.
    // ----------------------------------------------------------
    void execute() {
        status.store(TaskStatus::RUNNING);
        if (work_) work_(ctx_);
        status.store(TaskStatus::COMPLETE);
    }

    void cancel() {
        TaskStatus expected = TaskStatus::PENDING;
        status.compare_exchange_strong(expected, TaskStatus::CANCELLED);
    }

    bool isPending()   const { return status.load() == TaskStatus::PENDING;   }
    bool isRunning()   const { return status.load() == TaskStatus::RUNNING;   }
    bool isComplete()  const { return status.load() == TaskStatus::COMPLETE;  }
    bool isFailed()    const { return status.load() == TaskStatus::FAILED;    }
    bool isCancelled() const { return status.load() == TaskStatus::CANCELLED; }
    bool isDone() const {
        auto s = status.load();
        return s == TaskStatus::COMPLETE
            || s == TaskStatus::FAILED
            || s == TaskStatus::CANCELLED;
    }

    bool operator<(const Task& other) const {
        return static_cast<uint8_t>(priority) < static_cast<uint8_t>(other.priority);
    }

    const TaskPriority      priority;
    const char*             name;
    std::atomic<TaskStatus> status;

private:
    WorkFn   work_;
    void*    ctx_;
    DeleteFn deleter_;

    // Static helpers — used by make() as unambiguous plain function pointers.
    // Defined as member function templates rather than inline lambdas because
    // arm-none-eabi-gcc 11 does not reliably convert captureless lambdas
    // inside template functions to function pointers.
    template<typename Lambda>
    static void invokeHelper(void* ctx) {
        (*static_cast<Lambda*>(ctx))();
    }

    template<typename Lambda>
    static void deleteHelper(void* ctx) {
        delete static_cast<Lambda*>(ctx);
    }
};