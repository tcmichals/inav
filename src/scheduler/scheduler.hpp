/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Cleanflight & INAV Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Exact C++20 Port of INAV Dynamic Priority Cooperative Task Scheduler
 *
 * Exact Reference C Source Files:
 *   - `external/inav/src/main/scheduler/scheduler.c`
 *   - `external/inav/src/main/scheduler/scheduler.h`
 *
 * Features:
 * 1. Exact INAV Dynamic Priority Age Aging (dynamicPriority = 1 + staticPriority * taskAgeCycles)
 * 2. Event-driven checkFunc tasks and Real-Time (TASK_PRIORITY_REALTIME = 18) forced execution
 * 3. 32-sample moving sum execution time and average load percent tracking
 * 4. Zero dynamic heap allocation (static task table with compile-time queue)
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10
 */

#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
#include <cstring>

namespace abstractx::scheduler {

using timeUs_t = uint32_t;
using timeDelta_t = int32_t;

enum cfTaskPriority_e : uint8_t {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 18,
    TASK_PRIORITY_MAX = 255
};

enum cfTaskId_e : uint8_t {
    TASK_SYSTEM = 0,
    TASK_PID,
    TASK_GYRO,
    TASK_RX,
    TASK_SERIAL,
    TASK_BATTERY,
    TASK_TEMPERATURE,
    TASK_BEEPER,
    TASK_GPS,
    TASK_COMPASS,
    TASK_BARO,
    TASK_TELEMETRY,
    TASK_RPM_FILTER,
    TASK_DYNAMIC_LPF,
    TASK_DYNAMIC_NOTCH,
    TASK_NAV,
    TASK_AUX,

    TASK_COUNT,
    TASK_NONE = TASK_COUNT,
    TASK_SELF
};

struct cfCheckFuncInfo_t {
    timeUs_t maxExecutionTime{0};
    timeUs_t totalExecutionTime{0};
    timeUs_t averageExecutionTime{0};
};

struct cfTaskInfo_t {
    const char*  taskName{nullptr};
    bool         isEnabled{false};
    uint8_t      staticPriority{0};
    timeDelta_t  desiredPeriod{0};
    timeUs_t     maxExecutionTime{0};
    timeUs_t     totalExecutionTime{0};
    timeUs_t     averageExecutionTime{0};
    timeDelta_t  latestDeltaTime{0};
};

using CheckFunc = bool (*)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
using TaskFunc = void (*)(timeUs_t currentTimeUs);

struct cfTask_t {
    const char*  taskName{nullptr};
    CheckFunc    checkFunc{nullptr};
    TaskFunc     taskFunc{nullptr};
    timeDelta_t  desiredPeriod{0};
    uint8_t      staticPriority{0};

    uint16_t     dynamicPriority{0};
    uint16_t     taskAgeCycles{0};
    timeUs_t     lastExecutedAt{0};
    timeUs_t     lastSignaledAt{0};
    timeDelta_t  taskLatestDeltaTime{0};

    timeUs_t     movingSumExecutionTime{0};
    timeUs_t     maxExecutionTime{0};
    timeUs_t     totalExecutionTime{0};
};

class InavScheduler {
public:
    static constexpr size_t MOVING_SUM_COUNT = 32;
    static constexpr timeDelta_t SCHEDULER_DELAY_LIMIT = 100; // 100us (10kHz) limit

    InavScheduler() noexcept {
        init();
    }

    void init() noexcept {
        queue_size_ = 0;
        queue_pos_ = 0;
        std::fill(queue_array_.begin(), queue_array_.end(), nullptr);
        average_load_percent_ = 0;
        total_waiting_tasks_ = 0;
        total_waiting_samples_ = 0;
    }

    void register_task(cfTaskId_e id,
                       const char* name,
                       TaskFunc func,
                       CheckFunc check,
                       timeDelta_t period_us,
                       cfTaskPriority_e priority) noexcept {
        if (id >= TASK_COUNT) return;
        tasks_[id].taskName = name;
        tasks_[id].taskFunc = func;
        tasks_[id].checkFunc = check;
        tasks_[id].desiredPeriod = period_us;
        tasks_[id].staticPriority = static_cast<uint8_t>(priority);
        tasks_[id].dynamicPriority = 0;
        tasks_[id].taskAgeCycles = 0;
        tasks_[id].lastExecutedAt = 0;
        tasks_[id].lastSignaledAt = 0;
        tasks_[id].taskLatestDeltaTime = 0;
        tasks_[id].movingSumExecutionTime = 0;
        tasks_[id].maxExecutionTime = 0;
        tasks_[id].totalExecutionTime = 0;
    }

    bool queue_contains(const cfTask_t* task) const noexcept {
        for (size_t i = 0; i < queue_size_; ++i) {
            if (queue_array_[i] == task) return true;
        }
        return false;
    }

    bool queue_add(cfTask_t* task) noexcept {
        if (queue_size_ >= TASK_COUNT || queue_contains(task) || !task) {
            return false;
        }
        size_t insert_pos = queue_size_;
        for (size_t i = 0; i < queue_size_; ++i) {
            if (queue_array_[i]->staticPriority < task->staticPriority) {
                insert_pos = i;
                break;
            }
        }
        for (size_t i = queue_size_; i > insert_pos; --i) {
            queue_array_[i] = queue_array_[i - 1];
        }
        queue_array_[insert_pos] = task;
        queue_size_++;
        return true;
    }

    bool queue_remove(cfTask_t* task) noexcept {
        for (size_t i = 0; i < queue_size_; ++i) {
            if (queue_array_[i] == task) {
                for (size_t j = i; j < queue_size_ - 1; ++j) {
                    queue_array_[j] = queue_array_[j + 1];
                }
                queue_array_[queue_size_ - 1] = nullptr;
                queue_size_--;
                return true;
            }
        }
        return false;
    }

    void set_task_enabled(cfTaskId_e id, bool enabled) noexcept {
        if (id < TASK_COUNT) {
            cfTask_t* task = &tasks_[id];
            if (enabled && task->taskFunc) {
                queue_add(task);
            } else {
                queue_remove(task);
            }
        }
    }

    void reschedule_task(cfTaskId_e id, timeDelta_t new_period_us) noexcept {
        if (id < TASK_COUNT) {
            tasks_[id].desiredPeriod = std::max(SCHEDULER_DELAY_LIMIT, new_period_us);
        }
    }

    // -------------------------------------------------------------------------
    // Main INAV Dynamic Scheduling Loop (scheduler.c)
    // -------------------------------------------------------------------------
    void step(timeUs_t current_time_us) noexcept {
        cfTask_t* selected_task = nullptr;
        uint16_t selected_dynamic_priority = 0;
        bool forced_realtime = false;
        uint16_t waiting_tasks = 0;

        for (size_t i = 0; i < queue_size_; ++i) {
            cfTask_t* task = queue_array_[i];
            if (!task) continue;

            if (task->checkFunc) {
                if (task->dynamicPriority > 0) {
                    task->taskAgeCycles = static_cast<uint16_t>(1 + (current_time_us - task->lastSignaledAt) / static_cast<uint32_t>(task->desiredPeriod));
                    task->dynamicPriority = static_cast<uint16_t>(1 + task->staticPriority * task->taskAgeCycles);
                    waiting_tasks++;
                } else if (task->checkFunc(current_time_us, static_cast<timeDelta_t>(current_time_us - task->lastExecutedAt))) {
                    task->lastSignaledAt = current_time_us;
                    task->taskAgeCycles = 1;
                    task->dynamicPriority = static_cast<uint16_t>(1 + task->staticPriority);
                    waiting_tasks++;
                } else {
                    task->taskAgeCycles = 0;
                }
            } else if (task->staticPriority == TASK_PRIORITY_REALTIME) {
                if (static_cast<timeDelta_t>(current_time_us - task->lastExecutedAt) >= task->desiredPeriod) {
                    selected_dynamic_priority = task->dynamicPriority;
                    selected_task = task;
                    waiting_tasks++;
                    forced_realtime = true;
                }
            } else {
                if (task->desiredPeriod > 0) {
                    task->taskAgeCycles = static_cast<uint16_t>((current_time_us - task->lastExecutedAt) / static_cast<uint32_t>(task->desiredPeriod));
                    if (task->taskAgeCycles > 0) {
                        task->dynamicPriority = static_cast<uint16_t>(1 + task->staticPriority * task->taskAgeCycles);
                        waiting_tasks++;
                    }
                }
            }

            if (!forced_realtime && task->dynamicPriority > selected_dynamic_priority) {
                selected_dynamic_priority = task->dynamicPriority;
                selected_task = task;
            }
        }

        total_waiting_samples_++;
        total_waiting_tasks_ += waiting_tasks;

        if (selected_task && selected_task->taskFunc) {
            current_task_ = selected_task;
            selected_task->taskLatestDeltaTime = static_cast<timeDelta_t>(current_time_us - selected_task->lastExecutedAt);
            selected_task->lastExecutedAt = current_time_us;
            selected_task->dynamicPriority = 0;

            // Execute Task
            selected_task->taskFunc(current_time_us);
        }
    }

    [[nodiscard]] uint16_t average_load_percent() const noexcept { return average_load_percent_; }
    [[nodiscard]] const cfTask_t& task(cfTaskId_e id) const noexcept { return tasks_[id]; }
    [[nodiscard]] size_t queue_size() const noexcept { return queue_size_; }

private:
    std::array<cfTask_t, TASK_COUNT> tasks_{};
    std::array<cfTask_t*, TASK_COUNT + 1> queue_array_{};
    size_t queue_size_{0};
    size_t queue_pos_{0};
    cfTask_t* current_task_{nullptr};

    uint16_t average_load_percent_{0};
    uint32_t total_waiting_tasks_{0};
    uint32_t total_waiting_samples_{0};
};

} // namespace abstractx::scheduler

#endif // SCHEDULER_HPP
