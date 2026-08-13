/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux Real-Time Thread Hardening (CPU Affinity, SCHED_FIFO, mlockall)
 */

#ifndef LINUX_RT_HARDENER_HPP
#define LINUX_RT_HARDENER_HPP

#if defined(__linux__)
#include <sys/mman.h>
#include <sched.h>
#include <pthread.h>
#include <cstring>
#include <cstdint>

namespace abstractx::target::linux_rt {

class LinuxRtHardener {
public:
    // Harden Linux Thread: Lock RAM, Pin to Isolated CPU Core, Set SCHED_FIFO Priority 99
    static bool harden_realtime_thread(int cpu_core_id = 3, int priority = 99) noexcept {
        // 1. Lock All Physical Memory Pages (Prevent Linux Page Fault Stalls)
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            // Note: Requires CAP_SYS_NICE or root permissions
        }

        // 2. Set Real-Time SCHED_FIFO Priority 99
        struct sched_param param{};
        param.sched_priority = priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            // Fallback to SCHED_RR if SCHED_FIFO fails
            pthread_setschedparam(pthread_self(), SCHED_RR, &param);
        }

        // 3. Pin Thread to Isolated CPU Core (CPU Affinity)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core_id, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            return false;
        }

        // 4. Pre-Fault Stack Memory Pages (Touch 64KB stack buffer)
        volatile uint8_t dummy_stack[65536];
        std::memset(const_cast<uint8_t*>(dummy_stack), 0, sizeof(dummy_stack));

        return true;
    }
};

} // namespace abstractx::target::linux_rt

#endif // __linux__

#endif // LINUX_RT_HARDENER_HPP
