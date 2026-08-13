/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux Single Real-Time Core POSIX Hardener (Core 3, SCHED_FIFO Priority 99)
 */

#ifndef LINUX_RT_HARDENER_HPP
#define LINUX_RT_HARDENER_HPP

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <cstdint>
#include <cstring>

namespace abstractx::target::sitl {

class LinuxRtHardener {
public:
    // Harden Single Isolated Real-Time CPU Core 3 (SCHED_FIFO Priority 99)
    // Core 3 executes the 8 kHz Flight Loop AND non-blocking epoll I/O reactor concurrently (< 4.3 us frame time)
    static bool harden_single_realtime_core(uint32_t rt_core_id = 3) noexcept {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(rt_core_id, &cpuset);
        
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            return false;
        }

        struct sched_param param{};
        param.sched_priority = 99; // Maximum POSIX Real-Time Priority
        return (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0);
    }

    // Lock Physical Memory Pages & Pre-Fault Stack Space
    static bool lock_process_memory() noexcept {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            return false;
        }

        // Pre-fault 64 KB stack memory
        volatile char stack_buf[65536];
        std::memset((void*)stack_buf, 0, sizeof(stack_buf));
        return true;
    }
};

} // namespace abstractx::target::sitl

#endif // __linux__

#endif // LINUX_RT_HARDENER_HPP
