/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux POSIX Real-Time Thread Hardener (1.0 GHz Single-Core / Multi-Core Modes)
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
    // Single 1.0 GHz CPU Core Mode (Flight Loop + epoll Reactor on CPU Core 3, SCHED_FIFO Priority 99)
    static bool harden_single_core_unified(uint32_t cpu_core_id = 3) noexcept {
        return configure_thread(pthread_self(), 99, cpu_core_id);
    }

    // Configure Flight Loop Real-Time Thread (Priority 99, CPU Core 3)
    static bool harden_flight_thread(uint32_t cpu_core_id = 3) noexcept {
        return configure_thread(pthread_self(), 99, cpu_core_id);
    }

    // Configure Hardware I/O Offloader Thread (Priority 98, CPU Core 2)
    static bool harden_io_thread(pthread_t io_thread_handle, uint32_t cpu_core_id = 2) noexcept {
        return configure_thread(io_thread_handle, 98, cpu_core_id);
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

private:
    static bool configure_thread(pthread_t thread, int priority, uint32_t cpu_core_id) noexcept {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core_id, &cpuset);
        
        if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
            return false;
        }

        struct sched_param param{};
        param.sched_priority = priority;
        return (pthread_setschedparam(thread, SCHED_FIFO, &param) == 0);
    }
};

} // namespace abstractx::target::sitl

#endif // __linux__

#endif // LINUX_RT_HARDENER_HPP
