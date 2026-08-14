/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Shared Linux POSIX Real-Time Hardener (Core 3, SCHED_FIFO Priority 99, mlockall)
 *
 * Shared between Linux Quad-Core SBC (e.g. Raspberry Pi 5 / Jetson Orin) and Desktop SITL Simulator.
 */

#ifndef LINUX_RT_HARDENER_HPP
#define LINUX_RT_HARDENER_HPP

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <cstdint>
#include <cstring>

namespace abstractx::target::linux_common {

class LinuxRtHardener {
public:
    // Harden Single Isolated Real-Time CPU Core 3 (SCHED_FIFO Priority 99)
    // Gracefully degrades under non-root GDB debugging sessions
    static bool harden_single_realtime_core(uint32_t rt_core_id = 3) noexcept {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(rt_core_id, &cpuset);
        
        // Pin to isolated CPU Core 3
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        struct sched_param param{};
        param.sched_priority = 99; // Maximum POSIX Real-Time Priority
        
        // Attempt POSIX SCHED_FIFO setting (gracefully continues if non-root GDB session)
        (void)pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        return true;
    }

    // Lock Physical Memory Pages & Pre-Fault Stack Space
    static bool lock_process_memory() noexcept {
        // Attempt physical page locking (gracefully continues if non-root GDB session)
        (void)mlockall(MCL_CURRENT | MCL_FUTURE);

        // Pre-fault 64 KB stack memory
        volatile char stack_buf[65536];
        std::memset((void*)stack_buf, 0, sizeof(stack_buf));
        return true;
    }
};

} // namespace abstractx::target::linux_common

// Backwards-compatible alias for existing target namespaces
namespace abstractx::target::sitl {
    using LinuxRtHardener = abstractx::target::linux_common::LinuxRtHardener;
}

#endif // __linux__

#endif // LINUX_RT_HARDENER_HPP
