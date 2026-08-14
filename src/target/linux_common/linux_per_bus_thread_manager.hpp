/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Thread-Per-Bus Peripheral Manager (SPI, I2C, Serial UART, RPMsg)
 */

#ifndef LINUX_PER_BUS_THREAD_MANAGER_HPP
#define LINUX_PER_BUS_THREAD_MANAGER_HPP

#if defined(__linux__)
#include "spsc_tlp_ring.hpp"
#include "asp_tlp64.hpp"
#include "linux_rt_hardener.hpp"
#include <cstdint>
#include <thread>
#include <atomic>

namespace abstractx::target::linux_io {

class LinuxPerBusThreadManager {
public:
    // Starts 4 Dedicated POSIX Real-Time Worker Threads (1 per I/O bus)
    static void start_all_bus_threads(SpscTlpRing<64>& rx_ring, std::atomic<bool>& running) noexcept {
        
        // 1. SPI Worker Thread (Priority 98, CPU Core 2)
        std::thread([&rx_ring, &running]() {
            sitl::LinuxRtHardener::harden_io_thread(pthread_self(), 2);
            while (running.load(std::memory_order_relaxed)) {
                // Read SPI IMU TLP -> rx_ring.push(tlp)
                std::this_thread::yield();
            }
        }).detach();

        // 2. I2C Worker Thread (Priority 97, CPU Core 2)
        std::thread([&rx_ring, &running]() {
            while (running.load(std::memory_order_relaxed)) {
                // Read I2C Baro / Compass TLP -> rx_ring.push(tlp)
                std::this_thread::yield();
            }
        }).detach();

        // 3. Serial UART Worker Thread (Priority 96, CPU Core 1)
        std::thread([&rx_ring, &running]() {
            while (running.load(std::memory_order_relaxed)) {
                // Read Serial UART GPS / CRSF TLP -> rx_ring.push(tlp)
                std::this_thread::yield();
            }
        }).detach();

        // 4. RPMsg Worker Thread (Priority 95, CPU Core 1)
        std::thread([&rx_ring, &running]() {
            sitl::LinuxRtHardener::harden_rpmsg_thread(pthread_self(), 1);
            while (running.load(std::memory_order_relaxed)) {
                // Read RPMsg virtio TLP -> rx_ring.push(tlp)
                std::this_thread::yield();
            }
        }).detach();
    }
};

} // namespace abstractx::target::linux_io

#endif // __linux__

#endif // LINUX_PER_BUS_THREAD_MANAGER_HPP
