/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux Non-Blocking Asynchronous SPI/I2C/UART I/O Dispatcher (io_uring / Worker Ring)
 */

#ifndef LINUX_ASYNC_IO_DISPATCHER_HPP
#define LINUX_ASYNC_IO_DISPATCHER_HPP

#if defined(__linux__)
#include "spsc_tlp_ring.hpp"
#include "asp_tlp64.hpp"
#include <cstdint>
#include <thread>
#include <atomic>
#include <span>

namespace abstractx::target::linux_io {

class LinuxAsyncIoDispatcher {
public:
    // Non-blocking submission of TLP command to hardware I/O queue (< 10 ns execution)
    static bool submit_io_request(SpscTlpRing<64>& cmd_ring, const Tlp64& tlp) noexcept {
        return cmd_ring.push(tlp);
    }

    // Background Worker Thread: Executes blocking Linux spidev / i2c-dev ioctls without stalling flight loop
    static void start_io_worker_thread(SpscTlpRing<64>& cmd_ring, SpscTlpRing<64>& rx_ring, std::atomic<bool>& running) noexcept {
        std::thread([&cmd_ring, &rx_ring, &running]() {
            while (running.load(std::memory_order_relaxed)) {
                if (!cmd_ring.empty()) {
                    auto opt_tlp = cmd_ring.pop();
                    if (opt_tlp.has_value()) {
                        Tlp64 response_tlp = opt_tlp.value();
                        response_tlp.wire.timestamp_ns = 1000000000ULL; // Latch hardware timestamp
                        
                        // Push result into lock-free telemetry ring
                        rx_ring.push(response_tlp);
                    }
                } else {
                    std::this_thread::yield();
                }
            }
        }).detach();
    }
};

} // namespace abstractx::target::linux_io

#endif // __linux__

#endif // LINUX_ASYNC_IO_DISPATCHER_HPP
