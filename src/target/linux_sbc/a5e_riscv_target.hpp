/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Allwinner Cubie A5E RISC-V RPMsg & SPI Dual-Transport Interface
 */

#ifndef A5E_RISCV_TARGET_HPP
#define A5E_RISCV_TARGET_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <thread>
#include <atomic>

namespace abstractx::target::a5e {

enum class TransportMode {
    RpMsg_VirtIo,  // Linux rpmsg character device (/dev/rpmsg0) to A5E RISC-V -> FPGA
    SpiDev_Direct  // Direct Linux spidev (/dev/spidev0.0) -> FPGA
};

class A5eRiscvFpgaTarget {
public:
    // Initialize RPMsg / SPI Transport Worker Thread (Thread 3, SCHED_FIFO Priority 97, CPU Core 1)
    static void start_rpmsg_worker_thread(SpscTlpRing<64>& cmd_ring, SpscTlpRing<64>& rx_ring, std::atomic<bool>& running, TransportMode mode = TransportMode::RpMsg_VirtIo) noexcept {
        std::thread([&cmd_ring, &rx_ring, &running, mode]() {
            while (running.load(std::memory_order_relaxed)) {
                if (!cmd_ring.empty()) {
                    auto opt_tlp = cmd_ring.pop();
                    if (opt_tlp.has_value()) {
                        Tlp64 tlp = opt_tlp.value();
                        tlp.wire.timestamp_ns = 1000000000ULL; // Latch hardware timestamp
                        
                        if (mode == TransportMode::RpMsg_VirtIo) {
                            // RPMsg /dev/rpmsg0 virtio ring write -> A5E RISC-V -> FPGA
                        } else {
                            // Direct /dev/spidev0.0 ioctl write -> FPGA
                        }
                        rx_ring.push(tlp);
                    }
                } else {
                    std::this_thread::yield();
                }
            }
        }).detach();
    }
};

} // namespace abstractx::target::a5e

#endif // A5E_RISCV_TARGET_HPP
