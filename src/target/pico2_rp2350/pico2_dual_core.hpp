/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - RP2350 Dual-Core Asymmetric Architecture (Core 0 I/O + Core 1 Flight)
 *
 * Architecture Specification:
 *   Core 0 (I/O & Telemetry Processor):
 *     - CYW43439 Wi-Fi (802.11n) & lwIP stack
 *     - MSP TCP Server on port 5760 (INAV Configurator GUI)
 *     - BareCTF 1.8 Blackbox Logger (draining to 4MB QSPI flash or UDP/TCP sockets)
 *     - Peripheral bus masters (UART GPS, non-real-time housekeeping)
 *
 *   Core 1 (Isolated Real-Time Flight Processor):
 *     - Zero OS, zero preemption, zero heap allocations (deterministic WCET)
 *     - 8 kHz C++20 coroutine flight engine loop (FlightEngine)
 *     - RP2350 PIO2 Auto-DMA IMU streaming (0.0% CPU overhead)
 *     - Dynamic Gyro Notch + Spectral Analysis + INAV Mahony AHRS
 *     - 3D Position Estimation + S-Curve Guidance + Rate PID + DShot Motors
 *
 *   Inter-Core Interconnect:
 *     - Lock-free 64-byte SPSC rings in shared SRAM (telemetry_ring, logging_ring)
 *     - Zero mutex locks, zero spinlocks, zero priority inversions.
 */

#ifndef PICO2_DUAL_CORE_HPP
#define PICO2_DUAL_CORE_HPP

#include "spsc_tlp_ring.hpp"
#include "asp_tlp64.hpp"
#include <cstdint>
#include <string_view>

namespace abstractx::target::pico2 {

struct DualCoreMetrics {
    uint32_t core0_poll_rate_hz{0};
    uint32_t core1_flight_loop_hz{8000};
    uint32_t telemetry_backlog_depth{0};
    uint32_t logging_backlog_depth{0};
    bool core1_healthy{true};
    bool wifi_connected{false};
    bool msp_client_active{false};
};

class Pico2DualCoreTopology {
public:
    static constexpr std::string_view CORE0_ROLE = "Core 0: I/O Processor (Wi-Fi, lwIP MSP TCP 5760, Blackbox CTF)";
    static constexpr std::string_view CORE1_ROLE = "Core 1: Isolated RT Flight Processor (8 kHz Engine, PIO Auto-DMA, PID)";

    [[nodiscard]] static constexpr uint32_t target_flight_rate_hz() noexcept { return 8000u; }
    [[nodiscard]] static constexpr uint32_t msp_server_port() noexcept { return 5760u; }
    [[nodiscard]] static constexpr uint32_t blackbox_buffer_capacity() noexcept { return 64u; }
};

} // namespace abstractx::target::pico2

#endif // PICO2_DUAL_CORE_HPP
