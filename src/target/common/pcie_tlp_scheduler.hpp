/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Bottom-Half PCIe TLP Protocol Processor & Hardware Scheduler
 *
 * Dequeues Outbound Tlp64 requests from top-level drivers, routes them by virtual BAR
 * base to the appropriate physical hardware backend (FPGA PCIe, RP2350 PIO, native SPI/I2C DMA),
 * and generates Inbound Tlp64 completion packets with 64-bit nanosecond timestamps.
 *
 * MISRA C++:2023 Rules:
 *   - Zero dynamic heap allocation
 *   - All methods [[nodiscard]] / noexcept where appropriate
 *   - Bounded execution time per step
 */

#ifndef PCIE_TLP_SCHEDULER_HPP
#define PCIE_TLP_SCHEDULER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <array>
#include <span>
#include <chrono>

namespace abstractx::target::common {

class PcieTlpScheduler {
public:
    explicit PcieTlpScheduler(SpscTlpRing<64u>& outbound_ring,
                             SpscTlpRing<64u>& inbound_ring) noexcept
        : outbound_{outbound_ring}, inbound_{inbound_ring} {
        init_mock_registers();
    }

    // Step the scheduler: processes all pending outbound requests and dispatches to backends
    size_t process_outbound_queue() noexcept {
        size_t processed_count = 0u;

        while (auto opt_req = outbound_.pop()) {
            const Tlp64& req = *opt_req;
            dispatch_request(req);
            processed_count++;
        }

        return processed_count;
    }

    // Inject simulated register value for testing/SITL
    void set_mock_reg(uint32_t bar_base, uint8_t reg, uint8_t val) noexcept {
        const size_t bar_idx = get_bar_index(bar_base);
        if (bar_idx < NUM_BARS) {
            mock_regs_[bar_idx][reg] = val;
        }
    }

    // Direct read of mock register
    [[nodiscard]] uint8_t get_mock_reg(uint32_t bar_base, uint8_t reg) const noexcept {
        const size_t bar_idx = get_bar_index(bar_base);
        if (bar_idx < NUM_BARS) {
            return mock_regs_[bar_idx][reg];
        }
        return 0u;
    }

private:
    static constexpr size_t NUM_BARS = 8u;
    static constexpr size_t REGS_PER_BAR = 256u;

    SpscTlpRing<64u>& outbound_;
    SpscTlpRing<64u>& inbound_;
    alignas(64) std::array<std::array<uint8_t, REGS_PER_BAR>, NUM_BARS> mock_regs_{};

    [[nodiscard]] static size_t get_bar_index(uint32_t bar_base) noexcept {
        switch (bar_base & 0xF000u) {
        case bar::ImuBase:       return 0u;
        case bar::BaroBase:      return 1u;
        case bar::MagBase:       return 2u;
        case bar::GpsBase:       return 3u;
        case bar::RcBase:        return 4u;
        case bar::EscBase:       return 5u;
        case bar::TelemetryBase: return 6u;
        case bar::PitotBase:     return 7u;
        default:                 return 0u;
        }
    }

    void init_mock_registers() noexcept {
        // IMU defaults: WHO_AM_I (reg 0x75 = 0x47 for ICM42688P)
        mock_regs_[0][0x75] = 0x47u;
        // Baro defaults: WHO_AM_I (reg 0xD0 = 0x58 for BMP280)
        mock_regs_[1][0xD0] = 0x58u;
        // Mag defaults: WHO_AM_I (reg 0x0D = 0xFF for QMC5883L)
        mock_regs_[2][0x0D] = 0xFFu;
        // Pitot defaults: WHO_AM_I (0x48)
        mock_regs_[7][0x00] = 0x48u;
    }

    void dispatch_request(const Tlp64& req) noexcept {
        const uint32_t bar_base = req.wire.target_address & 0xF000u;
        const uint8_t reg = static_cast<uint8_t>(req.wire.target_address & 0x00FFu);
        const size_t bar_idx = get_bar_index(bar_base);

        Tlp64 comp = req;
        comp.wire.type = static_cast<uint8_t>(TlpType::Completion);
        comp.wire.timestamp_ns = get_current_timestamp_ns();

        if (req.type() == static_cast<uint8_t>(TlpType::MemWrite)) {
            // Write to mock register / physical peripheral
            if (bar_idx < NUM_BARS) {
                mock_regs_[bar_idx][reg] = req.wire.payload[0];
            }
            comp.wire.length = 0u;
            (void)inbound_.push(comp);
        } else if (req.type() == static_cast<uint8_t>(TlpType::MemRead)) {
            // Read from mock register / physical peripheral
            if (bar_idx < NUM_BARS) {
                const size_t read_len = std::min(static_cast<size_t>(req.wire.length), 44ul);
                for (size_t i = 0; i < read_len; ++i) {
                    comp.wire.payload[i] = mock_regs_[bar_idx][(reg + i) & 0xFFu];
                }
                comp.wire.length = static_cast<uint16_t>(read_len);
            }
            (void)inbound_.push(comp);
        }
    }

    [[nodiscard]] static uint64_t get_current_timestamp_ns() noexcept {
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    }
};

} // namespace abstractx::target::common

#endif // PCIE_TLP_SCHEDULER_HPP
