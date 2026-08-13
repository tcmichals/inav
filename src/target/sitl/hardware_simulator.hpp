/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux Hardware Simulation Engine (SITL / HIL)
 */

#ifndef HARDWARE_SIMULATOR_HPP
#define HARDWARE_SIMULATOR_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "blackbox_logger.hpp"
#include <cstdint>
#include <cmath>

namespace abstractx::sitl {

class HardwareSimulator {
public:
    HardwareSimulator() noexcept : current_timestamp_ns_(1000000000ULL) {}

    // Step hardware simulation by `dt_us` microseconds
    void step(uint64_t dt_us, SpscTlpRing<64>& telemetry_ring) noexcept {
        current_timestamp_ns_ += (dt_us * 1000ULL);
        step_count_++;

        // 1. Simulate IMU continuous Auto-DMA stream (ICM-42688-P emulation at 1 kHz)
        if (imu_enabled_ && (step_count_ % 1 == 0)) {
            Tlp64 imu_tlp{};
            imu_tlp.wire.type = static_cast<uint8_t>(TlpType::DmaStream);
            imu_tlp.wire.channel = static_cast<uint8_t>(Channel::Telemetry);
            imu_tlp.wire.target_address = bar::ImuBase + reg::imu::ContinuousAddr;
            imu_tlp.wire.timestamp_ns = current_timestamp_ns_;
            imu_tlp.wire.sequence = static_cast<uint16_t>(step_count_);

            // Emulate 1G Accel Z + subtle pitch vibration
            int16_t accel_z = 2048; // 1G @ 2048 LSB/g
            int16_t gyro_x = static_cast<int16_t>(10.0f * std::sin(step_count_ * 0.1f));

            // Pack 14-byte IMU burst (Accel X/Y/Z, Gyro X/Y/Z, Temp)
            imu_tlp.wire.payload[0] = 0; // Accel X High
            imu_tlp.wire.payload[1] = 0; // Accel X Low
            imu_tlp.wire.payload[2] = 0; // Accel Y High
            imu_tlp.wire.payload[3] = 0; // Accel Y Low
            imu_tlp.wire.payload[4] = static_cast<uint8_t>(accel_z >> 8);
            imu_tlp.wire.payload[5] = static_cast<uint8_t>(accel_z & 0xFF);
            imu_tlp.wire.payload[6] = static_cast<uint8_t>(gyro_x >> 8);
            imu_tlp.wire.payload[7] = static_cast<uint8_t>(gyro_x & 0xFF);

            telemetry_ring.push(imu_tlp);
        }
    }

    void handle_mem_write(uint32_t addr, uint32_t value) noexcept {
        if (addr == (bar::ImuBase + reg::imu::Control)) {
            imu_enabled_ = (value & 0x01) != 0;
        }
    }

    uint64_t timestamp_ns() const noexcept { return current_timestamp_ns_; }

private:
    uint64_t current_timestamp_ns_{0};
    uint64_t step_count_{0};
    bool imu_enabled_{true};
};

} // namespace abstractx::sitl

#endif // HARDWARE_SIMULATOR_HPP
