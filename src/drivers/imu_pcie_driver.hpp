/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Platform-Agnostic PCIe BAR IMU Driver
 */

#ifndef IMU_PCIE_DRIVER_HPP
#define IMU_PCIE_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers {

struct ImuSample {
    std::array<float, 3> accel_g{0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_deg_s{0.0f, 0.0f, 0.0f};
    uint64_t timestamp_ns{0};
};

class ImuPcieDriver {
public:
    constexpr ImuPcieDriver() noexcept = default;

    // Parse incoming continuous 14-byte IMU burst 64B TLP
    static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.timestamp_ns();

        // Unpack raw int16_t values from TLP payload
        int16_t ax = static_cast<int16_t>((tlp.wire.payload[0] << 8) | tlp.wire.payload[1]);
        int16_t ay = static_cast<int16_t>((tlp.wire.payload[2] << 8) | tlp.wire.payload[3]);
        int16_t az = static_cast<int16_t>((tlp.wire.payload[4] << 8) | tlp.wire.payload[5]);

        int16_t gx = static_cast<int16_t>((tlp.wire.payload[6] << 8) | tlp.wire.payload[7]);
        int16_t gy = static_cast<int16_t>((tlp.wire.payload[8] << 8) | tlp.wire.payload[9]);
        int16_t gz = static_cast<int16_t>((tlp.wire.payload[10] << 8) | tlp.wire.payload[11]);

        // Convert to physical units (ICM-42688-P scale factors: 2048 LSB/g, 16.4 LSB/deg/s)
        sample.accel_g[0] = static_cast<float>(ax) / 2048.0f;
        sample.accel_g[1] = static_cast<float>(ay) / 2048.0f;
        sample.accel_g[2] = static_cast<float>(az) / 2048.0f;

        sample.gyro_deg_s[0] = static_cast<float>(gx) / 16.4f;
        sample.gyro_deg_s[1] = static_cast<float>(gy) / 16.4f;
        sample.gyro_deg_s[2] = static_cast<float>(gz) / 16.4f;

        return sample;
    }
};

} // namespace abstractx::drivers

#endif // IMU_PCIE_DRIVER_HPP
