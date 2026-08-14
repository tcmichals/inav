/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - InvenSense MPU6000 / MPU6500 / ICM-20689 IMU Driver
 */

#ifndef MPU6000_HPP
#define MPU6000_HPP

#include "asp_tlp64.hpp"
#include "imu_pcie_driver.hpp"
#include <cstdint>

namespace abstractx::drivers::imu {

class Mpu6000 {
public:
    constexpr Mpu6000() noexcept = default;

    // Parse 64-byte PCIe TLP IMU Burst containing 14 bytes (Accel 6B + Temp 2B + Gyro 6B)
    static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        // Raw 16-bit big-endian register values
        int16_t raw_ax = static_cast<int16_t>((tlp.wire.payload[0] << 8) | tlp.wire.payload[1]);
        int16_t raw_ay = static_cast<int16_t>((tlp.wire.payload[2] << 8) | tlp.wire.payload[3]);
        int16_t raw_az = static_cast<int16_t>((tlp.wire.payload[4] << 8) | tlp.wire.payload[5]);

        int16_t raw_gx = static_cast<int16_t>((tlp.wire.payload[8] << 8) | tlp.wire.payload[9]);
        int16_t raw_gy = static_cast<int16_t>((tlp.wire.payload[10] << 8) | tlp.wire.payload[11]);
        int16_t raw_gz = static_cast<int16_t>((tlp.wire.payload[12] << 8) | tlp.wire.payload[13]);

        // MPU6000 +/-16g scale (2048 LSB/g)
        sample.accel_g[0] = static_cast<float>(raw_ax) / 2048.0f;
        sample.accel_g[1] = static_cast<float>(raw_ay) / 2048.0f;
        sample.accel_g[2] = static_cast<float>(raw_az) / 2048.0f;

        // MPU6000 +/-2000 deg/s scale (16.4 LSB/(deg/s))
        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.4f;
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.4f;
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.4f;

        return sample;
    }
};

} // namespace abstractx::drivers::imu

#endif // MPU6000_HPP
