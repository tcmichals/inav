/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated InvenSense MPU-6000 IMU Driver
 */

#ifndef MPU6000_DRIVER_HPP
#define MPU6000_DRIVER_HPP

#include "imu_base.hpp"
#include <span>

namespace abstractx::drivers::imu {

class Mpu6000 {
public:
    static constexpr uint8_t WHO_AM_I = 0x68;

    static std::span<const RegWrite> init_sequence() noexcept {
        static constexpr RegWrite seq[] = {
            { 0x6B, 0x80 }, // PWR_MGMT_1: Reset
            { 0x6B, 0x03 }, // PWR_MGMT_1: Clock PLL Z
            { 0x1A, 0x02 }, // CONFIG: DLPF 98 Hz
            { 0x1B, 0x18 }, // GYRO_CONFIG: +-2000 dps
            { 0x1C, 0x10 }  // ACCEL_CONFIG: +-8g
        };
        return std::span<const RegWrite>(seq);
    }

    static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload.data();

        int16_t raw_ax = static_cast<int16_t>((p[0] << 8) | p[1]);
        int16_t raw_ay = static_cast<int16_t>((p[2] << 8) | p[3]);
        int16_t raw_az = static_cast<int16_t>((p[4] << 8) | p[5]);

        int16_t raw_gx = static_cast<int16_t>((p[6] << 8) | p[7]);
        int16_t raw_gy = static_cast<int16_t>((p[8] << 8) | p[9]);
        int16_t raw_gz = static_cast<int16_t>((p[10] << 8) | p[11]);

        sample.accel_g[0] = static_cast<float>(raw_ax) / 4096.0f; // 8G scale
        sample.accel_g[1] = static_cast<float>(raw_ay) / 4096.0f;
        sample.accel_g[2] = static_cast<float>(raw_az) / 4096.0f;

        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.4f;
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.4f;
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.4f;

        return sample;
    }
};

} // namespace abstractx::drivers::imu

#endif // MPU6000_DRIVER_HPP
