/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Bosch BMI088 High-G Industrial IMU Driver
 */

#ifndef BMI088_DRIVER_HPP
#define BMI088_DRIVER_HPP

#include "imu_base.hpp"
#include <span>

namespace abstractx::drivers::imu {

class Bmi088 {
public:
    static constexpr uint8_t ACCEL_CHIP_ID = 0x1E;
    static constexpr uint8_t GYRO_CHIP_ID  = 0x0F;

    static std::span<const RegWrite> init_sequence() noexcept {
        static constexpr RegWrite seq[] = {
            { 0x7E, 0xB6 }, { 0x40, 0xA8 }, { 0x41, 0x03 }, { 0x14, 0xB6 }, { 0x0F, 0x00 }, { 0x10, 0x81 }
        };
        return std::span<const RegWrite>(seq);
    }

    static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_ax = static_cast<int16_t>((p[0] << 8) | p[1]);
        int16_t raw_ay = static_cast<int16_t>((p[2] << 8) | p[3]);
        int16_t raw_az = static_cast<int16_t>((p[4] << 8) | p[5]);

        int16_t raw_gx = static_cast<int16_t>((p[6] << 8) | p[7]);
        int16_t raw_gy = static_cast<int16_t>((p[8] << 8) | p[9]);
        int16_t raw_gz = static_cast<int16_t>((p[10] << 8) | p[11]);

        sample.accel_g[0] = static_cast<float>(raw_ax) / 1365.33f;
        sample.accel_g[1] = static_cast<float>(raw_ay) / 1365.33f;
        sample.accel_g[2] = static_cast<float>(raw_az) / 1365.33f;

        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.384f;
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.384f;
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.384f;

        return sample;
    }
};

} // namespace abstractx::drivers::imu

#endif // BMI088_DRIVER_HPP
