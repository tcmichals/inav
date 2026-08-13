/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated TDK ICM-42688-P IMU Driver
 */

#ifndef ICM42688P_DRIVER_HPP
#define ICM42688P_DRIVER_HPP

#include "imu_base.hpp"
#include <span>

namespace abstractx::drivers::imu {

class Icm42688P {
public:
    static constexpr uint8_t WHO_AM_I = 0x47;

    static std::span<const RegWrite> init_sequence() noexcept {
        static constexpr RegWrite seq[] = {
            { 0x11, 0x01 }, { 0x4E, 0x0F }, { 0x4F, 0x06 }, { 0x50, 0x06 }, { 0x52, 0x01 }
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

        int16_t raw_temp = static_cast<int16_t>((p[12] << 8) | p[13]);

        sample.accel_g[0] = static_cast<float>(raw_ax) / 2048.0f;
        sample.accel_g[1] = static_cast<float>(raw_ay) / 2048.0f;
        sample.accel_g[2] = static_cast<float>(raw_az) / 2048.0f;

        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.4f;
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.4f;
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.4f;

        sample.temperature_c = 25.0f + (static_cast<float>(raw_temp) / 326.8f);

        return sample;
    }
};

} // namespace abstractx::drivers::imu

#endif // ICM42688P_DRIVER_HPP
