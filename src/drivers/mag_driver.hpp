/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Universal Magnetometer / Compass Driver (QMC5883L, HMC5883L, IST8310, LIS3MDL)
 */

#ifndef MAG_DRIVER_HPP
#define MAG_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers {

// Supported Magnetometer Hardware Sensor Chipsets
enum class MagSensorType : uint8_t {
    Qmc5883L = 0, // QST QMC5883L
    Hmc5883L = 1, // Honeywell HMC5883L
    Ist8310  = 2, // Isentek IST8310
    Lis3mdl  = 3  // STMicroelectronics LIS3MDL
};

struct MagSample {
    std::array<float, 3> mag_mgauss{0.0f, 0.0f, 0.0f}; // Mag X, Y, Z in milliGauss
    float heading_deg{0.0f};                            // Magnetic compass heading (0..360 deg)
    uint64_t timestamp_ns{0};                           // 64-bit Hardware timestamp
    MagSensorType sensor_type{MagSensorType::Qmc5883L};
};

class MagDriver {
public:
    constexpr MagDriver() noexcept = default;

    // Parse Magnetometer 64B TLP frame into normalized MagSample
    static MagSample parse_tlp(const Tlp64& tlp, MagSensorType chip = MagSensorType::Qmc5883L) noexcept {
        MagSample sample{};
        sample.sensor_type = chip;
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_mx = static_cast<int16_t>((p[0] << 8) | p[1]);
        int16_t raw_my = static_cast<int16_t>((p[2] << 8) | p[3]);
        int16_t raw_mz = static_cast<int16_t>((p[4] << 8) | p[5]);

        // Normalize raw readings to milliGauss
        sample.mag_mgauss[0] = static_cast<float>(raw_mx);
        sample.mag_mgauss[1] = static_cast<float>(raw_my);
        sample.mag_mgauss[2] = static_cast<float>(raw_mz);

        // Compute magnetic heading angle (deg)
        float heading_rad = std::atan2(sample.mag_mgauss[1], sample.mag_mgauss[0]);
        if (heading_rad < 0.0f) heading_rad += 2.0f * 3.14159265f;
        sample.heading_deg = heading_rad * (180.0f / 3.14159265f);

        return sample;
    }
};

} // namespace abstractx::drivers

#endif // MAG_DRIVER_HPP
