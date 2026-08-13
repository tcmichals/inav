/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Base IMU Sample & Driver Interface
 */

#ifndef IMU_BASE_HPP
#define IMU_BASE_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers::imu {

struct RegWrite {
    uint8_t reg_addr;
    uint8_t value;
};

struct ImuSample {
    std::array<float, 3> accel_g{0.0f, 0.0f, 1.0f};    // Accel X, Y, Z in Gs
    std::array<float, 3> gyro_deg_s{0.0f, 0.0f, 0.0f}; // Gyro X, Y, Z in deg/s
    float temperature_c{25.0f};                         // Temperature in Celsius
    uint64_t timestamp_ns{0};                           // 64-bit Hardware DRDY timestamp
};

} // namespace abstractx::drivers::imu

#endif // IMU_BASE_HPP
