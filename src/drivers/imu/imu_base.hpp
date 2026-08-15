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

// Configuration passed to every IMU driver's init() method.
// Values are read from ConfigRegistry::get().sensor at boot time.
struct ImuConfig {
    uint32_t odr_hz{8000u};          // Target output data rate in Hz
    uint8_t  accel_range_g{16u};     // Accel full-scale: 2, 4, 8, or 16 g
    uint16_t gyro_range_dps{2000u};  // Gyro full-scale: 250, 500, 1000, or 2000 dps
    bool     enable_drdy_int{true};  // Route DRDY to INT1 GPIO pin
    uint16_t dlpf_cutoff_hz{0u};     // 0 = low-latency bypass / AAF only
};

// Returned by every IMU driver init() to distinguish failure modes.
enum class ImuInitResult : uint8_t {
    Ok             = 0u,
    WhoAmIMismatch = 1u,
    Timeout        = 2u,
    BusError       = 3u,
};

struct ImuSample {
    std::array<float, 3> accel_g{0.0f, 0.0f, 1.0f};    // Accel X, Y, Z in Gs
    std::array<float, 3> gyro_deg_s{0.0f, 0.0f, 0.0f}; // Gyro X, Y, Z in deg/s
    float temperature_c{25.0f};                         // Temperature in Celsius
    uint64_t timestamp_ns{0};                           // 64-bit Hardware DRDY timestamp
};

} // namespace abstractx::drivers::imu

#endif // IMU_BASE_HPP
