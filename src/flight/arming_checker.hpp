/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Pre-Arming Safety Checker & Hardware Validation Engine
 */

#ifndef ARMING_CHECKER_HPP
#define ARMING_CHECKER_HPP

#include <cstdint>

namespace abstractx::flight {

enum class ArmingPreventReason : uint32_t {
    None              = 0,
    NoImu             = (1 << 0), // IMU sensor not responding
    NoBaro            = (1 << 1), // Baro sensor missing
    NoGpsFix          = (1 << 2), // 3D GPS fix required for RTH
    ThrottleNotZero   = (1 << 3), // Stick throttle above minimum
    SystemCalibrating = (1 << 4), // Gyro calibration in progress
    BatteryCritical   = (1 << 5), // Battery voltage below 3.3V/cell
    AngleTilted       = (1 << 6)  // Quadcopter tilted > 25 degrees on ground
};

class ArmingChecker {
public:
    constexpr ArmingChecker() noexcept = default;

    static uint32_t evaluate_arming_flags(
        bool imu_ok,
        bool baro_ok,
        bool baro_required,
        bool gps_3d_fix,
        bool gps_required,
        uint16_t throttle_us,
        bool calibrating,
        float cell_voltage_v,
        bool battery_required,
        float roll_deg,
        float pitch_deg
    ) noexcept {
        uint32_t flags = 0;

        if (!imu_ok) flags |= static_cast<uint32_t>(ArmingPreventReason::NoImu);
        if (baro_required && !baro_ok) flags |= static_cast<uint32_t>(ArmingPreventReason::NoBaro);
        if (gps_required && !gps_3d_fix) flags |= static_cast<uint32_t>(ArmingPreventReason::NoGpsFix);
        if (throttle_us > 1100) flags |= static_cast<uint32_t>(ArmingPreventReason::ThrottleNotZero);
        if (calibrating) flags |= static_cast<uint32_t>(ArmingPreventReason::SystemCalibrating);
        if (battery_required && cell_voltage_v < 3.3f) flags |= static_cast<uint32_t>(ArmingPreventReason::BatteryCritical);
        if (std::abs(roll_deg) > 25.0f || std::abs(pitch_deg) > 25.0f) {
            flags |= static_cast<uint32_t>(ArmingPreventReason::AngleTilted);
        }

        return flags;
    }
};

} // namespace abstractx::flight

#endif // ARMING_CHECKER_HPP
