/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Betaflight / INAV Contributors (BorisB, Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production 3D Sensor Alignment & Board Rotation Engine
 *
 * Reference:
 *   - Upstream INAV: src/main/sensors/boardalignment.c
 *   - Upstream Betaflight: src/main/sensors/boardalignment.c
 *
 * Standard Alignments:
 *   1. CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG
 *   2. CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP
 *   3. Custom Euler Angles (Roll, Pitch, Yaw in 0.1 deg increments)
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef SENSORS_SENSOR_ALIGNMENT_HPP
#define SENSORS_SENSOR_ALIGNMENT_HPP

#include <cstdint>
#include <cmath>
#include "flight/pid.hpp"

namespace abstractx::sensors {

enum class SensorAlignment : uint8_t {
    ALIGN_DEFAULT      = 0u,
    CW0_DEG            = 1u,
    CW90_DEG           = 2u,
    CW180_DEG          = 3u,
    CW270_DEG          = 4u,
    CW0_DEG_FLIP       = 5u,
    CW90_DEG_FLIP      = 6u,
    CW180_DEG_FLIP     = 7u,
    CW270_DEG_FLIP     = 8u,
    ALIGN_CUSTOM       = 9u
};

struct BoardAlignmentConfig {
    SensorAlignment imu_alignment{SensorAlignment::CW0_DEG};
    SensorAlignment mag_alignment{SensorAlignment::CW0_DEG};
    int16_t         roll_decideg{0};   // 0.1 deg
    int16_t         pitch_decideg{0};
    int16_t         yaw_decideg{0};
};

class SensorAlignmentEngine {
public:
    constexpr SensorAlignmentEngine() noexcept = default;

    // -------------------------------------------------------------------------
    // Standard 8-Way Axis Rotation Transformation
    // -------------------------------------------------------------------------
    [[nodiscard]] static flight::Axis3f align_standard(const flight::Axis3f& in, SensorAlignment align) noexcept {
        flight::Axis3f out{};

        switch (align) {
            case SensorAlignment::ALIGN_DEFAULT:
            case SensorAlignment::CW0_DEG:
                out.roll  = in.roll;
                out.pitch = in.pitch;
                out.yaw   = in.yaw;
                break;

            case SensorAlignment::CW90_DEG:
                out.roll  = in.pitch;
                out.pitch = -in.roll;
                out.yaw   = in.yaw;
                break;

            case SensorAlignment::CW180_DEG:
                out.roll  = -in.roll;
                out.pitch = -in.pitch;
                out.yaw   = in.yaw;
                break;

            case SensorAlignment::CW270_DEG:
                out.roll  = -in.pitch;
                out.pitch = in.roll;
                out.yaw   = in.yaw;
                break;

            case SensorAlignment::CW0_DEG_FLIP:
                out.roll  = -in.roll;
                out.pitch = in.pitch;
                out.yaw   = -in.yaw;
                break;

            case SensorAlignment::CW90_DEG_FLIP:
                out.roll  = in.pitch;
                out.pitch = in.roll;
                out.yaw   = -in.yaw;
                break;

            case SensorAlignment::CW180_DEG_FLIP:
                out.roll  = in.roll;
                out.pitch = -in.pitch;
                out.yaw   = -in.yaw;
                break;

            case SensorAlignment::CW270_DEG_FLIP:
                out.roll  = -in.pitch;
                out.pitch = -in.roll;
                out.yaw   = -in.yaw;
                break;

            default:
                out = in;
                break;
        }

        return out;
    }

    // -------------------------------------------------------------------------
    // Custom Board Alignment Rotation (Decidegrees Roll, Pitch, Yaw)
    // -------------------------------------------------------------------------
    [[nodiscard]] static flight::Axis3f align_custom(
        const flight::Axis3f& in,
        int16_t roll_decideg,
        int16_t pitch_decideg,
        int16_t yaw_decideg) noexcept
    {
        if (roll_decideg == 0 && pitch_decideg == 0 && yaw_decideg == 0) {
            return in;
        }

        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        const float r = static_cast<float>(roll_decideg) * 0.1f * DEG_TO_RAD;
        const float p = static_cast<float>(pitch_decideg) * 0.1f * DEG_TO_RAD;
        const float y = static_cast<float>(yaw_decideg) * 0.1f * DEG_TO_RAD;

        const float cr = std::cos(r), sr = std::sin(r);
        const float cp = std::cos(p), sp = std::sin(p);
        const float cy = std::cos(y), sy = std::sin(y);

        // Rotation matrix elements R = Rz(yaw) * Ry(pitch) * Rx(roll)
        const float r11 = cy * cp;
        const float r12 = cy * sp * sr - sy * cr;
        const float r13 = cy * sp * cr + sy * sr;

        const float r21 = sy * cp;
        const float r22 = sy * sp * sr + cy * cr;
        const float r23 = sy * sp * cr - cy * sr;

        const float r31 = -sp;
        const float r32 = cp * sr;
        const float r33 = cp * cr;

        flight::Axis3f out{};
        out.roll  = r11 * in.roll + r12 * in.pitch + r13 * in.yaw;
        out.pitch = r21 * in.roll + r22 * in.pitch + r23 * in.yaw;
        out.yaw   = r31 * in.roll + r32 * in.pitch + r33 * in.yaw;

        return out;
    }
};

} // namespace abstractx::sensors

#endif // SENSORS_SENSOR_ALIGNMENT_HPP
