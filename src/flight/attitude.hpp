/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 Complementary IMU Sensor Fusion Filter
 */

#ifndef FLIGHT_ATTITUDE_HPP
#define FLIGHT_ATTITUDE_HPP

#include <cmath>
#include <array>

namespace abstractx::flight {

struct AttitudeAngles {
    float roll_deg{0.0f};  // Roll (-180 to +180 deg)
    float pitch_deg{0.0f}; // Pitch (-90 to +90 deg)
    float yaw_deg{0.0f};   // Yaw (0 to 360 deg)
};

class AttitudeFilter {
public:
    constexpr AttitudeFilter() noexcept = default;

    void reset() noexcept {
        angles_ = AttitudeAngles{};
    }

    // Update attitude estimation using Accel (g) and Gyro (deg/s) with dt (seconds)
    AttitudeAngles update(const std::array<float, 3>& accel, 
                          const std::array<float, 3>& gyro, 
                          float dt) noexcept {
        // Calculate Roll and Pitch from Accelerometer
        const float accel_roll = std::atan2(accel[1], accel[2]) * (180.0f / 3.14159265f);
        const float accel_pitch = std::atan2(-accel[0], std::sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * (180.0f / 3.14159265f);

        // Complementary Filter: 98% Gyro integration + 2% Accel correction
        constexpr float alpha = 0.98f;
        angles_.roll_deg = alpha * (angles_.roll_deg + gyro[0] * dt) + (1.0f - alpha) * accel_roll;
        angles_.pitch_deg = alpha * (angles_.pitch_deg + gyro[1] * dt) + (1.0f - alpha) * accel_pitch;
        angles_.yaw_deg += gyro[2] * dt;

        if (angles_.yaw_deg >= 360.0f) angles_.yaw_deg -= 360.0f;
        if (angles_.yaw_deg < 0.0f) angles_.yaw_deg += 360.0f;

        return angles_;
    }

    constexpr const AttitudeAngles& angles() const noexcept { return angles_; }

private:
    AttitudeAngles angles_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_ATTITUDE_HPP
