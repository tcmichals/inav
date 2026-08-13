/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 Zero-Allocation Extended Kalman Filter 3 (EKF3)
 */

#ifndef FLIGHT_EKF3_HPP
#define FLIGHT_EKF3_HPP

#include "imu_pcie_driver.hpp"
#include "attitude.hpp"
#include <cstdint>
#include <cmath>
#include <array>

namespace abstractx::flight {

struct EkfState {
    std::array<float, 3> pos_ned_cm{0.0f, 0.0f, 0.0f};  // North, East, Down (cm)
    std::array<float, 3> vel_ned_cms{0.0f, 0.0f, 0.0f}; // North, East, Down (cm/s)
    AttitudeAngles attitude{};                          // Roll, Pitch, Yaw (deg)
    std::array<float, 3> gyro_bias{0.0f, 0.0f, 0.0f};   // Estimated Gyro Biases
    uint64_t last_update_ns{0};
    bool is_healthy{true};
};

class Ekf3Filter {
public:
    constexpr Ekf3Filter() noexcept = default;

    void reset() noexcept {
        state_ = EkfState{};
        attitude_filter_.reset();
    }

    // Time Predict Step driven by 64B IMU TLP & 64-bit nanosecond hardware timestamps
    void predict_imu(const drivers::ImuSample& sample) noexcept {
        if (state_.last_update_ns == 0) {
            state_.last_update_ns = sample.timestamp_ns;
            return;
        }

        // Calculate dt in seconds from hardware nanosecond timestamps (Zero IRQ jitter!)
        float dt = static_cast<float>(sample.timestamp_ns - state_.last_update_ns) * 1e-9f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.001f; // Fallback bound
        state_.last_update_ns = sample.timestamp_ns;

        // Apply Gyro Bias Correction
        std::array<float, 3> corrected_gyro{
            sample.gyro_deg_s[0] - state_.gyro_bias[0],
            sample.gyro_deg_s[1] - state_.gyro_bias[1],
            sample.gyro_deg_s[2] - state_.gyro_bias[2]
        };

        // Update Attitude Estimation
        state_.attitude = attitude_filter_.update(sample.accel_g, corrected_gyro, dt);

        // Predict 3D Velocity & Position (simplifying body-to-earth rotation)
        float roll_rad = state_.attitude.roll_deg * (3.14159265f / 180.0f);
        float pitch_rad = state_.attitude.pitch_deg * (3.14159265f / 180.0f);

        // Accel Earth Frame (g -> cm/s^2)
        float accel_n_cms2 = -std::sin(pitch_rad) * 980.665f;
        float accel_e_cms2 = std::sin(roll_rad) * std::cos(pitch_rad) * 980.665f;
        float accel_d_cms2 = (sample.accel_g[2] - 1.0f) * 980.665f; // Subtract 1G

        // Predict Velocity
        state_.vel_ned_cms[0] += accel_n_cms2 * dt;
        state_.vel_ned_cms[1] += accel_e_cms2 * dt;
        state_.vel_ned_cms[2] += accel_d_cms2 * dt;

        // Predict Position
        state_.pos_ned_cm[0] += state_.vel_ned_cms[0] * dt;
        state_.pos_ned_cm[1] += state_.vel_ned_cms[1] * dt;
        state_.pos_ned_cm[2] += state_.vel_ned_cms[2] * dt;
    }

    // Measurement Correct Step driven by Barometer TLP
    void correct_baro(float baro_alt_cm, uint64_t /*timestamp_ns*/) noexcept {
        constexpr float gain = 0.15f; // Baro Kalman Correction Gain
        float innov = baro_alt_cm - (-state_.pos_ned_cm[2]);
        state_.pos_ned_cm[2] -= gain * innov; // Down position is negative altitude
    }

    // Measurement Correct Step driven by GPS TLP
    void correct_gps(float lat, float lon, float alt_cm, 
                     const std::array<float, 3>& vel_ned_cms, 
                     uint64_t /*timestamp_ns*/) noexcept {
        if (!home_set_) {
            home_lat_ = lat;
            home_lon_ = lon;
            home_set_ = true;
        }

        // Convert Lat/Lon to Local NED Position (cm)
        float pos_n_cm = (lat - home_lat_) * 111319.5f * 100.0f;
        float pos_e_cm = (lon - home_lon_) * 111319.5f * 100.0f;

        constexpr float pos_gain = 0.2f;
        constexpr float vel_gain = 0.3f;

        // Correct Position
        state_.pos_ned_cm[0] += pos_gain * (pos_n_cm - state_.pos_ned_cm[0]);
        state_.pos_ned_cm[1] += pos_gain * (pos_e_cm - state_.pos_ned_cm[1]);
        state_.pos_ned_cm[2] += pos_gain * (-alt_cm - state_.pos_ned_cm[2]);

        // Correct Velocity
        state_.vel_ned_cms[0] += vel_gain * (vel_ned_cms[0] - state_.vel_ned_cms[0]);
        state_.vel_ned_cms[1] += vel_gain * (vel_ned_cms[1] - state_.vel_ned_cms[1]);
        state_.vel_ned_cms[2] += vel_gain * (vel_ned_cms[2] - state_.vel_ned_cms[2]);
    }

    constexpr const EkfState& state() const noexcept { return state_; }

private:
    EkfState state_{};
    AttitudeFilter attitude_filter_{};
    float home_lat_{0.0f};
    float home_lon_{0.0f};
    bool home_set_{false};
};

} // namespace abstractx::flight

#endif // FLIGHT_EKF3_HPP
