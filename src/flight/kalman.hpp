/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2018-2026 Betaflight Contributors (Fedecopter, BorisB, et al.)
 * Copyright (C) 2018-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Production C++20 Zero-Allocation Gyro Kalman Filter
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/flight/kalman.c
 *   - Upstream INAV: src/main/flight/kalman.c
 */

#ifndef FLIGHT_KALMAN_HPP
#define FLIGHT_KALMAN_HPP


#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>

namespace abstractx::flight {

// -----------------------------------------------------------------------------
// 1. Single-Axis Dynamic Kalman Filter (1D Scalar State-Space)
// -----------------------------------------------------------------------------
class KalmanFilter1D {
public:
    constexpr KalmanFilter1D() noexcept = default;

    constexpr void reset(float initial_val = 0.0f) noexcept {
        x_est_ = initial_val;
        p_cov_ = 1.0f;
    }

    constexpr void configure(float q_process_noise, float r_measurement_noise) noexcept {
        q_ = (q_process_noise > 0.0f) ? q_process_noise : 100.0f;
        r_ = (r_measurement_noise > 0.0f) ? r_measurement_noise : 80.0f;
    }

    // Update step executed at loop rate (1kHz - 8kHz)
    constexpr float update(float measurement, float dt) noexcept {
        // 1. State Prediction
        const float x_pred = x_est_;
        const float p_pred = p_cov_ + q_ * (dt > 0.0f ? dt : 0.001f);

        // 2. Innovation & Measurement Update
        const float innovation = measurement - x_pred;
        const float k_gain = p_pred / (p_pred + r_);

        // 3. State & Covariance Correction
        x_est_ = x_pred + k_gain * innovation;
        p_cov_ = (1.0f - k_gain) * p_pred;

        return x_est_;
    }

    constexpr float state() const noexcept { return x_est_; }
    constexpr float covariance() const noexcept { return p_cov_; }

private:
    float x_est_{0.0f}; // Estimated state
    float p_cov_{1.0f}; // Estimation error covariance
    float q_{100.0f};   // Process noise variance
    float r_{80.0f};    // Measurement noise variance
};

// -----------------------------------------------------------------------------
// 2. 3-Axis Gyro Kalman Filter (Roll, Pitch, Yaw)
// -----------------------------------------------------------------------------
class GyroKalman3Axis {
public:
    constexpr GyroKalman3Axis() noexcept = default;

    constexpr void reset() noexcept {
        for (auto& axis : filters_) {
            axis.reset(0.0f);
        }
    }

    constexpr void configure(float q_noise, float r_noise) noexcept {
        for (auto& axis : filters_) {
            axis.configure(q_noise, r_noise);
        }
    }

    std::array<float, 3> update(const std::array<float, 3>& raw_gyro_deg_s, float dt) noexcept {
        return {
            filters_[0].update(raw_gyro_deg_s[0], dt),
            filters_[1].update(raw_gyro_deg_s[1], dt),
            filters_[2].update(raw_gyro_deg_s[2], dt)
        };
    }

    constexpr const std::array<KalmanFilter1D, 3>& filters() const noexcept {
        return filters_;
    }

private:
    std::array<KalmanFilter1D, 3> filters_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_KALMAN_HPP
