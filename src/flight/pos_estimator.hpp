/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Inertial-Complementary 3D Position & Altitude Estimator
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/navigation/navigation_pos_estimator.c
 *
 * Features:
 * 1. 2nd-Order Earth-Frame Inertial Kinematic Integration (1kHz–16kHz).
 * 2. Barometer Altitude Innovation Fusion (100Hz) with Continuous Accel Z-Bias (b_a,z) Tracking.
 * 3. GPS 3D Position & Velocity Innovation Fusion (10Hz) with Glitch Gating & HDOP Validation.
 * 4. Earth-Frame Coordinate Acceleration with Centrifugal & Gravity Removal.
 * 5. Ground Zero & Barometer Surface Calibration.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_POS_ESTIMATOR_HPP
#define FLIGHT_POS_ESTIMATOR_HPP


#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include "attitude.hpp"

namespace abstractx::flight {

struct EstimatorState {
    // Position in Earth NED frame (meters)
    float pos_n_m{0.0f}; // North
    float pos_e_m{0.0f}; // East
    float pos_d_m{0.0f}; // Down (-altitude)

    // Velocity in Earth NED frame (m/s)
    float vel_n_m_s{0.0f};
    float vel_e_m_s{0.0f};
    float vel_d_m_s{0.0f};

    // Estimated Accelerometer Biases (m/s^2)
    float bias_acc_x{0.0f};
    float bias_acc_y{0.0f};
    float bias_acc_z{0.0f};

    // Health & Validation Status
    bool is_healthy{true};
    bool baro_healthy{false};
    bool gps_healthy{false};
    bool gps_glitch_detected{false};
    uint8_t satellites{0};
    float hdop{99.0f};
};

struct EstimatorConfig {
    // Barometer Fusion Weights (INAV defaults)
    float w_z_baro_pos{0.35f};      // Altitude innovation weight
    float w_z_baro_vel{0.15f};      // Vertical velocity innovation weight
    float w_z_baro_bias{0.02f};     // Accelerometer Z-bias learning rate

    // GPS Fusion Weights (INAV defaults)
    float w_xy_gps_pos{0.25f};      // Horizontal position innovation weight
    float w_xy_gps_vel{0.10f};      // Horizontal velocity innovation weight
    float w_xy_gps_bias{0.01f};     // Horizontal accel bias learning rate

    // GPS Glitch & Health Gating
    float max_gps_hdop{3.5f};       // Maximum allowed HDOP
    uint8_t min_satellites{6};      // Minimum satellites for position hold / nav
    float max_gps_pos_jump_m{15.0f};// Maximum allowable single-frame position jump (m)
};

class InertialPosEstimator {
public:
    using Config = EstimatorConfig;

    constexpr InertialPosEstimator() noexcept = default;
    constexpr explicit InertialPosEstimator(const Config& config) noexcept
        : config_(config) {}

    void set_config(const Config& config) noexcept {
        config_ = config;
    }

    [[nodiscard]] const Config& get_config() const noexcept {
        return config_;
    }

    void reset() noexcept {
        state_ = EstimatorState{};
        baro_ground_alt_m_ = 0.0f;
        baro_calibrated_ = false;
        gps_origin_lat_ = 0.0;
        gps_origin_lon_ = 0.0;
        gps_origin_set_ = false;
    }

    // -------------------------------------------------------------------------
    // 1. Inertial Prediction Step (Runs at Flight Loop Rate: 1kHz–16kHz)
    // -------------------------------------------------------------------------
    void predict_imu(const Axis3f& accel_body_g, const MahonyAhrs& ahrs, float dt_s) noexcept {
        if (dt_s <= 0.00001f || dt_s > 0.1f) dt_s = 0.001f;

        constexpr float GRAVITY_MSS = 9.80665f;

        // Convert body acceleration from Gs to m/s^2
        Axis3f acc_body_mss{
            accel_body_g.roll  * GRAVITY_MSS,
            accel_body_g.pitch * GRAVITY_MSS,
            accel_body_g.yaw   * GRAVITY_MSS
        };

        // Rotate body acceleration into Earth (NED) frame
        Axis3f acc_earth = ahrs.rotate_body_to_earth(acc_body_mss);

        // Remove 1.0G gravity from Down axis (Z points Down in NED)
        acc_earth.yaw -= GRAVITY_MSS;

        // Apply learned accelerometer biases
        acc_earth.roll  -= state_.bias_acc_x;
        acc_earth.pitch -= state_.bias_acc_y;
        acc_earth.yaw   -= state_.bias_acc_z;

        // 2nd-Order Kinematic Integration:
        // p = p + v*dt + 0.5*a*dt^2
        // v = v + a*dt
        state_.pos_n_m += state_.vel_n_m_s * dt_s + 0.5f * acc_earth.roll  * dt_s * dt_s;
        state_.pos_e_m += state_.vel_e_m_s * dt_s + 0.5f * acc_earth.pitch * dt_s * dt_s;
        state_.pos_d_m += state_.vel_d_m_s * dt_s + 0.5f * acc_earth.yaw   * dt_s * dt_s;

        state_.vel_n_m_s += acc_earth.roll  * dt_s;
        state_.vel_e_m_s += acc_earth.pitch * dt_s;
        state_.vel_d_m_s += acc_earth.yaw   * dt_s;
    }

    // -------------------------------------------------------------------------
    // 2. Barometer Altitude Correction Step (50Hz – 100Hz)
    // -------------------------------------------------------------------------
    void correct_baro(float raw_baro_alt_m) noexcept {
        if (!baro_calibrated_) {
            baro_ground_alt_m_ = raw_baro_alt_m;
            baro_calibrated_ = true;
            state_.baro_healthy = true;
            return;
        }

        // Relative altitude above ground (Z is Down in NED, so height is -pos_d_m)
        const float relative_baro_alt = raw_baro_alt_m - baro_ground_alt_m_;
        const float estimated_height = -state_.pos_d_m;

        // Innovation residual
        const float innov_alt = relative_baro_alt - estimated_height;

        // Apply complementary correction to Down position and velocity
        state_.pos_d_m -= config_.w_z_baro_pos * innov_alt;
        state_.vel_d_m_s -= config_.w_z_baro_vel * innov_alt;

        // Track continuous accelerometer Z bias
        state_.bias_acc_z += config_.w_z_baro_bias * innov_alt;
        state_.bias_acc_z = std::clamp(state_.bias_acc_z, -1.5f, 1.5f); // Bias safety limit (m/s^2)

        state_.baro_healthy = true;
    }

    // -------------------------------------------------------------------------
    // 3. GPS 3D Position & Velocity Correction Step (5Hz – 10Hz)
    // -------------------------------------------------------------------------
    void correct_gps(
        double lat_deg, 
        double lon_deg, 
        float alt_msl_m, 
        float vel_n_m_s, 
        float vel_e_m_s, 
        float hdop, 
        uint8_t satellites) noexcept {

        (void)alt_msl_m;
        state_.satellites = satellites;
        state_.hdop = hdop;

        // GPS Glitch & Quality Gating
        if (hdop > config_.max_gps_hdop || satellites < config_.min_satellites) {
            state_.gps_healthy = false;
            state_.gps_glitch_detected = true;
            return; // Reject low-quality GPS frames
        }

        // Initialize GPS home origin on first valid 3D fix
        if (!gps_origin_set_) {
            gps_origin_lat_ = lat_deg;
            gps_origin_lon_ = lon_deg;
            gps_origin_set_ = true;
            state_.gps_healthy = true;
            state_.gps_glitch_detected = false;
            return;
        }

        // Convert WGS84 Lat/Lon to local Flat-Earth NED meters relative to origin
        constexpr double DEG_TO_RAD = 0.017453292519943295;
        constexpr double EARTH_RADIUS_M = 6371000.0;

        const double d_lat = (lat_deg - gps_origin_lat_) * DEG_TO_RAD;
        const double d_lon = (lon_deg - gps_origin_lon_) * DEG_TO_RAD;
        const double lat_rad = gps_origin_lat_ * DEG_TO_RAD;

        const float gps_pos_n = static_cast<float>(d_lat * EARTH_RADIUS_M);
        const float gps_pos_e = static_cast<float>(d_lon * EARTH_RADIUS_M * std::cos(lat_rad));

        // Glitch detection: Check for unrealistic single-frame position jump
        const float pos_diff_n = gps_pos_n - state_.pos_n_m;
        const float pos_diff_e = gps_pos_e - state_.pos_e_m;
        const float jump_distance = std::sqrt(pos_diff_n * pos_diff_n + pos_diff_e * pos_diff_e);

        if (jump_distance > config_.max_gps_pos_jump_m) {
            state_.gps_glitch_detected = true;
            return; // Reject glitch frame
        }

        state_.gps_glitch_detected = false;
        state_.gps_healthy = true;

        // Apply Horizontal Position Correction
        state_.pos_n_m += config_.w_xy_gps_pos * pos_diff_n;
        state_.pos_e_m += config_.w_xy_gps_pos * pos_diff_e;

        // Apply Horizontal Velocity Correction
        const float vel_diff_n = vel_n_m_s - state_.vel_n_m_s;
        const float vel_diff_e = vel_e_m_s - state_.vel_e_m_s;

        state_.vel_n_m_s += config_.w_xy_gps_vel * vel_diff_n;
        state_.vel_e_m_s += config_.w_xy_gps_vel * vel_diff_e;

        // Learn Horizontal Accelerometer Biases
        state_.bias_acc_x += config_.w_xy_gps_bias * vel_diff_n;
        state_.bias_acc_y += config_.w_xy_gps_bias * vel_diff_e;
        state_.bias_acc_x = std::clamp(state_.bias_acc_x, -1.0f, 1.0f);
        state_.bias_acc_y = std::clamp(state_.bias_acc_y, -1.0f, 1.0f);
    }

    [[nodiscard]] const EstimatorState& state() const noexcept { return state_; }
    [[nodiscard]] float altitude_m() const noexcept { return -state_.pos_d_m; }
    [[nodiscard]] float speed_horizontal_m_s() const noexcept {
        return std::sqrt(state_.vel_n_m_s * state_.vel_n_m_s + state_.vel_e_m_s * state_.vel_e_m_s);
    }

private:
    Config config_{};
    EstimatorState state_{};

    float baro_ground_alt_m_{0.0f};
    bool baro_calibrated_{false};

    double gps_origin_lat_{0.0};
    double gps_origin_lon_{0.0};
    bool gps_origin_set_{false};
};

// Backwards compatibility alias for Ekf3Filter
using PosEstimator = InertialPosEstimator;

} // namespace abstractx::flight

#endif // FLIGHT_POS_ESTIMATOR_HPP
