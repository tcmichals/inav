/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2018-2026 INAV Contributors (Konstantin Sharlaimov et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Optical Flow Sensor & Ground Velocity Estimator
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `external/inav/src/main/sensors/opflow.c`
 *   - `external/inav/src/main/sensors/opflow.h`
 *   - `external/inav/src/main/drivers/opflow/`
 *
 * Mathematical Operation:
 *   1. Integrates raw pixel displacement (dx, dy) over time delta dt.
 *   2. Applies board alignment sensor transformation matrix.
 *   3. Gyro Body Rate Compensation: Subtracts angular rates (omega_x, omega_y)
 *      so pure roll/pitch rotational maneuvers do not trigger false optical flow.
 *   4. Rangefinder Altitude Fusion: Computes true ground velocity:
 *      v_ground = (flow_rate - body_rate) * h_agl
 *   5. Surface Quality Gating: Dual-threshold hysteresis (High >= 35, Low <= 10).
 *
 * Standards: MISRA C++:2023, DO-178C, NASA/JPL Power of 10 (Zero dynamic heap allocations, [[nodiscard]], noexcept)
 */

#ifndef SENSORS_OPTICAL_FLOW_HPP
#define SENSORS_OPTICAL_FLOW_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "sensors/sensor_alignment.hpp"

namespace abstractx::sensors {

enum class OpflowQualityState : uint8_t {
    Invalid = 0,
    Valid   = 1
};

struct OpflowConfig {
    float scale{100.0f};                          // Sensor scaling factor (pixels to rad/s)
    SensorAlignment alignment{SensorAlignment::CW0_DEG}; // Optical flow camera mounting alignment
    uint8_t squal_threshold_high{35u};            // Surface quality threshold to become VALID
    uint8_t squal_threshold_low{10u};             // Surface quality threshold to drop to INVALID
    uint32_t timeout_us{200000u};                 // 200 ms timeout (at least 5 Hz updates)
};

struct OpflowSample {
    float flow_rate_rad_s[2]{0.0f, 0.0f};         // Uncompensated optical flow rate (X, Y) [rad/s]
    float body_rate_rad_s[2]{0.0f, 0.0f};         // Gyro angular rate integrated over flow interval [rad/s]
    float comp_flow_rad_s[2]{0.0f, 0.0f};         // Pure translational flow (flow - body) [rad/s]
    float vel_ground_m_s[2]{0.0f, 0.0f};          // Ground velocity (comp_flow * altitude) [m/s]
    uint8_t surface_quality{0u};                  // Surface quality metric (0..255)
    bool is_valid{false};                         // True when surface quality and update rate meet limits
    uint64_t timestamp_ns{0u};                    // Hardware timestamp
};

class OpticalFlowEstimator {
public:
    constexpr OpticalFlowEstimator() noexcept = default;

    void init(const OpflowConfig& config = OpflowConfig{}) noexcept {
        config_ = config;
        reset();
    }

    void reset() noexcept {
        quality_state_ = OpflowQualityState::Invalid;
        is_healthy_ = false;
        last_update_us_ = 0u;
        gyro_accum_deg_[0] = 0.0f;
        gyro_accum_deg_[1] = 0.0f;
        gyro_accum_time_us_ = 0u;
        latest_sample_ = OpflowSample{};
    }

    // Accumulate gyro readings between optical flow bursts for body rate compensation
    void accumulate_gyro(float gyro_roll_dps, float gyro_pitch_dps, uint32_t dt_us) noexcept {
        if (!is_healthy_) return;
        gyro_accum_deg_[0] += gyro_roll_dps * static_cast<float>(dt_us);
        gyro_accum_deg_[1] += gyro_pitch_dps * static_cast<float>(dt_us);
        gyro_accum_time_us_ += dt_us;
    }

    // Process new raw optical flow sample from camera / ISP hardware
    [[nodiscard]] OpflowSample update(int16_t raw_dx,
                                      int16_t raw_dy,
                                      uint8_t quality,
                                      uint32_t delta_time_us,
                                      float altitude_agl_m,
                                      uint32_t current_time_us,
                                      uint64_t timestamp_ns = 0u) noexcept {
        if (delta_time_us == 0u) {
            return latest_sample_;
        }

        is_healthy_ = true;
        last_update_us_ = current_time_us;
        latest_sample_.surface_quality = quality;
        latest_sample_.timestamp_ns = timestamp_ns;

        // Quality Hysteresis (INAV opflow.c:179-191)
        switch (quality_state_) {
        case OpflowQualityState::Invalid:
            if (quality >= config_.squal_threshold_high) {
                quality_state_ = OpflowQualityState::Valid;
            }
            break;
        case OpflowQualityState::Valid:
            if (quality <= config_.squal_threshold_low) {
                quality_state_ = OpflowQualityState::Invalid;
            }
            break;
        }

        latest_sample_.is_valid = (quality_state_ == OpflowQualityState::Valid);

        // Calculate average body angular rate over the optical flow integration interval
        float body_rate_dps[2]{0.0f, 0.0f};
        if (gyro_accum_time_us_ > 0u) {
            body_rate_dps[0] = gyro_accum_deg_[0] / static_cast<float>(gyro_accum_time_us_);
            body_rate_dps[1] = gyro_accum_deg_[1] / static_cast<float>(gyro_accum_time_us_);
        }

        float flow_rate_dps[2]{0.0f, 0.0f};
        if (latest_sample_.is_valid && config_.scale > 0.01f) {
            // Apply mounting alignment to raw (dx, dy)
            flight::Axis3f raw_axis{static_cast<float>(raw_dx), static_cast<float>(raw_dy), 0.0f};
            flight::Axis3f aligned = SensorAlignmentEngine::align_standard(raw_axis, config_.alignment);

            // Integral to rate scaler (INAV opflow.c:208)
            const float integral_to_rate_scaler = (1.0e6f / static_cast<float>(delta_time_us)) / config_.scale;
            flow_rate_dps[0] = aligned.roll * integral_to_rate_scaler;
            flow_rate_dps[1] = aligned.pitch * integral_to_rate_scaler;
        }

        // Convert degrees to radians (INAV opflow.c:247-250)
        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        latest_sample_.body_rate_rad_s[0] = body_rate_dps[0] * DEG_TO_RAD;
        latest_sample_.body_rate_rad_s[1] = body_rate_dps[1] * DEG_TO_RAD;
        latest_sample_.flow_rate_rad_s[0] = flow_rate_dps[0] * DEG_TO_RAD;
        latest_sample_.flow_rate_rad_s[1] = flow_rate_dps[1] * DEG_TO_RAD;

        // Subtract body rotation rate from optical flow (Translational Flow = Flow - Gyro)
        latest_sample_.comp_flow_rad_s[0] = latest_sample_.flow_rate_rad_s[0] - latest_sample_.body_rate_rad_s[0];
        latest_sample_.comp_flow_rad_s[1] = latest_sample_.flow_rate_rad_s[1] - latest_sample_.body_rate_rad_s[1];

        // Compute ground velocity: v = omega_trans * height (clamped positive AGL)
        const float effective_alt_m = std::max(0.05f, altitude_agl_m);
        if (latest_sample_.is_valid && altitude_agl_m > 0.1f) {
            latest_sample_.vel_ground_m_s[0] = latest_sample_.comp_flow_rad_s[0] * effective_alt_m;
            latest_sample_.vel_ground_m_s[1] = latest_sample_.comp_flow_rad_s[1] * effective_alt_m;
        } else {
            latest_sample_.vel_ground_m_s[0] = 0.0f;
            latest_sample_.vel_ground_m_s[1] = 0.0f;
        }

        // Reset gyro accumulators for next integration window
        gyro_accum_deg_[0] = 0.0f;
        gyro_accum_deg_[1] = 0.0f;
        gyro_accum_time_us_ = 0u;

        return latest_sample_;
    }

    // Timeout watchdog check (INAV opflow.c:257-270)
    void check_timeout(uint32_t current_time_us) noexcept {
        if (is_healthy_ && ((current_time_us - last_update_us_) > config_.timeout_us)) {
            is_healthy_ = false;
            quality_state_ = OpflowQualityState::Invalid;
            latest_sample_.is_valid = false;
            latest_sample_.surface_quality = 0u;
            latest_sample_.flow_rate_rad_s[0] = 0.0f;
            latest_sample_.flow_rate_rad_s[1] = 0.0f;
            latest_sample_.vel_ground_m_s[0] = 0.0f;
            latest_sample_.vel_ground_m_s[1] = 0.0f;
        }
    }

    // Parse 64-Byte TLP packet (Optical Flow payload format)
    [[nodiscard]] static OpflowSample parse_tlp(const Tlp64& tlp, float altitude_m = 1.0f) noexcept {
        OpflowSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;
        int16_t dx = static_cast<int16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8u));
        int16_t dy = static_cast<int16_t>(p[2] | (static_cast<uint16_t>(p[3]) << 8u));
        uint8_t quality = p[4];

        constexpr float SCALE = 100.0f;
        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        sample.flow_rate_rad_s[0] = (static_cast<float>(dx) / SCALE) * DEG_TO_RAD;
        sample.flow_rate_rad_s[1] = (static_cast<float>(dy) / SCALE) * DEG_TO_RAD;
        sample.surface_quality = quality;
        sample.is_valid = (quality >= 35u);

        sample.vel_ground_m_s[0] = sample.flow_rate_rad_s[0] * altitude_m;
        sample.vel_ground_m_s[1] = sample.flow_rate_rad_s[1] * altitude_m;

        return sample;
    }

    [[nodiscard]] bool is_healthy() const noexcept { return is_healthy_; }
    [[nodiscard]] const OpflowSample& latest_sample() const noexcept { return latest_sample_; }

private:
    OpflowConfig config_{};
    OpflowQualityState quality_state_{OpflowQualityState::Invalid};
    bool is_healthy_{false};
    uint32_t last_update_us_{0u};

    float gyro_accum_deg_[2]{0.0f, 0.0f};
    uint32_t gyro_accum_time_us_{0u};

    OpflowSample latest_sample_{};
};

} // namespace abstractx::sensors

#endif // SENSORS_OPTICAL_FLOW_HPP
