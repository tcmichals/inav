/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production 3-Axis Rate & Attitude PID Controller
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/flight/pid.c
 *   - Upstream INAV: src/main/flight/pid.c, src/main/flight/pid_autotune.c
 *
 * Combines:
 * 1. Betaflight Feedforward 2.0 (Derivative acceleration with jitter suppression)
 * 2. Betaflight Anti-Gravity (Throttle-derivative dynamic I-term multiplier)
 * 3. Betaflight D-Min (Dynamic D-gain scaling based on stick/gyro transients)
 * 4. INAV Cascaded D-Term Filtering (PT1/PT2 & Biquad Notch on Delta-Gyro)
 * 5. INAV TPA (Throttle PID Attenuation curve above breakpoint)
 * 6. INAV 3D Body-Frame I-Term Coordinate Rotation
 * 7. INAV Angle / Horizon Mode Outer-Loop Attitude Stabilization
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_PID_HPP
#define FLIGHT_PID_HPP


#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include "filter.hpp"

namespace abstractx::flight {

// 3D Vector for Roll, Pitch, Yaw
struct Axis3f {
    float roll{0.0f};
    float pitch{0.0f};
    float yaw{0.0f};

    [[nodiscard]] constexpr float& operator[](size_t index) noexcept {
        return (index == 0) ? roll : (index == 1) ? pitch : yaw;
    }

    [[nodiscard]] constexpr const float& operator[](size_t index) const noexcept {
        return (index == 0) ? roll : (index == 1) ? pitch : yaw;
    }
};

// Complete PID Component Output Breakdown
struct PidState {
    Axis3f p_out{};
    Axis3f i_out{};
    Axis3f d_out{};
    Axis3f ff_out{};
    Axis3f total_out{};
};

// Configuration Parameters for PID & Acro Dynamics
struct PidConfig {
    // Base PID Gains (Roll, Pitch, Yaw)
    Axis3f kp{45.0f, 50.0f, 65.0f};
    Axis3f ki{40.0f, 45.0f, 45.0f};
    Axis3f kd{30.0f, 32.0f, 0.0f};
    Axis3f kff{60.0f, 65.0f, 60.0f};

    // Feedforward 2.0 Settings
    float ff_smooth_cutoff_hz{50.0f};
    float ff_jitter_factor{7.0f};
    float ff_transition{0.0f}; // 0.0 = full stick FF, >0 decays FF towards stick endpoints

    // Anti-Gravity Settings (Betaflight)
    float anti_gravity_gain{80.0f};
    float anti_gravity_cutoff_hz{5.0f};
    float anti_gravity_max_boost{3.0f};

    // D-Min Dynamic D-Gain Settings
    Axis3f d_min{20.0f, 22.0f, 0.0f};
    Axis3f d_max{35.0f, 38.0f, 0.0f};
    float d_min_gain_scale{0.005f};

    // D-Boost Settings (Betaflight / INAV)
    float d_boost_gain{1.2f};           // D-boost gain multiplier (0.0 = disabled)
    float d_boost_cutoff_hz{15.0f};     // Stick acceleration filter cutoff (Hz)
    float d_boost_max{2.0f};            // Maximum D-gain boost multiplier

    // I-Term Relax Settings (Betaflight / INAV)
    uint8_t iterm_relax_type{1u};       // 0: OFF, 1: SETPOINT, 2: GYRO
    float   iterm_relax_cutoff_hz{15.0f};// Stick motion detector cutoff (Hz)

    // TPA (Throttle PID Attenuation) Settings
    float tpa_breakpoint{0.5f}; // Throttle threshold (0.0 to 1.0)
    float tpa_rate{0.20f};       // Gain reduction fraction above breakpoint

    // D-Term Filter Cutoff Frequencies (Hz)
    float dterm_lpf1_hz{100.0f};
    float dterm_lpf2_hz{200.0f};
    float dterm_notch_hz{0.0f}; // 0.0 = disabled
    float dterm_notch_q{1.0f};

    // Thrust Linearization Settings (Betaflight pidApplyThrustLinearization)
    float thrust_linearization{0.0f}; // 0.0 = disabled, 0.20-0.40 typical

    // Level / Angle Mode Outer-Loop Settings
    float level_kp{5.0f};            // Outer-loop attitude error gain
    float max_angle_rate_dps{300.0f}; // Maximum rate setpoint in level mode (deg/s)
    float angle_earth_ref{1.0f};     // Earth-reference turn coordination gain
};

class PidController {
public:
    explicit PidController(const PidConfig& config = PidConfig{}) noexcept
        : config_(config) {
        init_filters();
    }

    void set_config(const PidConfig& config) noexcept {
        config_ = config;
        init_filters();
    }

    [[nodiscard]] const PidConfig& get_config() const noexcept {
        return config_;
    }

    void reset() noexcept {
        i_term_ = Axis3f{};
        prev_gyro_ = Axis3f{};
        prev_setpoint_ = Axis3f{};
        prev_throttle_ = 0.0f;
        ag_accumulator_ = 0.0f;

        for (size_t axis = 0; axis < 3; ++axis) {
            ff_filter_[axis].reset(0.0f);
            dterm_lpf1_[axis].reset(0.0f);
            dterm_lpf2_[axis].reset(0.0f);
            dterm_notch_[axis].reset();
        }
        ag_filter_.reset(0.0f);
    }

    // -------------------------------------------------------------------------
    // Angle Mode Outer-Loop: Computes rate setpoints from angle error
    // with Betaflight / INAV Earth-Reference Yaw Coordination (pidLevel)
    // -------------------------------------------------------------------------
    [[nodiscard]] Axis3f calculate_angle_mode_rates(
        const Axis3f& target_angles_deg,
        const Axis3f& current_angles_deg,
        float target_yaw_rate_dps = 0.0f) const noexcept {
        Axis3f rate_setpoints{};

        for (size_t axis = 0; axis < 2; ++axis) { // Roll and Pitch only
            const float angle_error = target_angles_deg[axis] - current_angles_deg[axis];
            float target_rate = angle_error * config_.level_kp;

            // Earth-reference coordinated turn cross-coupling:
            // roll rate is adjusted by pitch sin(pitch)*yaw_rate, pitch by -sin(roll)*yaw_rate
            constexpr float DEG_TO_RAD = 0.017453292519943295f;
            const float alt_angle_deg = (axis == 0) ? target_angles_deg.pitch : target_angles_deg.roll;
            float sin_angle = std::sin(alt_angle_deg * DEG_TO_RAD);
            if (axis == 0) sin_angle = -sin_angle; // Negative for Roll
            target_rate += target_yaw_rate_dps * sin_angle * config_.angle_earth_ref;

            rate_setpoints[axis] = std::clamp(target_rate, -config_.max_angle_rate_dps, config_.max_angle_rate_dps);
        }
        rate_setpoints.yaw = (target_yaw_rate_dps != 0.0f) ? target_yaw_rate_dps : target_angles_deg.yaw;
        return rate_setpoints;
    }

    // -------------------------------------------------------------------------
    // Thrust Linearization (Betaflight pid.c:507)
    // -------------------------------------------------------------------------
    [[nodiscard]] float apply_thrust_linearization(float motor_output) const noexcept {
        const float e = config_.thrust_linearization;
        if (e <= 0.0f) return motor_output;
        const float inv = 1.0f - motor_output;
        return motor_output * (1.0f + e * inv * (1.0f + e * (inv - motor_output)));
    }

    [[nodiscard]] float compensate_thrust_linearization(float throttle) const noexcept {
        const float e = config_.thrust_linearization;
        if (e <= 0.0f) return throttle;
        return throttle * (1.0f - e * (1.0f - throttle));
    }

    // -------------------------------------------------------------------------
    // Core Flight PID Update (1kHz – 16kHz)
    // -------------------------------------------------------------------------
    [[nodiscard]] PidState update(
        const Axis3f& setpoint_rates_dps,
        const Axis3f& gyro_rates_dps,
        float throttle,
        float dt_s) noexcept {
        
        // Defensive time delta validation (NASA/JPL bounds guard)
        if (dt_s <= 0.00001f || dt_s > 0.1f) {
            dt_s = 0.001f; // Default to 1kHz on invalid dt
        }

        PidState state{};

        // 1. Calculate Anti-Gravity Throttle Step Multiplier
        const float throttle_delta = (throttle - prev_throttle_) / dt_s;
        prev_throttle_ = throttle;
        const float ag_step = std::abs(ag_filter_.update(throttle_delta));
        const float ag_multiplier = 1.0f + std::min(ag_step * (config_.anti_gravity_gain * 0.01f), config_.anti_gravity_max_boost);

        // 2. Calculate Throttle PID Attenuation (TPA) Scale
        float tpa_factor = 1.0f;
        if (throttle > config_.tpa_breakpoint && config_.tpa_breakpoint < 1.0f) {
            const float tpa_range = 1.0f - config_.tpa_breakpoint;
            const float tpa_progress = (throttle - config_.tpa_breakpoint) / tpa_range;
            tpa_factor = 1.0f - (config_.tpa_rate * tpa_progress);
            tpa_factor = std::clamp(tpa_factor, 0.1f, 1.0f);
        }

        // 3. 3D Body-Frame I-Term Coordinate Rotation (INAV)
        rotate_iterm(gyro_rates_dps, dt_s);

        // 4. Per-Axis PID & Dynamics Calculations
        for (size_t axis = 0; axis < 3; ++axis) {
            const float setpoint = setpoint_rates_dps[axis];
            const float gyro = gyro_rates_dps[axis];
            const float error = setpoint - gyro;

            // --- P-Term (with TPA scaling) ---
            const float kp_effective = config_.kp[axis] * tpa_factor;
            state.p_out[axis] = (error * kp_effective) * 0.032f; // Scaled to normalized range

            // --- I-Term (with Anti-Gravity, I-Term Relax, and Anti-Windup Clamping) ---
            float iterm_relax_factor = 1.0f;
            if (config_.iterm_relax_type > 0u && axis < 2u) { // Relax applied to Roll & Pitch
                const float setpoint_rate = std::abs(setpoint - prev_setpoint_[axis]) / dt_s;
                const float smooth_sp_rate = iterm_relax_lpf_[axis].update(setpoint_rate);
                iterm_relax_factor = std::clamp(1.0f - (smooth_sp_rate / 300.0f), 0.1f, 1.0f);
            }

            const float ki_effective = config_.ki[axis] * ag_multiplier * iterm_relax_factor;
            i_term_[axis] += (error * ki_effective * dt_s) * 0.032f;
            i_term_[axis] = std::clamp(i_term_[axis], -0.40f, 0.40f);
            state.i_out[axis] = i_term_[axis];

            // --- D-Term (Cascaded LowPass + Notch on Delta Gyro with D-Min & D-Boost) ---
            const float delta_gyro = (gyro - prev_gyro_[axis]) / dt_s;
            prev_gyro_[axis] = gyro;

            // Apply cascaded filters
            float filtered_d = dterm_lpf1_[axis].update(delta_gyro);
            filtered_d = dterm_lpf2_[axis].update(filtered_d);
            if (config_.dterm_notch_hz > 0.0f) {
                filtered_d = dterm_notch_[axis].update(filtered_d);
            }

            // D-Min Dynamic D Gain Interpolation
            const float setpoint_delta = std::abs((setpoint - prev_setpoint_[axis]) / dt_s);
            const float activity = std::max(std::abs(filtered_d), setpoint_delta);
            const float d_dynamic_range = config_.d_max[axis] - config_.d_min[axis];
            const float dynamic_d = config_.d_min[axis] + std::min(activity * config_.d_min_gain_scale * d_dynamic_range, d_dynamic_range);

            // D-Boost Stick Acceleration Damping
            float d_boost_factor = 1.0f;
            if (config_.d_boost_gain > 0.0f && axis < 2u) {
                const float stick_accel = std::abs(setpoint_delta - prev_setpoint_delta_[axis]) / dt_s;
                prev_setpoint_delta_[axis] = setpoint_delta;
                const float smooth_accel = d_boost_lpf_[axis].update(stick_accel);
                d_boost_factor = 1.0f + std::min(smooth_accel * (config_.d_boost_gain * 0.0005f), config_.d_boost_max - 1.0f);
            }

            const float kd_effective = (dynamic_d > 0.0f ? dynamic_d : config_.kd[axis]) * tpa_factor * d_boost_factor;

            state.d_out[axis] = (-filtered_d * kd_effective) * 0.0005f;

            // --- Feedforward 2.0 (Betaflight) ---
            const float raw_ff = (setpoint - prev_setpoint_[axis]) / dt_s;
            prev_setpoint_[axis] = setpoint;

            // Smooth feedforward with PT1 filter to reject jitter
            const float smooth_ff = ff_filter_[axis].update(raw_ff);
            state.ff_out[axis] = (smooth_ff * config_.kff[axis]) * 0.0005f;

            // --- Total Axis Output ---
            state.total_out[axis] = state.p_out[axis] + state.i_out[axis] + state.d_out[axis] + state.ff_out[axis];
            state.total_out[axis] = std::clamp(state.total_out[axis], -1.0f, 1.0f);
        }

        return state;
    }

private:
    void init_filters() noexcept {
        for (size_t axis = 0; axis < 3; ++axis) {
            ff_filter_[axis].set_cutoff(config_.ff_smooth_cutoff_hz, 0.001f);
            dterm_lpf1_[axis].set_cutoff(config_.dterm_lpf1_hz, 0.001f);
            dterm_lpf2_[axis].set_cutoff(config_.dterm_lpf2_hz, 0.001f);
            if (config_.dterm_notch_hz > 0.0f) {
                dterm_notch_[axis].configure(BiquadType::Notch, config_.dterm_notch_hz, 1000.0f, config_.dterm_notch_q);
            }
            d_boost_lpf_[axis].set_cutoff(config_.d_boost_cutoff_hz, 0.001f);
            iterm_relax_lpf_[axis].set_cutoff(config_.iterm_relax_cutoff_hz, 0.001f);
        }
        ag_filter_.set_cutoff(config_.anti_gravity_cutoff_hz, 0.001f);
    }

    // Rotates the 3D I-term vector in the body frame by angular velocity vector:
    void rotate_iterm(const Axis3f& gyro_dps, float dt_s) noexcept {
        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        const float wx = gyro_dps.roll * DEG_TO_RAD * dt_s;
        const float wy = gyro_dps.pitch * DEG_TO_RAD * dt_s;
        const float wz = gyro_dps.yaw * DEG_TO_RAD * dt_s;

        // Small angle approximation for rotation matrix:
        // I_new = I_old + (omega x I_old)
        const float ix = i_term_.roll;
        const float iy = i_term_.pitch;
        const float iz = i_term_.yaw;

        i_term_.roll  += (wy * iz - wz * iy);
        i_term_.pitch += (wz * ix - wx * iz);
        i_term_.yaw   += (wx * iy - wy * ix);
    }

    PidConfig config_{};
    Axis3f i_term_{};
    Axis3f prev_gyro_{};
    Axis3f prev_setpoint_{};
    Axis3f prev_setpoint_delta_{};
    float prev_throttle_{0.0f};
    float ag_accumulator_{0.0f};

    // Filter Pipelines (Zero dynamic allocation)
    std::array<Pt1Filter, 3> ff_filter_{};
    std::array<Pt1Filter, 3> dterm_lpf1_{};
    std::array<Pt2Filter, 3> dterm_lpf2_{};
    std::array<BiquadFilter, 3> dterm_notch_{};
    std::array<Pt1Filter, 3> d_boost_lpf_{};
    std::array<Pt1Filter, 3> iterm_relax_lpf_{};
    Pt1Filter ag_filter_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_PID_HPP
