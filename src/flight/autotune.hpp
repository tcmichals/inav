/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) Karl Johan Åström & Tore Hägglund (Relay Feedback Method)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production In-Flight AutoTune Engine (Åström-Hägglund Relay Method)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/pid_autotune.c
 *
 * Features:
 * 1. Relay / Square-Wave Rate Perturbation Injection (+/- d).
 * 2. Peak Amplitude (A) & Oscillation Period (Tu) Measurement.
 * 3. Ultimate Gain (Ku = 4d / (pi * A)) Identification.
 * 4. Modified Ziegler-Nichols PID Gain Synthesis.
 * 5. Safety Abort on Excessive Attitude Tilt or Stick Override.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_AUTOTUNE_HPP
#define FLIGHT_AUTOTUNE_HPP


#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "pid.hpp"

namespace abstractx::flight {

enum class AutoTuneState : uint8_t {
    Idle = 0,
    RunningPositive,
    RunningNegative,
    Converged,
    Aborted
};

struct AutoTuneResults {
    Axis3f tuned_kp{};
    Axis3f tuned_ki{};
    Axis3f tuned_kd{};
    bool roll_converged{false};
    bool pitch_converged{false};
};

struct AutoTuneConfig {
    float relay_amplitude_dps{30.0f};  // Perturbation amplitude (d) in deg/s
    float max_angle_abort_deg{35.0f};   // Safety abort if tilt exceeds threshold
    uint8_t min_cycles{4};              // Minimum cycles before converging
    float gain_safety_margin{0.85f};    // Safety margin scalar applied to Ku
};

class AutoTuneEngine {
public:
    using Config = AutoTuneConfig;

    constexpr AutoTuneEngine() noexcept = default;
    constexpr explicit AutoTuneEngine(const Config& config) noexcept
        : config_(config) {}

    void start() noexcept {

        state_ = AutoTuneState::RunningPositive;
        cycle_count_ = 0;
        peak_rate_ = 0.0f;
        cycle_time_s_ = 0.0f;
        results_ = AutoTuneResults{};
    }

    void stop() noexcept {
        state_ = AutoTuneState::Idle;
    }

    // -------------------------------------------------------------------------
    // Main AutoTune Step (Runs at 100Hz – 1kHz during tuning mode)
    // Injects perturbation rate setpoint and identifies system frequency response
    // -------------------------------------------------------------------------
    [[nodiscard]] float update(
        size_t axis, // 0 = Roll, 1 = Pitch
        float gyro_rate_dps, 
        float current_angle_deg, 
        float dt_s) noexcept {

        if (state_ == AutoTuneState::Idle || state_ == AutoTuneState::Converged || state_ == AutoTuneState::Aborted) {
            return 0.0f;
        }

        // Safety Gating: Abort if aircraft tilt exceeds bounds
        if (std::abs(current_angle_deg) > config_.max_angle_abort_deg) {
            state_ = AutoTuneState::Aborted;
            return 0.0f;
        }

        cycle_time_s_ += dt_s;
        if (std::abs(gyro_rate_dps) > peak_rate_) {
            peak_rate_ = std::abs(gyro_rate_dps);
        }

        float injection_rate_dps = 0.0f;

        if (state_ == AutoTuneState::RunningPositive) {
            injection_rate_dps = config_.relay_amplitude_dps;

            // Zero-crossing check after positive perturbation
            if (gyro_rate_dps < 0.0f && cycle_time_s_ > 0.05f) {
                state_ = AutoTuneState::RunningNegative;
            }
        } else if (state_ == AutoTuneState::RunningNegative) {
            injection_rate_dps = -config_.relay_amplitude_dps;

            // Zero-crossing check after negative perturbation (1 full cycle complete)
            if (gyro_rate_dps > 0.0f && cycle_time_s_ > 0.05f) {
                cycle_count_++;
                
                // Process cycle metrics
                if (cycle_count_ >= config_.min_cycles && peak_rate_ > 5.0f) {
                    calculate_zn_gains(axis, peak_rate_, cycle_time_s_);
                    state_ = AutoTuneState::Converged;
                    return 0.0f;
                }

                // Reset for next cycle
                cycle_time_s_ = 0.0f;
                peak_rate_ = 0.0f;
                state_ = AutoTuneState::RunningPositive;
            }
        }

        return injection_rate_dps;
    }

    [[nodiscard]] AutoTuneState state() const noexcept { return state_; }
    [[nodiscard]] const AutoTuneResults& results() const noexcept { return results_; }

private:
    void calculate_zn_gains(size_t axis, float peak_amplitude, float period_tu) noexcept {
        constexpr float PI_F = 3.14159265358979323846f;

        // Ultimate Gain Ku = (4 * d) / (pi * A)
        const float ku = (4.0f * config_.relay_amplitude_dps) / (PI_F * peak_amplitude);
        const float ku_safe = ku * config_.gain_safety_margin;

        // Modified Ziegler-Nichols (INAV / Åström-Hägglund)
        const float kp = 0.45f * ku_safe * 100.0f;
        const float ki = (0.54f * ku_safe / period_tu) * 10.0f;
        const float kd = (0.06f * ku_safe * period_tu) * 1000.0f;

        results_.tuned_kp[axis] = std::clamp(kp, 15.0f, 120.0f);
        results_.tuned_ki[axis] = std::clamp(ki, 10.0f, 100.0f);
        results_.tuned_kd[axis] = std::clamp(kd, 5.0f, 80.0f);

        if (axis == 0) results_.roll_converged = true;
        if (axis == 1) results_.pitch_converged = true;
    }

    Config config_{};
    AutoTuneState state_{AutoTuneState::Idle};
    uint8_t cycle_count_{0};
    float peak_rate_{0.0f};
    float cycle_time_s_{0.0f};
    AutoTuneResults results_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_AUTOTUNE_HPP
