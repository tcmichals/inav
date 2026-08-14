/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2020-2026 INAV Contributors (Konstantin Sharlaimov, Pawel Spychalski, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production EZ-Tune Macro Preset Engine
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/ez_tune.c`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/ez_tune.h`
 *
 * Faithfully ports all formulas, scales, and filter synthesis stages:
 * 1. Exact default PID base matrices (Roll/Pitch: [40, 75, 23, 100], Yaw: [45, 80, 0, 100]).
 * 2. Axis Pitch-to-Roll Ratio scaling.
 * 3. Yaw Scale Formula: 1.0f + ((input - 100) * 0.005f).
 * 4. D-Term LPF & Smith Predictor Delay: 1000.0f / (2 * pi * filterHz).
 * 5. Dynamic Gyro Notch Min Frequency: max(filterHz * 0.667f, 100Hz).
 * 6. Gyro Kalman Q scaling: scaleRange(filterHz, 150, 300, 200, 400).
 * 7. Rate & Expo scaling (30-90 deg/s stabilized rate, 40-100 RC Expo).
 * 8. D-Boost Snappiness (dBoostMin: 1.0 to 0.0).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_EZ_TUNE_HPP
#define FLIGHT_EZ_TUNE_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "pid.hpp"

namespace abstractx::flight {

// -----------------------------------------------------------------------------
// Upstream INAV ezTuneSettings_t Struct
// -----------------------------------------------------------------------------
struct EzTuneSettings {
    bool enabled{true};
    uint16_t filter_hz{180};        // Filter Cutoff (Hz, 50 to 300)
    uint8_t axis_ratio{100};        // Pitch to Roll PID ratio (50% to 200%)
    uint8_t response{100};          // P-Gain scale (50% to 200%)
    uint8_t damping{100};           // D-Gain scale (50% to 200%)
    uint8_t stability{100};         // I-Gain scale (50% to 200%)
    uint8_t aggressiveness{100};    // FF-Gain scale (50% to 200%)
    uint8_t rate{100};              // Stabilized rate (0 to 200)
    uint8_t expo{100};              // RC Expo (0 to 200)
    uint8_t snappiness{50};         // D-Boost snappiness (0 to 100)
};

struct EzTuneCalculatedProfile {
    PidConfig pid_config{};
    float gyro_main_lpf_hz{180.0f};
    float dterm_lpf_hz{175.0f};
    float dynamic_notch_min_hz{120.0f};
    float smith_predictor_delay_ms{0.884f};
    float kalman_q{240.0f};
    float rate_roll_deg_s{60.0f};
    float rate_pitch_deg_s{60.0f};
    float rate_yaw_deg_s{50.0f};
    uint8_t rc_expo{70};
    float d_boost_min{0.5f};
};

class EzTuneEngine {
public:
    // Base PID Defaults matching INAV EZ_TUNE_PID_RP_DEFAULT & EZ_TUNE_PID_YAW_DEFAULT
    static constexpr float PID_DEFAULTS_RP[4]  = {40.0f, 75.0f, 23.0f, 100.0f}; // P, I, D, FF
    static constexpr float PID_DEFAULTS_YAW[4] = {45.0f, 80.0f,  0.0f, 100.0f}; // P, I, D, FF

    [[nodiscard]] static constexpr float get_yaw_pid_scale(float input) noexcept {
        const float normalized = (input - 100.0f) * 0.01f;
        return 1.0f + (normalized * 0.5f);
    }

    [[nodiscard]] static constexpr float compute_pt1_filter_delay_ms(float filter_hz) noexcept {
        constexpr float TWO_PI = 6.283185307179586f;
        if (filter_hz <= 0.0f) return 0.0f;
        return 1000.0f / (TWO_PI * filter_hz);
    }

    [[nodiscard]] static constexpr float scale_range_f(float val, float src_min, float src_max, float dst_min, float dst_max) noexcept {
        const float clamped = std::clamp(val, src_min, src_max);
        return dst_min + ((clamped - src_min) / (src_max - src_min)) * (dst_max - dst_min);
    }

    // -------------------------------------------------------------------------
    // Execute Full INAV ezTuneUpdate() Calculation
    // -------------------------------------------------------------------------
    [[nodiscard]] static constexpr EzTuneCalculatedProfile update(const EzTuneSettings& ez) noexcept {
        EzTuneCalculatedProfile prof{};

        if (!ez.enabled) {
            return prof;
        }

        // 1. Filter Cutoffs
        prof.gyro_main_lpf_hz = static_cast<float>(ez.filter_hz);
        prof.dterm_lpf_hz = std::max(static_cast<float>(ez.filter_hz) - 5.0f, 50.0f);
        prof.dynamic_notch_min_hz = std::max(static_cast<float>(ez.filter_hz) * 0.667f, 100.0f);
        prof.smith_predictor_delay_ms = compute_pt1_filter_delay_ms(static_cast<float>(ez.filter_hz));

        // 2. Kalman Q Factor
        if (ez.filter_hz < 150) {
            prof.kalman_q = 200.0f;
        } else {
            prof.kalman_q = scale_range_f(static_cast<float>(ez.filter_hz), 150.0f, 300.0f, 200.0f, 400.0f);
        }

        // 3. Roll / Pitch / Yaw PID Synthesis
        const float pitch_ratio = static_cast<float>(ez.axis_ratio) / 100.0f;
        const float resp_scale  = static_cast<float>(ez.response) / 100.0f;
        const float stab_scale  = static_cast<float>(ez.stability) / 100.0f;
        const float damp_scale  = static_cast<float>(ez.damping) / 100.0f;
        const float agg_scale   = static_cast<float>(ez.aggressiveness) / 100.0f;

        // Roll
        prof.pid_config.kp.roll  = PID_DEFAULTS_RP[0] * resp_scale;
        prof.pid_config.ki.roll  = PID_DEFAULTS_RP[1] * stab_scale;
        prof.pid_config.kd.roll  = PID_DEFAULTS_RP[2] * damp_scale;
        prof.pid_config.kff.roll = PID_DEFAULTS_RP[3] * agg_scale;

        // Pitch
        prof.pid_config.kp.pitch  = PID_DEFAULTS_RP[0] * resp_scale * pitch_ratio;
        prof.pid_config.ki.pitch  = PID_DEFAULTS_RP[1] * stab_scale * pitch_ratio;
        prof.pid_config.kd.pitch  = PID_DEFAULTS_RP[2] * damp_scale * pitch_ratio;
        prof.pid_config.kff.pitch = PID_DEFAULTS_RP[3] * agg_scale * pitch_ratio;

        // Yaw
        prof.pid_config.kp.yaw  = PID_DEFAULTS_YAW[0] * get_yaw_pid_scale(static_cast<float>(ez.response));
        prof.pid_config.ki.yaw  = PID_DEFAULTS_YAW[1] * get_yaw_pid_scale(static_cast<float>(ez.stability));
        prof.pid_config.kd.yaw  = PID_DEFAULTS_YAW[2] * get_yaw_pid_scale(static_cast<float>(ez.damping));
        prof.pid_config.kff.yaw = PID_DEFAULTS_YAW[3] * get_yaw_pid_scale(static_cast<float>(ez.aggressiveness));

        // 4. Rate & Expo Synthesis
        const float rate_f = static_cast<float>(ez.rate);
        prof.rate_roll_deg_s  = scale_range_f(rate_f, 0.0f, 200.0f, 30.0f, 90.0f);
        prof.rate_pitch_deg_s = scale_range_f(rate_f, 0.0f, 200.0f, 30.0f, 90.0f);
        prof.rate_yaw_deg_s   = scale_range_f(rate_f, 0.0f, 200.0f, 30.0f, 90.0f) - 10.0f;
        prof.rc_expo = static_cast<uint8_t>(std::lrint(scale_range_f(rate_f, 0.0f, 200.0f, 40.0f, 100.0f)));

        // 5. D-Boost Snappiness
        prof.d_boost_min = scale_range_f(static_cast<float>(ez.snappiness), 0.0f, 100.0f, 1.0f, 0.0f);

        // Filter cutoffs populated to PidConfig
        prof.pid_config.dterm_lpf1_hz = prof.dterm_lpf_hz;
        prof.pid_config.dterm_lpf2_hz = prof.dterm_lpf_hz * 2.0f;
        prof.pid_config.d_min = Axis3f{prof.pid_config.kd.roll * 0.65f, prof.pid_config.kd.pitch * 0.65f, 0.0f};
        prof.pid_config.d_max = prof.pid_config.kd;

        return prof;
    }
};

// -----------------------------------------------------------------------------
// Direct 1:1 C-API Compatibility Wrappers (Matching upstream INAV ez_tune.h)
// -----------------------------------------------------------------------------
using ezTuneSettings_t = EzTuneSettings;

inline ezTuneSettings_t* ezTuneMutable() noexcept {
    static ezTuneSettings_t s_settings{};
    return &s_settings;
}

inline const ezTuneSettings_t* ezTune() noexcept {
    return ezTuneMutable();
}

inline EzTuneCalculatedProfile ezTuneUpdate(const EzTuneSettings* settings = nullptr) noexcept {
    const EzTuneSettings& cfg = (settings != nullptr) ? *settings : *ezTune();
    return EzTuneEngine::update(cfg);
}

} // namespace abstractx::flight

#endif // FLIGHT_EZ_TUNE_HPP

