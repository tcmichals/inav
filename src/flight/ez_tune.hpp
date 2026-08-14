/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2020-2026 INAV Contributors (Konstantin Sharlaimov, Pawel Spychalski, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production EZ-Tune Macro Preset Engine
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/ez_tune.c
 *
 * Features:
 * 1. 3 High-Level Intuitive Sliders: Response, Damping, Tracking (0.5x to 2.0x).
 * 2. Harmonic Synthesis into 18 Low-Level PID, Feedforward, Anti-Gravity & Filter Gains.
 * 3. Pre-Tuned Airframe Templates (5" Freestyle, 7" Long Range, Cinewhoop, Flying Wing).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_EZ_TUNE_HPP
#define FLIGHT_EZ_TUNE_HPP


#include <cmath>
#include <algorithm>
#include "pid.hpp"

namespace abstractx::flight {

struct EzTuneSliders {
    float response{1.0f}; // 0.5 (gentle/smooth) to 2.0 (crisp/robotic)
    float damping{1.0f};  // 0.5 (loose/cool motors) to 2.0 (tight/no overshoot)
    float tracking{1.0f}; // 0.5 (relaxed) to 2.0 (locked-in wind rejection)
};

enum class EzAirframePreset : uint8_t {
    Freestyle5Inch = 0,
    LongRange7Inch = 1,
    Cinewhoop3Inch = 2,
    FixedWingFlying = 3
};

class EzTuneEngine {
public:
    [[nodiscard]] static constexpr PidConfig synthesize(
        const EzTuneSliders& sliders,
        EzAirframePreset preset = EzAirframePreset::Freestyle5Inch) noexcept {

        const float resp = std::clamp(sliders.response, 0.5f, 2.0f);
        const float damp = std::clamp(sliders.damping, 0.5f, 2.0f);
        const float track = std::clamp(sliders.tracking, 0.5f, 2.0f);

        PidConfig base_cfg = get_base_preset(preset);

        PidConfig cfg = base_cfg;

        // 1. Synthesize P, I, D Gains
        for (size_t i = 0; i < 3; ++i) {
            // P-Gain scales with response and slightly inversely with damping
            cfg.kp[i] = base_cfg.kp[i] * resp * (1.0f / std::sqrt(damp));

            // I-Gain scales with tracking and response
            cfg.ki[i] = base_cfg.ki[i] * track * resp;

            // D-Gain scales with damping and response
            cfg.kd[i] = base_cfg.kd[i] * damp * resp;

            // Feedforward scales quadratically with response
            cfg.kff[i] = base_cfg.kff[i] * (resp * resp);

            // D-Min dynamic range scales with damping
            cfg.d_min[i] = base_cfg.d_min[i] * damp;
            cfg.d_max[i] = base_cfg.d_max[i] * damp;
        }

        // 2. Synthesize Anti-Gravity & Filter Cutoffs
        cfg.anti_gravity_gain = base_cfg.anti_gravity_gain * track;
        cfg.dterm_lpf1_hz = base_cfg.dterm_lpf1_hz * std::sqrt(damp);
        cfg.dterm_lpf2_hz = base_cfg.dterm_lpf2_hz * std::sqrt(damp);

        return cfg;
    }

    [[nodiscard]] static constexpr PidConfig get_base_preset(EzAirframePreset preset) noexcept {
        PidConfig cfg{};

        switch (preset) {
            case EzAirframePreset::Freestyle5Inch:
                cfg.kp = Axis3f{45.0f, 50.0f, 65.0f};
                cfg.ki = Axis3f{40.0f, 45.0f, 45.0f};
                cfg.kd = Axis3f{30.0f, 32.0f, 0.0f};
                cfg.kff = Axis3f{60.0f, 65.0f, 60.0f};
                cfg.anti_gravity_gain = 80.0f;
                cfg.dterm_lpf1_hz = 100.0f;
                cfg.dterm_lpf2_hz = 200.0f;
                break;

            case EzAirframePreset::LongRange7Inch:
                cfg.kp = Axis3f{35.0f, 38.0f, 50.0f};
                cfg.ki = Axis3f{30.0f, 35.0f, 35.0f};
                cfg.kd = Axis3f{22.0f, 24.0f, 0.0f};
                cfg.kff = Axis3f{40.0f, 45.0f, 40.0f};
                cfg.anti_gravity_gain = 50.0f;
                cfg.dterm_lpf1_hz = 70.0f;
                cfg.dterm_lpf2_hz = 140.0f;
                break;

            case EzAirframePreset::Cinewhoop3Inch:
                cfg.kp = Axis3f{55.0f, 60.0f, 75.0f};
                cfg.ki = Axis3f{50.0f, 55.0f, 55.0f};
                cfg.kd = Axis3f{35.0f, 38.0f, 0.0f};
                cfg.kff = Axis3f{70.0f, 75.0f, 70.0f};
                cfg.anti_gravity_gain = 100.0f;
                cfg.dterm_lpf1_hz = 110.0f;
                cfg.dterm_lpf2_hz = 220.0f;
                break;

            case EzAirframePreset::FixedWingFlying:
                cfg.kp = Axis3f{25.0f, 30.0f, 40.0f};
                cfg.ki = Axis3f{15.0f, 20.0f, 20.0f};
                cfg.kd = Axis3f{10.0f, 12.0f, 0.0f};
                cfg.kff = Axis3f{30.0f, 30.0f, 0.0f};
                cfg.anti_gravity_gain = 0.0f;
                cfg.dterm_lpf1_hz = 50.0f;
                cfg.dterm_lpf2_hz = 100.0f;
                break;
        }

        return cfg;
    }
};

} // namespace abstractx::flight

#endif // FLIGHT_EZ_TUNE_HPP
