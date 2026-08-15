/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2019-2026 INAV Contributors (Pawel Spychalski, Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Matrix Smith Predictor
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `external/inav/src/main/flight/smith_predictor.c`
 *   - `external/inav/src/main/flight/smith_predictor.h`
 *
 * Mathematical Operation:
 *   Compensates for sensor filtering group delay and motor latency on Gyro Rate feedback:
 *     1. Simulates airframe/filter delay model: model_state = PT1(gyro_input, filter_hz, dt)
 *     2. Circular delay line buffer of length L = round(delay_ms / (1000 * dt))
 *     3. Lead compensation calculation:
 *        compensated_gyro = gyro_input + strength * (model_state - delayed_model_sample)
 *
 * Adheres to MISRA C++:2023 & NASA/JPL Power of 10 principles:
 *   - Zero dynamic allocation (Fixed array ring buffer: MAX_DELAY_SAMPLES = 64)
 *   - Fixed-width types, [[nodiscard]], const noexcept, bounded WCET.
 */

#ifndef FLIGHT_SMITH_PREDICTOR_HPP
#define FLIGHT_SMITH_PREDICTOR_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <span>
#include "filter.hpp"

namespace abstractx::flight {

struct SmithPredictorConfig {
    bool     enabled{true};
    float    delay_ms{0.884f};     // Delay time (ms) to compensate
    float    filter_hz{180.0f};    // Model PT1 cutoff frequency (Hz)
    float    strength{1.0f};       // Compensation strength (0.0 to 1.0)
};

class SmithPredictor {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 64u;

    constexpr SmithPredictor() noexcept = default;

    void init(const SmithPredictorConfig& cfg, float looptime_s) noexcept {
        cfg_ = cfg;
        looptime_s_ = looptime_s;

        if (looptime_s > 0.0f && cfg.delay_ms > 0.0f) {
            float samples_f = (cfg.delay_ms * 0.001f) / looptime_s;
            delay_samples_ = std::clamp(static_cast<uint32_t>(std::round(samples_f)), 1u, static_cast<uint32_t>(MAX_DELAY_SAMPLES - 1u));
        } else {
            delay_samples_ = 1u;
        }

        // Initialize PT1 model filters for 3 axes
        for (size_t axis = 0; axis < 3; ++axis) {
            model_filter_[axis].set_cutoff(cfg_.filter_hz, looptime_s_);
            delay_line_[axis].fill(0.0f);
        }
        write_idx_ = 0u;
    }

    void reset() noexcept {
        for (size_t axis = 0; axis < 3; ++axis) {
            model_filter_[axis].reset();
            delay_line_[axis].fill(0.0f);
        }
        write_idx_ = 0u;
    }

    // -------------------------------------------------------------------------
    // Update Smith Predictor with current raw gyro rates (deg/s)
    // -------------------------------------------------------------------------
    [[nodiscard]] Axis3f update(const Axis3f& gyro_in) noexcept {
        if (!cfg_.enabled || delay_samples_ == 0u) {
            return gyro_in;
        }

        Axis3f out{};

        // Calculate read index from circular delay line
        uint32_t read_idx = (write_idx_ + MAX_DELAY_SAMPLES - delay_samples_) % MAX_DELAY_SAMPLES;

        const float in_arr[3] = {gyro_in.roll, gyro_in.pitch, gyro_in.yaw};
        float out_arr[3] = {0.0f, 0.0f, 0.0f};

        for (size_t axis = 0; axis < 3; ++axis) {
            // 1. Update internal PT1 model of filter delay
            float model_val = model_filter_[axis].update(in_arr[axis]);

            // 2. Fetch delayed model sample
            float delayed_val = delay_line_[axis][read_idx];

            // 3. Store current model sample into ring buffer
            delay_line_[axis][write_idx_] = model_val;

            // 4. Calculate phase lead prediction: y_hat = y + strength * (model - delayed_model)
            float lead = model_val - delayed_val;
            out_arr[axis] = in_arr[axis] + (cfg_.strength * lead);
        }

        // Advance circular write index
        write_idx_ = (write_idx_ + 1u) % MAX_DELAY_SAMPLES;

        out.roll  = out_arr[0];
        out.pitch = out_arr[1];
        out.yaw   = out_arr[2];

        return out;
    }

    [[nodiscard]] uint32_t delay_samples() const noexcept { return delay_samples_; }
    [[nodiscard]] const SmithPredictorConfig& config() const noexcept { return cfg_; }

private:
    SmithPredictorConfig cfg_{};
    float looptime_s_{0.001f};
    uint32_t delay_samples_{1u};
    uint32_t write_idx_{0u};

    std::array<Pt1Filter, 3u> model_filter_{};
    std::array<std::array<float, MAX_DELAY_SAMPLES>, 3u> delay_line_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_SMITH_PREDICTOR_HPP
