/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Production C++20 Zero-Allocation Cascaded Sensor Filters
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/common/filter.c
 *   - Upstream Betaflight: src/main/common/filter.c
 */

#ifndef FLIGHT_FILTER_HPP
#define FLIGHT_FILTER_HPP


#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>

namespace abstractx::flight {

// Constants
inline constexpr float PI_F = 3.14159265358979323846f;

// -----------------------------------------------------------------------------
// 1. First-Order Low-Pass Filter (PT1)
// -----------------------------------------------------------------------------
class Pt1Filter {
public:
    constexpr Pt1Filter() noexcept = default;

    constexpr void reset(float initial_value = 0.0f) noexcept {
        state_ = initial_value;
    }


    void set_cutoff(float cutoff_hz, float dt) noexcept {
        if (cutoff_hz <= 0.0f || dt <= 0.0f) {
            k_ = 1.0f; // Pass-through
            return;
        }
        const float rc = 1.0f / (2.0f * PI_F * cutoff_hz);
        k_ = dt / (dt + rc);
        k_ = std::clamp(k_, 0.0f, 1.0f);
    }

    constexpr float update(float input) noexcept {
        state_ += k_ * (input - state_);
        return state_;
    }

    float apply(float input, float cutoff_hz, float dt) noexcept {
        set_cutoff(cutoff_hz, dt);
        return update(input);
    }

    constexpr float state() const noexcept { return state_; }

private:
    float state_{0.0f};
    float k_{1.0f};
};


// -----------------------------------------------------------------------------
// 2. Second-Order Low-Pass Filter (PT2 - Cascaded 2x PT1)
// -----------------------------------------------------------------------------
class Pt2Filter {
public:
    constexpr Pt2Filter() noexcept = default;

    constexpr void reset(float initial_value = 0.0f) noexcept {
        stage1_.reset(initial_value);
        stage2_.reset(initial_value);
    }

    void set_cutoff(float cutoff_hz, float dt) noexcept {
        stage1_.set_cutoff(cutoff_hz, dt);
        stage2_.set_cutoff(cutoff_hz, dt);
    }

    constexpr float update(float input) noexcept {
        return stage2_.update(stage1_.update(input));
    }

    constexpr float state() const noexcept { return stage2_.state(); }

private:
    Pt1Filter stage1_{};
    Pt1Filter stage2_{};
};

// -----------------------------------------------------------------------------
// 3. Third-Order Low-Pass Filter (PT3 - Cascaded 3x PT1)
// -----------------------------------------------------------------------------
class Pt3Filter {
public:
    constexpr Pt3Filter() noexcept = default;

    constexpr void reset(float initial_value = 0.0f) noexcept {
        stage1_.reset(initial_value);
        stage2_.reset(initial_value);
        stage3_.reset(initial_value);
    }

    void set_cutoff(float cutoff_hz, float dt) noexcept {
        stage1_.set_cutoff(cutoff_hz, dt);
        stage2_.set_cutoff(cutoff_hz, dt);
        stage3_.set_cutoff(cutoff_hz, dt);
    }

    constexpr float update(float input) noexcept {
        return stage3_.update(stage2_.update(stage1_.update(input)));
    }

    constexpr float state() const noexcept { return stage3_.state(); }

private:
    Pt1Filter stage1_{};
    Pt1Filter stage2_{};
    Pt1Filter stage3_{};
};

// -----------------------------------------------------------------------------
// 4. Biquad Direct Form II Transposed Filter (LowPass & Notch)
// -----------------------------------------------------------------------------
enum class BiquadType : uint8_t {
    LowPass,
    Notch
};

class BiquadFilter {
public:
    constexpr BiquadFilter() noexcept = default;

    constexpr void reset() noexcept {
        x1_ = 0.0f;
        x2_ = 0.0f;
    }

    void set_notch(float center_freq_hz, float q, float dt_s) noexcept {
        const float sample_rate_hz = (dt_s > 0.0f) ? (1.0f / dt_s) : 1000.0f;
        configure(BiquadType::Notch, center_freq_hz, sample_rate_hz, q);
    }

    void configure(BiquadType type, float center_freq_hz, float sample_rate_hz, float q = 0.7071f) noexcept {
        if (sample_rate_hz <= 0.0f || center_freq_hz <= 0.0f || center_freq_hz >= (sample_rate_hz / 2.0f)) {
            // Passthrough configuration
            b0_ = 1.0f; b1_ = 0.0f; b2_ = 0.0f;
            a1_ = 0.0f; a2_ = 0.0f;
            return;
        }


        const float omega = 2.0f * PI_F * center_freq_hz / sample_rate_hz;
        const float sn = std::sin(omega);
        const float cs = std::cos(omega);
        const float alpha = sn / (2.0f * q);

        float a0 = 1.0f;

        if (type == BiquadType::LowPass) {
            b0_ = (1.0f - cs) * 0.5f;
            b1_ = 1.0f - cs;
            b2_ = (1.0f - cs) * 0.5f;
            a0 = 1.0f + alpha;
            a1_ = -2.0f * cs;
            a2_ = 1.0f - alpha;
        } else { // Notch
            b0_ = 1.0f;
            b1_ = -2.0f * cs;
            b2_ = 1.0f;
            a0 = 1.0f + alpha;
            a1_ = -2.0f * cs;
            a2_ = 1.0f - alpha;
        }

        // Normalize coefficients by a0
        const float inv_a0 = 1.0f / a0;
        b0_ *= inv_a0;
        b1_ *= inv_a0;
        b2_ *= inv_a0;
        a1_ *= inv_a0;
        a2_ *= inv_a0;
    }

    constexpr float update(float input) noexcept {
        const float output = b0_ * input + x1_;
        x1_ = b1_ * input - a1_ * output + x2_;
        x2_ = b2_ * input - a2_ * output;
        return output;
    }

private:
    float b0_{1.0f}, b1_{0.0f}, b2_{0.0f};
    float a1_{0.0f}, a2_{0.0f};
    float x1_{0.0f}, x2_{0.0f};
};

// -----------------------------------------------------------------------------
// 5. Rate Slew Limiter
// -----------------------------------------------------------------------------
class SlewLimiter {
public:
    constexpr SlewLimiter() noexcept = default;

    constexpr void reset(float initial_value = 0.0f) noexcept {
        state_ = initial_value;
    }

    constexpr float update(float input, float max_rate_per_sec, float dt) noexcept {
        if (dt <= 0.0f || max_rate_per_sec <= 0.0f) {
            state_ = input;
            return state_;
        }
        const float max_delta = max_rate_per_sec * dt;
        const float delta = input - state_;
        if (delta > max_delta) {
            state_ += max_delta;
        } else if (delta < -max_delta) {
            state_ -= max_delta;
        } else {
            state_ = input;
        }
        return state_;
    }

    constexpr float state() const noexcept { return state_; }

private:
    float state_{0.0f};
};

} // namespace abstractx::flight

#endif // FLIGHT_FILTER_HPP
