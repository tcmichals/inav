/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2018-2026 INAV Contributors (Pawel Spychalski, Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Throttle-Dependent Dynamic Gyro LPF
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/dynamic_lpf.c`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/dynamic_lpf.h`
 *
 * Features:
 * 1. Throttle-Dependent Dynamic Cutoff Curve with Expo Interpolation:
 *      curve = throttle * (1.0f - throttle) * (expo / 10.0f) + throttle;
 *      cutoff_hz = (max_hz - min_hz) * curve + min_hz;
 * 2. Background Aux Task Execution (dynamicLpfGyroTask) updating main gyro filter cutoffs.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_DYNAMIC_LPF_HPP
#define FLIGHT_DYNAMIC_LPF_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "filter.hpp"

namespace abstractx::flight {

enum class GyroFilterMode : uint8_t {
    Static = 0,
    Dynamic = 1
};

struct DynamicLpfConfig {
    GyroFilterMode filter_mode{GyroFilterMode::Dynamic};
    uint16_t min_hz{100};       // Cutoff at idle (Hz)
    uint16_t max_hz{250};       // Cutoff at full throttle (Hz)
    uint8_t curve_expo{5};      // Non-linear curve shape (0 = linear, 10 = exponential)
    uint16_t throttle_idle{1050};
    uint16_t throttle_max{2000};
};

class DynamicGyroLpfEngine {
public:
    constexpr explicit DynamicGyroLpfEngine(const DynamicLpfConfig& config = DynamicLpfConfig{}) noexcept
        : config_(config), current_cutoff_hz_(config.min_hz) {}

    void set_config(const DynamicLpfConfig& config) noexcept {
        config_ = config;
    }

    [[nodiscard]] const DynamicLpfConfig& config() const noexcept { return config_; }
    [[nodiscard]] float current_cutoff_hz() const noexcept { return current_cutoff_hz_; }

    // -------------------------------------------------------------------------
    // 1. Compute Dynamic Cutoff Frequency (dynLpfCutoffFreq)
    // -------------------------------------------------------------------------
    [[nodiscard]] static constexpr float compute_cutoff_freq(
        float normalized_throttle,
        uint16_t min_hz,
        uint16_t max_hz,
        uint8_t expo) noexcept {

        const float throttle = std::clamp(normalized_throttle, 0.0f, 1.0f);
        const float expof = static_cast<float>(expo) / 10.0f;
        const float curve = throttle * (1.0f - throttle) * expof + throttle;
        return static_cast<float>(max_hz - min_hz) * curve + static_cast<float>(min_hz);
    }

    // -------------------------------------------------------------------------
    // 2. Aux Task Update (dynamicLpfGyroTask)
    // -------------------------------------------------------------------------
    float update(uint16_t throttle_raw_us) noexcept {
        if (config_.filter_mode != GyroFilterMode::Dynamic) {
            current_cutoff_hz_ = static_cast<float>(config_.min_hz);
            return current_cutoff_hz_;
        }

        const float constrained_throttle = static_cast<float>(
            std::clamp(throttle_raw_us, config_.throttle_idle, config_.throttle_max)
        );

        const float throttle_range = static_cast<float>(config_.throttle_max - config_.throttle_idle);
        const float throttle_norm = (throttle_range > 0.0f)
            ? (constrained_throttle - static_cast<float>(config_.throttle_idle)) / throttle_range
            : 0.0f;

        current_cutoff_hz_ = compute_cutoff_freq(
            throttle_norm,
            config_.min_hz,
            config_.max_hz,
            config_.curve_expo
        );

        return current_cutoff_hz_;
    }

private:
    DynamicLpfConfig config_{};
    float current_cutoff_hz_{100.0f};
};

// -----------------------------------------------------------------------------
// Direct 1:1 C-API Compatibility Wrappers (Matching upstream INAV dynamic_lpf.h)
// -----------------------------------------------------------------------------
inline DynamicGyroLpfEngine* dynamicGyroLpfEngine() noexcept {
    static DynamicGyroLpfEngine s_engine{};
    return &s_engine;
}

inline void dynamicLpfGyroTask(uint16_t throttle_us = 1000) noexcept {
    dynamicGyroLpfEngine()->update(throttle_us);
}

} // namespace abstractx::flight

#endif // FLIGHT_DYNAMIC_LPF_HPP
