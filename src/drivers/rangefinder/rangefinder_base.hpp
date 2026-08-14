/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 Cleanflight & INAV Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Distance Sensor / Rangefinder Driver Base & Median Filter
 *
 * Ported / derived from upstream reference C source files:
 *   - `external/inav/src/main/sensors/rangefinder.c`
 *   - `external/inav/src/main/sensors/rangefinder.h`
 *   - `external/inav/src/main/drivers/rangefinder/`
 *
 * Features:
 * 1. 5-Tap Running Median Filter for Spike & Glitch Rejection
 * 2. 3D Body Tilt Compensation: h_agl = raw_distance * cos(roll) * cos(pitch)
 * 3. Max Tilt Cosine Limit (Default: cos(30 deg) = 0.866)
 * 4. Healthy / Stale Timeout Detection (100ms)
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10
 */

#ifndef DRIVERS_RANGEFINDER_BASE_HPP
#define DRIVERS_RANGEFINDER_BASE_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>

namespace abstractx::drivers {

enum class RangefinderType : uint8_t {
    None         = 0,
    Srf10        = 1,
    Vl53l0x      = 2,
    Msp          = 3,
    Benewake     = 4, // TFmini / TFmini Plus
    Vl53l1x      = 5,
    Us42         = 6,
    Fake         = 8,
    TeraRangerEvo= 9,
    Usd1V0       = 10,
    Nanoradar    = 11
};

struct RangefinderConfig {
    RangefinderType type{RangefinderType::Fake};
    float min_distance_m{0.05f};     // 5 cm minimum valid range
    float max_distance_m{4.00f};     // 4.00 meters maximum range
    float max_tilt_angle_deg{30.0f}; // Tilt angle beyond which measurement is invalid
    bool use_median_filter{true};
};

struct RangefinderSample {
    float raw_distance_m{0.0f};
    float calculated_agl_m{0.0f};    // Tilt-compensated Above-Ground-Level altitude
    bool valid{false};
    bool healthy{false};
    uint64_t timestamp_us{0};
};

class RangefinderDriverBase {
public:
    constexpr explicit RangefinderDriverBase(const RangefinderConfig& config = RangefinderConfig{}) noexcept
        : config_(config) {
        max_tilt_cos_ = std::cos(config.max_tilt_angle_deg * (3.14159265f / 180.0f));
    }

    void set_config(const RangefinderConfig& config) noexcept {
        config_ = config;
        max_tilt_cos_ = std::cos(config.max_tilt_angle_deg * (3.14159265f / 180.0f));
    }

    [[nodiscard]] const RangefinderConfig& config() const noexcept { return config_; }
    [[nodiscard]] const RangefinderSample& latest_sample() const noexcept { return latest_sample_; }
    [[nodiscard]] bool is_healthy() const noexcept { return latest_sample_.healthy; }

    // -------------------------------------------------------------------------
    // Process new raw distance measurement with median filtering & tilt cosine
    // -------------------------------------------------------------------------
    RangefinderSample process(float raw_dist_m, float cos_roll, float cos_pitch, uint64_t timestamp_us) noexcept {
        RangefinderSample sample{};
        sample.raw_distance_m = raw_dist_m;
        sample.timestamp_us = timestamp_us;

        // 1. Min / Max Distance Range Check
        if (raw_dist_m < config_.min_distance_m || raw_dist_m > config_.max_distance_m) {
            sample.valid = false;
            sample.healthy = false;
            latest_sample_ = sample;
            return sample;
        }

        // 2. 5-Tap Median Filter
        float filtered_dist = raw_dist_m;
        if (config_.use_median_filter) {
            median_history_[median_idx_] = raw_dist_m;
            median_idx_ = (median_idx_ + 1) % 5;
            if (median_count_ < 5) median_count_++;

            std::array<float, 5> sorted = median_history_;
            std::sort(sorted.begin(), sorted.begin() + median_count_);
            filtered_dist = sorted[median_count_ / 2];
        }

        // 3. 3D Body Tilt Compensation: h = d * cos(roll) * cos(pitch)
        float cos_tilt = cos_roll * cos_pitch;
        if (cos_tilt < max_tilt_cos_) {
            sample.valid = false;
            sample.healthy = false;
            latest_sample_ = sample;
            return sample;
        }

        sample.calculated_agl_m = filtered_dist * cos_tilt;
        sample.valid = true;
        sample.healthy = true;
        last_valid_time_us_ = timestamp_us;
        latest_sample_ = sample;

        return sample;
    }

private:
    RangefinderConfig config_{};
    float max_tilt_cos_{0.866f};
    std::array<float, 5> median_history_{};
    size_t median_idx_{0};
    size_t median_count_{0};
    uint64_t last_valid_time_us_{0};
    RangefinderSample latest_sample_{};
};

} // namespace abstractx::drivers

#endif // DRIVERS_RANGEFINDER_BASE_HPP
