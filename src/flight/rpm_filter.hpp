/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2019-2026 Cleanflight & INAV Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Exact C++20 Port of INAV Multi-Harmonic DShot RPM Notch Filter Bank
 *
 * Reference Upstream C Source Files:
 *   - `external/inav/src/main/flight/rpm_filter.c`
 *   - `external/inav/src/main/flight/rpm_filter.h`
 *
 * Features:
 * 1. Tracks up to 3 motor harmonics (1st base, 2nd, 3rd harmonic) per motor across Roll, Pitch, Yaw.
 * 2. 150 Hz PT1 filter on raw ESC RPM telemetry to prevent frequency stepping noise.
 * 3. Exact INAV Biquad Notch filter coefficient recalculation:
 *      f_harmonic = constrain(base_freq * (harmonic + 1), min_hz, max_hz)
 * 4. Zero dynamic memory allocation (static compile-time motor and harmonic tables).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10
 */

#ifndef FLIGHT_RPM_FILTER_HPP
#define FLIGHT_RPM_FILTER_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "filter.hpp"

namespace abstractx::flight {

inline constexpr size_t RPM_FILTER_MAX_MOTORS = 8;
inline constexpr size_t RPM_FILTER_HARMONICS  = 3;
inline constexpr float  RPM_TO_HZ             = 1.0f / 60.0f;
inline constexpr float  RPM_LPF_HZ            = 150.0f;

struct RpmFilterConfig {
    bool     gyro_filter_enabled{true};
    uint8_t  gyro_harmonics{3};       // 1..3 harmonics
    uint8_t  gyro_min_hz{100};        // 100 Hz floor
    uint16_t gyro_q{500};             // Q = 5.00 (gyro_q / 100.0)
    uint32_t looptime_us{1000};       // 1000us (1kHz)
};

template <size_t MotorCount = 4>
class DshotRpmFilterBank {
    static_assert(MotorCount <= RPM_FILTER_MAX_MOTORS, "Motor count exceeds maximum supported motors");

public:
    constexpr explicit DshotRpmFilterBank(const RpmFilterConfig& config = RpmFilterConfig{}) noexcept
        : config_(config) {
        init(config);
    }

    void init(const RpmFilterConfig& config) noexcept {
        config_ = config;
        q_ = static_cast<float>(config.gyro_q) / 100.0f;
        min_hz_ = static_cast<float>(config.gyro_min_hz);
        harmonics_ = std::min<uint8_t>(config.gyro_harmonics, static_cast<uint8_t>(RPM_FILTER_HARMONICS));
        max_hz_ = 0.48f * 1000000.0f / static_cast<float>(config.looptime_us);

        for (size_t m = 0; m < MotorCount; ++m) {
            motor_rpm_lpf_[m].reset(0.0f);
            motor_rpm_lpf_[m].set_cutoff(RPM_LPF_HZ, 0.002f); // 500Hz update task rate (dt=2ms)
            for (size_t ax = 0; ax < 3; ++ax) {
                for (size_t h = 0; h < RPM_FILTER_HARMONICS; ++h) {
                    float init_freq = min_hz_ * static_cast<float>(h + 1);
                    filters_[ax][m][h].configure(BiquadType::Notch, init_freq, 1000000.0f / static_cast<float>(config.looptime_us), q_);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 500 Hz Background Task: Read ESC RPM telemetry and update notch centers
    // -------------------------------------------------------------------------
    void update_rpm_telemetry(std::span<const float, MotorCount> raw_rpms) noexcept {
        if (!config_.gyro_filter_enabled) return;

        for (size_t m = 0; m < MotorCount; ++m) {
            float filtered_hz = motor_rpm_lpf_[m].update(raw_rpms[m] * RPM_TO_HZ);

            for (size_t h = 0; h < harmonics_; ++h) {
                float harmonic_freq = filtered_hz * static_cast<float>(h + 1);
                harmonic_freq = std::clamp(harmonic_freq, min_hz_, max_hz_);

                for (size_t ax = 0; ax < 3; ++ax) {
                    filters_[ax][m][h].configure(
                        BiquadType::Notch,
                        harmonic_freq,
                        1000000.0f / static_cast<float>(config_.looptime_us),
                        q_
                    );
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 1kHz Flight Loop: Apply cascaded DF1 biquad notch filters per motor/harmonic
    // -------------------------------------------------------------------------
    [[nodiscard]] float apply(size_t axis, float input) noexcept {
        if (!config_.gyro_filter_enabled || axis >= 3) {
            return input;
        }

        float output = input;
        for (size_t m = 0; m < MotorCount; ++m) {
            for (size_t h = 0; h < harmonics_; ++h) {
                output = filters_[axis][m][h].update(output);
            }
        }
        return output;
    }

    [[nodiscard]] const RpmFilterConfig& config() const noexcept { return config_; }
    [[nodiscard]] float max_hz() const noexcept { return max_hz_; }
    [[nodiscard]] float min_hz() const noexcept { return min_hz_; }

private:
    RpmFilterConfig config_{};
    float q_{5.0f};
    float min_hz_{100.0f};
    float max_hz_{480.0f};
    uint8_t harmonics_{3};

    std::array<Pt1Filter, MotorCount> motor_rpm_lpf_{};
    std::array<std::array<std::array<BiquadFilter, RPM_FILTER_HARMONICS>, MotorCount>, 3> filters_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_RPM_FILTER_HPP
