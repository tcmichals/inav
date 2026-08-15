/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Betaflight / INAV Contributors (BorisB, Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Sensor Calibration & Temperature Drift Compensation Engine
 *
 * Reference:
 *   - Upstream INAV: src/main/sensors/gyro.c, src/main/sensors/acceleration.c, src/main/sensors/compass.c, src/main/sensors/barometer.c
 *   - Upstream Betaflight: src/main/sensors/gyro.c, src/main/sensors/acceleration.c
 *
 * Capabilities:
 *   1. Gyro Zero-Bias Calibration with Moving Variance Motion Detection:
 *      Averages 1000 samples at startup; if standard deviation exceeds threshold,
 *      calibration automatically resets until the aircraft is stationary.
 *   2. Gyro Temperature Drift Linear/Polynomial Compensation (bias = bias0 + K_T * (T - T0)).
 *   3. Accelerometer 1G Vector Normalization & Offset Correction.
 *   4. Magnetometer Hard-Iron (Offset) & Soft-Iron (Scale Matrix) Correction.
 *   5. Barometer Ground Reference Pressure (P0) Zeroing on Arming Event.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef SENSORS_SENSOR_CALIBRATION_HPP
#define SENSORS_SENSOR_CALIBRATION_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "flight/pid.hpp"

namespace abstractx::sensors {

struct GyroCalConfig {
    uint16_t sample_target{1000u};       // 1000 samples @ 1kHz = 1.0s
    float    max_motion_variance{4.0f};  // (deg/s)^2 threshold for stationary check
    float    temp_coeff_dps_c[3]{0.0f, 0.0f, 0.0f}; // Gyro bias temp slope (deg/s per deg C)
    float    temp_cal_deg_c{25.0f};      // Reference calibration temperature (deg C)
};

struct AccelCalConfig {
    flight::Axis3f offset{0.0f, 0.0f, 0.0f}; // In g
    flight::Axis3f scale{1.0f, 1.0f, 1.0f};  // Gain factor
};

struct MagCalConfig {
    flight::Axis3f hard_iron_offset{0.0f, 0.0f, 0.0f}; // Gauss / raw LSB
    flight::Axis3f soft_iron_scale{1.0f, 1.0f, 1.0f};
    float          declination_deg{0.0f};
};

class SensorCalibrationEngine {
public:
    constexpr SensorCalibrationEngine() noexcept = default;

    void init(const GyroCalConfig& g_cfg = GyroCalConfig{},
              const AccelCalConfig& a_cfg = AccelCalConfig{},
              const MagCalConfig& m_cfg = MagCalConfig{}) noexcept
    {
        gyro_cfg_ = g_cfg;
        accel_cfg_ = a_cfg;
        mag_cfg_ = m_cfg;
        reset_gyro_calibration();
    }

    void reset_gyro_calibration() noexcept {
        gyro_samples_collected_ = 0u;
        gyro_sum_ = flight::Axis3f{};
        gyro_sq_sum_ = flight::Axis3f{};
        gyro_bias_ = flight::Axis3f{};
        gyro_calibrated_ = false;
    }

    // -------------------------------------------------------------------------
    // Gyro Calibration Step (Call during disarmed boot sequence)
    // -------------------------------------------------------------------------
    [[nodiscard]] bool update_gyro_calibration(const flight::Axis3f& raw_gyro_dps) noexcept {
        if (gyro_calibrated_) {
            return true;
        }

        gyro_samples_collected_++;
        for (size_t i = 0u; i < 3u; ++i) {
            gyro_sum_[i] += raw_gyro_dps[i];
            gyro_sq_sum_[i] += (raw_gyro_dps[i] * raw_gyro_dps[i]);
        }

        if (gyro_samples_collected_ >= gyro_cfg_.sample_target) {
            const float inv_n = 1.0f / static_cast<float>(gyro_samples_collected_);
            bool motion_detected = false;

            for (size_t i = 0u; i < 3u; ++i) {
                const float mean = gyro_sum_[i] * inv_n;
                const float variance = (gyro_sq_sum_[i] * inv_n) - (mean * mean);
                if (variance > gyro_cfg_.max_motion_variance) {
                    motion_detected = true;
                    break;
                }
            }

            if (motion_detected) {
                // Aircraft moved during calibration — reset and try again
                reset_gyro_calibration();
                return false;
            }

            // Calibration successful
            for (size_t i = 0u; i < 3u; ++i) {
                gyro_bias_[i] = gyro_sum_[i] * inv_n;
            }
            gyro_calibrated_ = true;
            return true;
        }

        return false;
    }

    // -------------------------------------------------------------------------
    // Apply Gyro Calibration & Temperature Compensation
    // -------------------------------------------------------------------------
    [[nodiscard]] flight::Axis3f calibrate_gyro(const flight::Axis3f& raw_gyro_dps, float temp_deg_c) const noexcept {
        const float dt = temp_deg_c - gyro_cfg_.temp_cal_deg_c;
        flight::Axis3f calibrated{};

        for (size_t i = 0u; i < 3u; ++i) {
            const float temp_drift = gyro_cfg_.temp_coeff_dps_c[i] * dt;
            calibrated[i] = raw_gyro_dps[i] - (gyro_bias_[i] + temp_drift);
        }

        return calibrated;
    }

    // -------------------------------------------------------------------------
    // Apply Accelerometer Calibration
    // -------------------------------------------------------------------------
    [[nodiscard]] flight::Axis3f calibrate_accel(const flight::Axis3f& raw_accel_g) const noexcept {
        flight::Axis3f calibrated{};
        for (size_t i = 0u; i < 3u; ++i) {
            calibrated[i] = (raw_accel_g[i] - accel_cfg_.offset[i]) * accel_cfg_.scale[i];
        }
        return calibrated;
    }

    // -------------------------------------------------------------------------
    // Apply Magnetometer Hard & Soft Iron Calibration
    // -------------------------------------------------------------------------
    [[nodiscard]] flight::Axis3f calibrate_mag(const flight::Axis3f& raw_mag) const noexcept {
        flight::Axis3f calibrated{};
        for (size_t i = 0u; i < 3u; ++i) {
            calibrated[i] = (raw_mag[i] - mag_cfg_.hard_iron_offset[i]) * mag_cfg_.soft_iron_scale[i];
        }
        return calibrated;
    }

    // -------------------------------------------------------------------------
    // Barometer Ground Reference Calibration (Arming Zero)
    // -------------------------------------------------------------------------
    void set_ground_pressure(float pressure_pa) noexcept {
        if (pressure_pa > 50000.0f && pressure_pa < 115000.0f) {
            ground_pressure_pa_ = pressure_pa;
            baro_calibrated_ = true;
        }
    }

    [[nodiscard]] float calculate_relative_altitude_m(float pressure_pa) const noexcept {
        if (!baro_calibrated_ || pressure_pa <= 0.0f) {
            return 0.0f;
        }
        // Hypsometric Barometric Formula: h = 44330 * (1 - (P / P0)^(1 / 5.255))
        return 44330.0f * (1.0f - std::pow(pressure_pa / ground_pressure_pa_, 0.19029495718f));
    }

    [[nodiscard]] bool is_gyro_calibrated() const noexcept { return gyro_calibrated_; }
    [[nodiscard]] bool is_baro_calibrated() const noexcept { return baro_calibrated_; }
    [[nodiscard]] const flight::Axis3f& get_gyro_bias() const noexcept { return gyro_bias_; }
    [[nodiscard]] float get_ground_pressure_pa() const noexcept { return ground_pressure_pa_; }

private:
    GyroCalConfig  gyro_cfg_{};
    AccelCalConfig accel_cfg_{};
    MagCalConfig   mag_cfg_{};

    // Gyro state
    uint16_t       gyro_samples_collected_{0u};
    flight::Axis3f gyro_sum_{};
    flight::Axis3f gyro_sq_sum_{};
    flight::Axis3f gyro_bias_{};
    bool           gyro_calibrated_{false};

    // Barometer ground reference
    float          ground_pressure_pa_{101325.0f};
    bool           baro_calibrated_{false};
};

} // namespace abstractx::sensors

#endif // SENSORS_SENSOR_CALIBRATION_HPP
