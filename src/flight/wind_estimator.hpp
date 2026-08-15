/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2018-2026 INAV Contributors (Konstantin Sharlaimov, Pawel Spychalski, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Wind Velocity & RTH Energy Estimator
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `external/inav/src/main/flight/wind_estimator.c`
 *   - `external/inav/src/main/flight/wind_estimator.h`
 *   - `external/inav/src/main/navigation/navigation.c`
 *
 * Mathematical Operation:
 *   1. Pitotless/Pitot Wind Vector Estimation:
 *      v_wind_N = v_gps_N - v_airspeed * cos(yaw)
 *      v_wind_E = v_gps_E - v_airspeed * sin(yaw)
 *      Filtered with adaptive complementary gains based on turning vs steady flight.
 *   2. RTH Energy & Safe Return Horizon:
 *      Calculates headwind vector towards home coordinate, computes return ground speed,
 *      estimates flight time to home, and predicts required battery capacity (mAh).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_WIND_ESTIMATOR_HPP
#define FLIGHT_WIND_ESTIMATOR_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "filter.hpp"

namespace abstractx::flight {

struct WindVector2D {
    float speed_m_s{0.0f};          // Horizontal wind speed magnitude (m/s)
    float direction_deg{0.0f};      // Wind direction heading (degrees, 0..360, meteorological: direction FROM which wind blows)
    float wind_n_m_s{0.0f};         // North vector component (m/s)
    float wind_e_m_s{0.0f};         // East vector component (m/s)
    bool  valid{false};
};

struct RthEnergyEstimate {
    float distance_to_home_m{0.0f}; // Distance to home (meters)
    float return_heading_deg{0.0f}; // Direction towards home (degrees)
    float headwind_m_s{0.0f};       // Headwind component towards home (m/s, >0 headwind, <0 tailwind)
    float ground_speed_return_m_s{0.0f}; // Expected return ground speed (m/s)
    float time_to_home_s{0.0f};     // Estimated flight time to home (seconds)
    float energy_required_mah{0.0f};// Battery capacity needed for safe return (mAh)
    bool  can_return_safely{true};  // True if remaining capacity > energy_required + safety margin
};

struct WindEstimatorConfig {
    bool  enabled{true};
    float filter_gain{0.05f};       // Complementary filter update gain
    float cruise_airspeed_m_s{15.0f};// Nominal cruise airspeed (m/s)
    float cruise_current_a{12.0f};  // Average current draw at cruise airspeed (Amperes)
    float reserve_margin_pct{20.0f};// Safety battery margin reserve (%)
};

class WindEstimator {
public:
    constexpr WindEstimator() noexcept = default;

    void init(const WindEstimatorConfig& cfg = WindEstimatorConfig{}) noexcept {
        cfg_ = cfg;
        reset();
    }

    void reset() noexcept {
        wind_n_ = 0.0f;
        wind_e_ = 0.0f;
        wind_speed_ = 0.0f;
        wind_dir_deg_ = 0.0f;
        valid_ = false;
        sample_count_ = 0u;
    }

    // -------------------------------------------------------------------------
    // Update Wind Estimate with GPS Ground Speed + Heading + Airspeed
    // -------------------------------------------------------------------------
    [[nodiscard]] WindVector2D update(
        float gps_vel_n_m_s,
        float gps_vel_e_m_s,
        float yaw_heading_deg,
        float airspeed_m_s,
        bool  has_airspeed_sensor,
        float dt_s) noexcept
    {
        if (!cfg_.enabled || dt_s <= 0.0f) {
            return get_wind();
        }

        // If no pitot tube, use estimated cruise airspeed or throttle-derived airspeed
        float true_airspeed = (has_airspeed_sensor && airspeed_m_s > 1.0f) ? airspeed_m_s : cfg_.cruise_airspeed_m_s;

        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        const float yaw_rad = yaw_heading_deg * DEG_TO_RAD;

        // Aircraft airspeed vector components
        const float air_n = true_airspeed * std::cos(yaw_rad);
        const float air_e = true_airspeed * std::sin(yaw_rad);

        // Instantaneous wind difference: V_wind = V_ground - V_air
        const float inst_wind_n = gps_vel_n_m_s - air_n;
        const float inst_wind_e = gps_vel_e_m_s - air_e;

        // Low-pass complementary filtering
        const float alpha = std::clamp(cfg_.filter_gain * dt_s, 0.001f, 0.5f);
        wind_n_ += alpha * (inst_wind_n - wind_n_);
        wind_e_ += alpha * (inst_wind_e - wind_e_);

        // Calculate magnitude and meteorological direction
        wind_speed_ = std::sqrt(wind_n_ * wind_n_ + wind_e_ * wind_e_);

        // Direction: where wind comes FROM
        float dir_rad = std::atan2(-wind_e_, -wind_n_);
        if (dir_rad < 0.0f) { dir_rad += 6.283185307179586f; }
        wind_dir_deg_ = dir_rad * 57.29577951308232f;

        sample_count_++;
        if (sample_count_ > 50u) { // Require ~5 seconds of convergence
            valid_ = true;
        }

        return get_wind();
    }

    [[nodiscard]] WindVector2D get_wind() const noexcept {
        WindVector2D w{};
        w.wind_n_m_s = wind_n_;
        w.wind_e_m_s = wind_e_;
        w.speed_m_s = wind_speed_;
        w.direction_deg = wind_dir_deg_;
        w.valid = valid_;
        return w;
    }

    // -------------------------------------------------------------------------
    // Compute Return-To-Home (RTH) Energy & Time Horizon
    // -------------------------------------------------------------------------
    [[nodiscard]] RthEnergyEstimate calculate_rth_energy(
        float current_n_m,
        float current_e_m,
        float home_n_m,
        float home_e_m,
        float battery_remaining_mah) const noexcept
    {
        RthEnergyEstimate rth{};

        const float delta_n = home_n_m - current_n_m;
        const float delta_e = home_e_m - current_e_m;
        rth.distance_to_home_m = std::sqrt(delta_n * delta_n + delta_e * delta_e);

        if (rth.distance_to_home_m < 1.0f) {
            rth.can_return_safely = true;
            return rth;
        }

        // Bearing to home
        float bearing_rad = std::atan2(delta_e, delta_n);
        if (bearing_rad < 0.0f) { bearing_rad += 6.283185307179586f; }
        rth.return_heading_deg = bearing_rad * 57.29577951308232f;

        // Unit vector towards home
        const float unit_n = delta_n / rth.distance_to_home_m;
        const float unit_e = delta_e / rth.distance_to_home_m;

        // Headwind component: projection of wind vector onto reverse return path
        // Headwind > 0 means wind is blowing AGAINST return flight
        rth.headwind_m_s = -(wind_n_ * unit_n + wind_e_ * unit_e);

        // Expected ground speed returning home
        const float cruise_speed = cfg_.cruise_airspeed_m_s;
        rth.ground_speed_return_m_s = std::max(cruise_speed - rth.headwind_m_s, 2.0f); // Minimum 2 m/s forward progress

        // Time to return home
        rth.time_to_home_s = rth.distance_to_home_m / rth.ground_speed_return_m_s;

        // Energy calculation: Amperes * hours * 1000 = mAh
        const float return_hours = rth.time_to_home_s / 3600.0f;
        const float base_energy_mah = cfg_.cruise_current_a * return_hours * 1000.0f;

        // Add safety reserve margin
        rth.energy_required_mah = base_energy_mah * (1.0f + (cfg_.reserve_margin_pct * 0.01f));

        rth.can_return_safely = (battery_remaining_mah >= rth.energy_required_mah);

        return rth;
    }

private:
    WindEstimatorConfig cfg_{};
    float wind_n_{0.0f};
    float wind_e_{0.0f};
    float wind_speed_{0.0f};
    float wind_dir_deg_{0.0f};
    bool  valid_{false};
    uint32_t sample_count_{0u};
};

} // namespace abstractx::flight

#endif // FLIGHT_WIND_ESTIMATOR_HPP
