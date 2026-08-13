/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 iNav Autonomous Navigation & Waypoint Engine
 */

#ifndef FLIGHT_NAVIGATION_HPP
#define FLIGHT_NAVIGATION_HPP

#include "config_registry.hpp"
#include <cstdint>
#include <array>

namespace abstractx::flight {

enum class NavMode : uint8_t {
    Manual       = 0,
    Angle        = 1,
    Horizon      = 2,
    PositionHold = 3,
    ReturnToHome = 4,
    Waypoint     = 5
};

struct NavState {
    NavMode mode{NavMode::Manual};
    float target_lat{0.0f};
    float target_lon{0.0f};
    float target_alt_cm{0.0f};
    float home_lat{0.0f};
    float home_lon{0.0f};
    bool home_is_set{false};
};

class NavigationEngine {
public:
    constexpr NavigationEngine() noexcept = default;

    void set_mode(NavMode mode) noexcept {
        state_.mode = mode;
    }

    void set_home(float lat, float lon, float alt_cm) noexcept {
        state_.home_lat = lat;
        state_.home_lon = lon;
        state_.home_alt_cm = alt_cm;
        state_.home_is_set = true;
    }

    // Step navigation controller (returns target velocity vector in cm/s)
    std::array<float, 3> update(float current_lat, float current_lon, float current_alt_cm) noexcept {
        std::array<float, 3> target_vel_cms{0.0f, 0.0f, 0.0f};
        const auto& nav_cfg = ConfigRegistry::get().nav;

        if (state_.mode == NavMode::ReturnToHome && state_.home_is_set) {
            // Calculate distance & heading to home
            float d_lat = (state_.home_lat - current_lat) * 111319.5f; // approx meters
            float d_lon = (state_.home_lon - current_lon) * 111319.5f;

            target_vel_cms[0] = d_lat * 10.0f; // Scale to cm/s
            target_vel_cms[1] = d_lon * 10.0f;

            // Enforce max RTH speed
            float speed = std::sqrt(target_vel_cms[0]*target_vel_cms[0] + target_vel_cms[1]*target_vel_cms[1]);
            if (speed > nav_cfg.max_speed_cms) {
                target_vel_cms[0] = (target_vel_cms[0] / speed) * nav_cfg.max_speed_cms;
                target_vel_cms[1] = (target_vel_cms[1] / speed) * nav_cfg.max_speed_cms;
            }

            // Climb to RTH altitude
            if (current_alt_cm < nav_cfg.rth_altitude_cm) {
                target_vel_cms[2] = static_cast<float>(nav_cfg.max_climb_rate_cms);
            }
        }

        return target_vel_cms;
    }

    constexpr const NavState& state() const noexcept { return state_; }

private:
    NavState state_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_NAVIGATION_HPP
