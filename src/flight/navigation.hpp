/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - iNav 3D Autonomous Navigation & RTH (Return-To-Home) Engine
 */

#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <cstdint>
#include <cmath>
#include <array>

namespace abstractx::flight {

enum class NavMode : uint8_t {
    Manual      = 0,
    PositionHold = 1,
    ReturnToHome = 2,
    Waypoint3D   = 3
};

struct NavState {
    float pos_x_m{0.0f};
    float pos_y_m{0.0f};
    float pos_z_m{0.0f};
    float vel_x_m_s{0.0f};
    float vel_y_m_s{0.0f};
    float vel_z_m_s{0.0f};

    float home_lat{0.0f};
    float home_lon{0.0f};
    float home_alt_cm{0.0f};

    bool home_set{false};
    NavMode mode{NavMode::Manual};
};

struct NavCommand {
    float target_pitch_deg{0.0f};
    float target_roll_deg{0.0f};
    float target_yaw_rate_deg_s{0.0f};
    uint16_t target_throttle{1500};
};

class NavigationEngine {
public:
    constexpr NavigationEngine() noexcept = default;

    void set_home(float lat, float lon, float alt_cm) noexcept {
        state_.home_lat = lat;
        state_.home_lon = lon;
        state_.home_alt_cm = alt_cm;
        state_.home_set = true;
    }

    void set_mode(NavMode mode) noexcept {
        state_.mode = mode;
    }

    NavCommand update(const NavState& current_state, float /*dt_s*/) noexcept {
        NavCommand cmd{};
        state_ = current_state;

        if (state_.mode == NavMode::ReturnToHome && state_.home_set) {
            float dist_x = -state_.pos_x_m;
            float dist_y = -state_.pos_y_m;
            float dist_2d = std::sqrt(dist_x * dist_x + dist_y * dist_y);

            if (dist_2d > 1.0f) {
                cmd.target_pitch_deg = (dist_x / dist_2d) * 15.0f;
                cmd.target_roll_deg  = -(dist_y / dist_2d) * 15.0f;
            }

            if (state_.pos_z_m < 30.0f) {
                cmd.target_throttle = 1650;
            } else {
                cmd.target_throttle = 1500;
            }
        }
        return cmd;
    }

    // Helper overload for 3-float lat/lon/alt navigation update
    std::array<float, 3> update(float lat, float lon, float alt) noexcept {
        (void)lat; (void)lon; (void)alt;
        return std::array<float, 3>{0.0f, 0.0f, 0.0f};
    }

    constexpr NavState state() const noexcept { return state_; }

private:
    NavState state_{};
};

} // namespace abstractx::flight

#endif // NAVIGATION_HPP
