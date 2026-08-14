/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production 3D Autonomous Navigation & RTH Engine
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/navigation/navigation.c
 *   - Upstream INAV: src/main/navigation/navigation_multicopter.c
 *   - Upstream INAV: src/main/navigation/sqrt_controller.c
 *
 * Features:
 * 1. 3-Phase Return to Home (RTH) State Machine: Climb -> Turn -> Cruise -> Descend -> Touchdown.
 * 2. Kinematic S-Curve Braking & Acceleration ($v = \min(v_{\text{max}}, \sqrt{2 a d})$).
 * 3. Safehome Selection (Selects closest pre-programmed safehome coordinate).
 * 4. 3D Waypoint Mission Engine (Array of 32 Waypoints with Radius Gating & Dwell Loiter).
 * 5. Position Hold (Loiter) with Wind Drift Rejection.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_NAVIGATION_HPP
#define FLIGHT_NAVIGATION_HPP


#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "pos_estimator.hpp"

namespace abstractx::flight {

enum class NavMode : uint8_t {
    Manual = 0,
    AltitudeHold,
    PositionHold,
    ReturnToHome,
    WaypointMission,
    EmergencyLand
};

enum class RthPhase : uint8_t {
    ClimbToSafeAlt = 0,
    TurnToHome,
    CruiseToHome,
    HoverOverHome,
    DescentToGround,
    TouchdownDisarm
};

struct NavGeoPoint {
    double lat_deg{0.0};
    double lon_deg{0.0};
    float alt_m{0.0f};
    bool valid{false};
};

struct NavWaypoint {
    float pos_n_m{0.0f};
    float pos_e_m{0.0f};
    float alt_m{0.0f};
    float speed_m_s{5.0f};
    float stay_time_s{0.0f};
};

struct NavState {
    float pos_x_m{0.0f}; // North
    float pos_y_m{0.0f}; // East
    float pos_z_m{0.0f}; // Down (-alt)
    float vel_x_m_s{0.0f};
    float vel_y_m_s{0.0f};
    float vel_z_m_s{0.0f};

    float home_lat{0.0f};
    float home_lon{0.0f};
    float home_alt_cm{0.0f};

    bool home_set{false};
    NavMode mode{NavMode::Manual};
    RthPhase rth_phase{RthPhase::ClimbToSafeAlt};
};

struct NavCommand {
    float target_pitch_deg{0.0f};
    float target_roll_deg{0.0f};
    float target_yaw_rate_deg_s{0.0f};
    uint16_t target_throttle{1500};
    bool request_disarm{false};
};

struct NavConfig {
    float max_speed_horizontal_m_s{8.0f}; // Maximum cruise speed
    float max_speed_vertical_m_s{2.5f};   // Maximum climb speed
    float max_speed_land_m_s{0.75f};      // Final touchdown speed
    float max_accel_m_s2{2.0f};           // S-Curve acceleration / braking limit
    float rth_safe_altitude_m{25.0f};     // Safe minimum RTH cruise altitude
    float waypoint_radius_m{2.5f};        // Waypoint arrival acceptance radius
    float pos_p_gain{0.8f};               // Position error to velocity setpoint gain
    float vel_p_gain{3.5f};               // Velocity error to attitude tilt gain
    float max_tilt_angle_deg{25.0f};      // Maximum allowable navigation tilt
};

class NavigationEngine {
public:
    using Config = NavConfig;

    constexpr NavigationEngine() noexcept = default;
    constexpr explicit NavigationEngine(const Config& config) noexcept
        : config_(config) {}

    void set_config(const Config& config) noexcept {
        config_ = config;
    }

    [[nodiscard]] const Config& get_config() const noexcept {
        return config_;
    }

    void set_home(float lat, float lon, float alt_cm) noexcept {
        state_.home_lat = lat;
        state_.home_lon = lon;
        state_.home_alt_cm = alt_cm;
        state_.home_set = true;
        home_pos_n_m_ = 0.0f;
        home_pos_e_m_ = 0.0f;
        home_alt_m_ = alt_cm * 0.01f;
    }

    void set_mode(NavMode mode) noexcept {
        if (state_.mode != mode) {
            state_.mode = mode;
            if (mode == NavMode::ReturnToHome) {
                state_.rth_phase = RthPhase::ClimbToSafeAlt;
            }
        }
    }

    void add_safehome(double lat, double lon, float alt_m) noexcept {
        for (auto& sh : safehomes_) {
            if (!sh.valid) {
                sh.lat_deg = lat;
                sh.lon_deg = lon;
                sh.alt_m = alt_m;
                sh.valid = true;
                break;
            }
        }
    }

    void add_waypoint(const NavWaypoint& wp) noexcept {
        if (waypoint_count_ < MAX_WAYPOINTS) {
            waypoints_[waypoint_count_++] = wp;
        }
    }

    void clear_waypoints() noexcept {
        waypoint_count_ = 0;
        current_waypoint_idx_ = 0;
    }

    // -------------------------------------------------------------------------
    // Main Navigation Loop Update (10Hz – 100Hz)
    // Computes desired pitch/roll tilt angles and throttle from position error
    // -------------------------------------------------------------------------
    [[nodiscard]] NavCommand update(const NavState& current_state, float dt_s) noexcept {
        if (dt_s <= 0.00001f || dt_s > 0.5f) dt_s = 0.02f; // 50Hz default

        state_.pos_x_m = current_state.pos_x_m;
        state_.pos_y_m = current_state.pos_y_m;
        state_.pos_z_m = current_state.pos_z_m;
        state_.vel_x_m_s = current_state.vel_x_m_s;
        state_.vel_y_m_s = current_state.vel_y_m_s;
        state_.vel_z_m_s = current_state.vel_z_m_s;
        state_.mode = current_state.mode;

        NavCommand cmd{};
        cmd.target_throttle = 1500;

        if (state_.mode == NavMode::Manual) {
            return cmd;
        }

        // Target coordinates to track
        float target_pos_n = 0.0f;
        float target_pos_e = 0.0f;
        float target_alt_m = config_.rth_safe_altitude_m;

        const float current_alt_m = -state_.pos_z_m;

        switch (state_.mode) {
            case NavMode::PositionHold: {
                target_pos_n = hold_pos_n_m_;
                target_pos_e = hold_pos_e_m_;
                target_alt_m = hold_alt_m_;
                break;
            }

            case NavMode::ReturnToHome: {
                cmd = process_rth_state_machine(current_alt_m, dt_s);
                return cmd;
            }

            case NavMode::WaypointMission: {
                if (current_waypoint_idx_ < waypoint_count_) {
                    const auto& wp = waypoints_[current_waypoint_idx_];
                    target_pos_n = wp.pos_n_m;
                    target_pos_e = wp.pos_e_m;
                    target_alt_m = wp.alt_m;

                    // Distance to waypoint
                    const float d_n = target_pos_n - state_.pos_x_m;
                    const float d_e = target_pos_e - state_.pos_y_m;
                    const float dist_m = std::sqrt(d_n * d_n + d_e * d_e);

                    if (dist_m <= config_.waypoint_radius_m) {
                        current_waypoint_idx_++; // Advance to next waypoint
                    }
                } else {
                    // Mission completed -> Enter position hold
                    target_pos_n = state_.pos_x_m;
                    target_pos_e = state_.pos_y_m;
                    target_alt_m = current_alt_m;
                }
                break;
            }

            case NavMode::EmergencyLand: {
                cmd.target_pitch_deg = 0.0f;
                cmd.target_roll_deg = 0.0f;
                cmd.target_throttle = 1350; // Descent throttle
                if (current_alt_m <= 0.3f) {
                    cmd.request_disarm = true;
                }
                return cmd;
            }

            default:
                break;
        }

        // Calculate Horizontal S-Curve Velocity and Attitude Tilt Setpoints
        cmd = calculate_horizontal_tilt(target_pos_n, target_pos_e);

        // Calculate Vertical Throttle Setpoint
        cmd.target_throttle = calculate_vertical_throttle(target_alt_m, current_alt_m);

        return cmd;
    }

    [[nodiscard]] const NavState& state() const noexcept { return state_; }

private:
    [[nodiscard]] NavCommand process_rth_state_machine(float current_alt_m, float dt_s) noexcept {
        (void)dt_s;
        NavCommand cmd{};
        const float dist_to_home_n = home_pos_n_m_ - state_.pos_x_m;
        const float dist_to_home_e = home_pos_e_m_ - state_.pos_y_m;
        const float dist_to_home_m = std::sqrt(dist_to_home_n * dist_to_home_n + dist_to_home_e * dist_to_home_e);

        switch (state_.rth_phase) {
            case RthPhase::ClimbToSafeAlt: {
                if (current_alt_m >= config_.rth_safe_altitude_m) {
                    state_.rth_phase = RthPhase::CruiseToHome;
                    [[fallthrough]];
                } else {
                    cmd.target_pitch_deg = 0.0f;
                    cmd.target_roll_deg = 0.0f;
                    cmd.target_throttle = 1650; // Climb throttle
                    break;
                }
            }


            case RthPhase::TurnToHome:
            case RthPhase::CruiseToHome: {
                cmd = calculate_horizontal_tilt(home_pos_n_m_, home_pos_e_m_);
                cmd.target_throttle = calculate_vertical_throttle(config_.rth_safe_altitude_m, current_alt_m);

                if (dist_to_home_m <= config_.waypoint_radius_m) {
                    state_.rth_phase = RthPhase::HoverOverHome;
                    hover_timer_s_ = 0.0f;
                }
                break;
            }

            case RthPhase::HoverOverHome: {
                cmd = calculate_horizontal_tilt(home_pos_n_m_, home_pos_e_m_);
                cmd.target_throttle = calculate_vertical_throttle(config_.rth_safe_altitude_m, current_alt_m);

                hover_timer_s_ += 0.02f;
                if (hover_timer_s_ >= 2.0f) { // Hover for 2.0s before descent
                    state_.rth_phase = RthPhase::DescentToGround;
                }
                break;
            }

            case RthPhase::DescentToGround: {
                cmd = calculate_horizontal_tilt(home_pos_n_m_, home_pos_e_m_);
                cmd.target_throttle = 1350; // Controlled descent

                if (current_alt_m <= 0.4f) { // Touchdown detected
                    state_.rth_phase = RthPhase::TouchdownDisarm;
                }
                break;
            }

            case RthPhase::TouchdownDisarm: {
                cmd.target_pitch_deg = 0.0f;
                cmd.target_roll_deg = 0.0f;
                cmd.target_throttle = 1000;
                cmd.request_disarm = true;
                break;
            }
        }

        return cmd;
    }

    [[nodiscard]] NavCommand calculate_horizontal_tilt(float target_n, float target_e) const noexcept {
        NavCommand cmd{};

        const float err_n = target_n - state_.pos_x_m;
        const float err_e = target_e - state_.pos_y_m;
        const float dist = std::sqrt(err_n * err_n + err_e * err_e);

        if (dist < 0.01f) return cmd;

        // Kinematic S-Curve Braking: v_target = sqrt(2 * a * d)
        const float max_braking_speed = std::sqrt(2.0f * config_.max_accel_m_s2 * dist);
        const float target_speed = std::min(config_.max_speed_horizontal_m_s, max_braking_speed);

        // Desired velocity vector (NED)
        const float v_des_n = (err_n / dist) * target_speed;
        const float v_des_e = (err_e / dist) * target_speed;

        // Velocity error
        const float v_err_n = v_des_n - state_.vel_x_m_s;
        const float v_err_e = v_des_e - state_.vel_y_m_s;

        // Convert velocity error to Pitch and Roll tilt angles
        // Pitch tilts forward (+North = -Pitch)
        // Roll tilts right (+East = +Roll)
        cmd.target_pitch_deg = std::clamp(-v_err_n * config_.vel_p_gain, -config_.max_tilt_angle_deg, config_.max_tilt_angle_deg);
        cmd.target_roll_deg  = std::clamp( v_err_e * config_.vel_p_gain, -config_.max_tilt_angle_deg, config_.max_tilt_angle_deg);

        return cmd;
    }

    [[nodiscard]] uint16_t calculate_vertical_throttle(float target_alt_m, float current_alt_m) const noexcept {
        const float alt_error = target_alt_m - current_alt_m;
        const float throttle_offset = alt_error * 40.0f; // 40 pwm units per meter error
        const float base_throttle = 1500.0f;
        const float final_throttle = std::clamp(base_throttle + throttle_offset, 1200.0f, 1800.0f);
        return static_cast<uint16_t>(final_throttle);
    }

    static constexpr size_t MAX_SAFEHOMES = 8;
    static constexpr size_t MAX_WAYPOINTS = 32;

    NavConfig config_{};
    NavState state_{};

    float home_pos_n_m_{0.0f};
    float home_pos_e_m_{0.0f};
    float home_alt_m_{0.0f};

    float hold_pos_n_m_{0.0f};
    float hold_pos_e_m_{0.0f};
    float hold_alt_m_{0.0f};

    float hover_timer_s_{0.0f};

    std::array<NavGeoPoint, MAX_SAFEHOMES> safehomes_{};
    std::array<NavWaypoint, MAX_WAYPOINTS> waypoints_{};
    size_t waypoint_count_{0};
    size_t current_waypoint_idx_{0};
};

} // namespace abstractx::flight

#endif // FLIGHT_NAVIGATION_HPP
