/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 3-Axis Flight PID Controller
 */

#ifndef FLIGHT_PID_HPP
#define FLIGHT_PID_HPP

#include "config_registry.hpp"
#include <cstdint>
#include <array>

namespace abstractx::flight {

struct PidState {
    std::array<float, 3> p_out{0.0f, 0.0f, 0.0f};
    std::array<float, 3> i_out{0.0f, 0.0f, 0.0f};
    std::array<float, 3> d_out{0.0f, 0.0f, 0.0f};
    std::array<float, 3> total_out{0.0f, 0.0f, 0.0f};
};

class PidController {
public:
    constexpr PidController() noexcept = default;

    void reset() noexcept {
        i_term_.fill(0.0f);
        prev_gyro_.fill(0.0f);
    }

    // Execute PID update loop (dt in seconds)
    PidState update(const std::array<float, 3>& target_rates, 
                   const std::array<float, 3>& gyro_rates, 
                   float dt) noexcept {
        PidState state{};
        const auto& pid_cfg = ConfigRegistry::get().pid;

        for (size_t axis = 0; axis < 3; ++axis) {
            const float error = target_rates[axis] - gyro_rates[axis];

            // P-Term
            state.p_out[axis] = error * pid_cfg.kp[axis];

            // I-Term with anti-windup clamping
            i_term_[axis] += error * pid_cfg.ki[axis] * dt;
            if (i_term_[axis] > 0.4f) i_term_[axis] = 0.4f;
            if (i_term_[axis] < -0.4f) i_term_[axis] = -0.4f;
            state.i_out[axis] = i_term_[axis];

            // D-Term on Gyro Derivative
            const float delta_gyro = (gyro_rates[axis] - prev_gyro_[axis]) / dt;
            prev_gyro_[axis] = gyro_rates[axis];
            state.d_out[axis] = -delta_gyro * pid_cfg.kd[axis];

            // Total Output
            state.total_out[axis] = state.p_out[axis] + state.i_out[axis] + state.d_out[axis];
        }

        return state;
    }

private:
    std::array<float, 3> i_term_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> prev_gyro_{0.0f, 0.0f, 0.0f};
};

} // namespace abstractx::flight

#endif // FLIGHT_PID_HPP
