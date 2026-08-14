/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2015-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Comprehensive C++20 Airframe Presets (Copters, Fixed-Wing, VTOL, Tricopters)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/mixer.c
 *   - Upstream Betaflight: src/main/flight/mixer.c
 */

#ifndef FLIGHT_MIXER_HPP
#define FLIGHT_MIXER_HPP

#include "pid.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

namespace abstractx::flight {

// Vehicle Category Types (Matching iNav & Betaflight Configurator Airframe Types)
enum class AirframeType : uint8_t {
    Multirotor = 0,
    FixedWing  = 1,
    Tricopter  = 2,
    VtolHybrid = 3
};

// Motor / Servo Mix Rule Entry
struct MixRule {
    float throttle{1.0f};
    float roll{0.0f};
    float pitch{0.0f};
    float yaw{0.0f};
};

// C++20 Template Motor & Servo Mixer with AirMode & Dynamic Saturation Scaling
template <size_t OutputCount = 4>
class Mixer {
public:
    constexpr explicit Mixer(const std::array<MixRule, OutputCount>& rules, AirframeType type = AirframeType::Multirotor) noexcept 
        : rules_(rules), type_(type) {}

    // Mix Throttle (0..1) + PID outputs (Roll, Pitch, Yaw) with AirMode & Saturation Scaling
    [[nodiscard]] std::array<uint16_t, OutputCount> mix(float throttle, const PidState& pid, bool airmode = true) const noexcept {
        std::array<float, OutputCount> raw_out{};
        float max_val = -1e9f;
        float min_val = 1e9f;

        // 1. Calculate raw unconstrained motor mixes
        for (size_t i = 0; i < OutputCount; ++i) {
            float motor_mix = (throttle * rules_[i].throttle) +
                              (pid.total_out[0] * rules_[i].roll) +
                              (pid.total_out[1] * rules_[i].pitch) +
                              (pid.total_out[2] * rules_[i].yaw);
            raw_out[i] = motor_mix;
            if (motor_mix > max_val) max_val = motor_mix;
            if (motor_mix < min_val) min_val = motor_mix;
        }

        // 2. AirMode Dynamic Range Scaling (Betaflight/INAV Algorithm)
        // If range exceeds 1.0, scale PID contributions down proportionally to prevent clipping
        float range = max_val - min_val;
        if (range > 1.0f) {
            float scale = 1.0f / range;
            for (size_t i = 0; i < OutputCount; ++i) {
                float pid_component = (pid.total_out[0] * rules_[i].roll) +
                                      (pid.total_out[1] * rules_[i].pitch) +
                                      (pid.total_out[2] * rules_[i].yaw);
                raw_out[i] = (throttle * rules_[i].throttle) + (pid_component * scale);
            }
            max_val = -1e9f;
            min_val = 1e9f;
            for (size_t i = 0; i < OutputCount; ++i) {
                if (raw_out[i] > max_val) max_val = raw_out[i];
                if (raw_out[i] < min_val) min_val = raw_out[i];
            }
        }

        // Shift up/down if clipping on top or bottom
        if (airmode && min_val < 0.0f) {
            float offset = -min_val;
            for (size_t i = 0; i < OutputCount; ++i) {
                raw_out[i] += offset;
            }
        } else if (max_val > 1.0f) {
            float offset = max_val - 1.0f;
            for (size_t i = 0; i < OutputCount; ++i) {
                raw_out[i] -= offset;
            }
        }

        // 3. Final Clamp & Convert to Microseconds (1000..2000 us)
        std::array<uint16_t, OutputCount> outputs{};
        for (size_t i = 0; i < OutputCount; ++i) {
            float out = std::clamp(raw_out[i], 0.0f, 1.0f);
            outputs[i] = static_cast<uint16_t>(1000.0f + (out * 1000.0f));
        }

        return outputs;
    }

    constexpr AirframeType type() const noexcept { return type_; }
    constexpr size_t output_count() const noexcept { return OutputCount; }
    constexpr const std::array<MixRule, OutputCount>& rules() const noexcept { return rules_; }

private:
    std::array<MixRule, OutputCount> rules_;
    AirframeType type_{AirframeType::Multirotor};
};

// Comprehensive Airframe Presets (iNav & Betaflight Airframe Suite)
namespace presets {

    // 1. Quadcopter X (Props In - Standard)
    constexpr std::array<MixRule, 4> QuadX = {{
        { 1.0f, -1.0f,  1.0f, -1.0f }, // Motor 1: Rear Right (CCW)
        { 1.0f, -1.0f, -1.0f,  1.0f }, // Motor 2: Front Right (CW)
        { 1.0f,  1.0f,  1.0f,  1.0f }, // Motor 3: Rear Left (CW)
        { 1.0f,  1.0f, -1.0f, -1.0f }  // Motor 4: Front Left (CCW)
    }};

    // 2. Quadcopter X (Props Out - Reversed Rotation)
    constexpr std::array<MixRule, 4> QuadX_PropsOut = {{
        { 1.0f, -1.0f,  1.0f,  1.0f }, // Motor 1: Rear Right (CW)
        { 1.0f, -1.0f, -1.0f, -1.0f }, // Motor 2: Front Right (CCW)
        { 1.0f,  1.0f,  1.0f, -1.0f }, // Motor 3: Rear Left (CCW)
        { 1.0f,  1.0f, -1.0f,  1.0f }  // Motor 4: Front Left (CW)
    }};

    // 3. Quadcopter Plus (4 Motors)
    constexpr std::array<MixRule, 4> QuadPlus = {{
        { 1.0f,  0.0f,  1.0f, -1.0f }, // Rear Motor
        { 1.0f, -1.0f,  0.0f,  1.0f }, // Right Motor
        { 1.0f,  0.0f, -1.0f, -1.0f }, // Front Motor
        { 1.0f,  1.0f,  0.0f,  1.0f }  // Left Motor
    }};

    // 4. Tricopter (3 Motors + 1 Yaw Servo)
    constexpr std::array<MixRule, 4> Tricopter = {{
        { 1.0f, -1.0f,  0.67f,  0.0f }, // Front Right Motor
        { 1.0f,  1.0f,  0.67f,  0.0f }, // Front Left Motor
        { 1.0f,  0.0f, -1.33f,  0.0f }, // Tail Motor
        { 0.0f,  0.0f,  0.0f,   1.0f }  // Tail Yaw Servo
    }};

    // 5. Fixed-Wing Standard Airplane (1 Motor + 3 Servos: Aileron, Elevator, Rudder)
    constexpr std::array<MixRule, 4> AirplaneStandard = {{
        { 1.0f,  0.0f,  0.0f,  0.0f }, // Motor Throttle
        { 0.0f,  1.0f,  0.0f,  0.0f }, // Aileron Servo
        { 0.0f,  0.0f,  1.0f,  0.0f }, // Elevator Servo
        { 0.0f,  0.0f,  0.0f,  1.0f }  // Rudder Servo
    }};

    // 6. Fixed-Wing Flying Wing / Delta Wing (1 Motor + 2 Elevon Servos)
    constexpr std::array<MixRule, 3> FlyingWing = {{
        { 1.0f,  0.0f,  0.0f,  0.0f }, // Motor Throttle
        { 0.0f,  0.5f,  0.5f,  0.0f }, // Left Elevon (Pitch + Roll)
        { 0.0f, -0.5f,  0.5f,  0.0f }  // Right Elevon (Pitch - Roll)
    }};

    // 7. Hexacopter X (6 Motors)
    constexpr std::array<MixRule, 6> HexX = {{
        { 1.0f, -0.5f,  0.866f, -1.0f },
        { 1.0f, -1.0f,  0.0f,    1.0f },
        { 1.0f, -0.5f, -0.866f, -1.0f },
        { 1.0f,  0.5f,  0.866f,  1.0f },
        { 1.0f,  1.0f,  0.0f,   -1.0f },
        { 1.0f,  0.5f, -0.866f,  1.0f }
    }};

    // 8. Octocopter X8 Coaxial (8 Motors)
    constexpr std::array<MixRule, 8> OctoX8 = {{
        { 1.0f, -1.0f,  1.0f, -1.0f }, // Top Rear Right (CCW)
        { 1.0f, -1.0f, -1.0f,  1.0f }, // Top Front Right (CW)
        { 1.0f,  1.0f,  1.0f,  1.0f }, // Top Rear Left (CW)
        { 1.0f,  1.0f, -1.0f, -1.0f }, // Top Front Left (CCW)
        { 1.0f, -1.0f,  1.0f,  1.0f }, // Bottom Rear Right (CW)
        { 1.0f, -1.0f, -1.0f, -1.0f }, // Bottom Front Right (CCW)
        { 1.0f,  1.0f,  1.0f, -1.0f }, // Bottom Rear Left (CCW)
        { 1.0f,  1.0f, -1.0f,  1.0f }  // Bottom Front Left (CW)
    }};

    // 9. Flat Octocopter (8 Motors)
    constexpr std::array<MixRule, 8> FlatOctoX = {{
        { 1.0f, -0.383f,  0.924f, -1.0f },
        { 1.0f, -0.924f,  0.383f,  1.0f },
        { 1.0f, -0.924f, -0.383f, -1.0f },
        { 1.0f, -0.383f, -0.924f,  1.0f },
        { 1.0f,  0.383f,  0.924f,  1.0f },
        { 1.0f,  0.924f,  0.383f, -1.0f },
        { 1.0f,  0.924f, -0.383f,  1.0f },
        { 1.0f,  0.383f, -0.924f, -1.0f }
    }};

} // namespace presets

} // namespace abstractx::flight

#endif // FLIGHT_MIXER_HPP
