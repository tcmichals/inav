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

// C++20 Template Motor & Servo Mixer (Zero RAM waste!)
template <size_t OutputCount = 4>
class Mixer {
public:
    constexpr explicit Mixer(const std::array<MixRule, OutputCount>& rules, AirframeType type = AirframeType::Multirotor) noexcept 
        : rules_(rules), type_(type) {}

    // Mix Throttle (0..1) + PID outputs (Roll, Pitch, Yaw) into N DShot/PWM output commands
    std::array<uint16_t, OutputCount> mix(float throttle, const PidState& pid) const noexcept {
        std::array<uint16_t, OutputCount> outputs{};

        for (size_t i = 0; i < OutputCount; ++i) {
            float out = (throttle * rules_[i].throttle) +
                        (pid.total_out[0] * rules_[i].roll) +
                        (pid.total_out[1] * rules_[i].pitch) +
                        (pid.total_out[2] * rules_[i].yaw);

            // Constrain output to DShot/PWM command range (1000..2000 us)
            out = std::clamp(out, 0.0f, 1.0f);
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

    // 1. Quadcopter X (4 Motors)
    constexpr std::array<MixRule, 4> QuadX = {{
        { 1.0f, -1.0f,  1.0f, -1.0f }, // Rear Right (CW)
        { 1.0f, -1.0f, -1.0f,  1.0f }, // Front Right (CCW)
        { 1.0f,  1.0f,  1.0f,  1.0f }, // Rear Left (CCW)
        { 1.0f,  1.0f, -1.0f, -1.0f }  // Front Left (CW)
    }};

    // 2. Quadcopter Plus (4 Motors)
    constexpr std::array<MixRule, 4> QuadPlus = {{
        { 1.0f,  0.0f,  1.0f, -1.0f }, // Rear Motor
        { 1.0f, -1.0f,  0.0f,  1.0f }, // Right Motor
        { 1.0f,  0.0f, -1.0f, -1.0f }, // Front Motor
        { 1.0f,  1.0f,  0.0f,  1.0f }  // Left Motor
    }};

    // 3. Tricopter (3 Motors + 1 Yaw Servo)
    constexpr std::array<MixRule, 4> Tricopter = {{
        { 1.0f, -1.0f,  0.67f,  0.0f }, // Front Right Motor
        { 1.0f,  1.0f,  0.67f,  0.0f }, // Front Left Motor
        { 1.0f,  0.0f, -1.33f,  0.0f }, // Tail Motor
        { 0.0f,  0.0f,  0.0f,   1.0f }  // Tail Yaw Servo
    }};

    // 4. Fixed-Wing Standard Airplane (1 Motor + 3 Servos: Aileron, Elevator, Rudder)
    constexpr std::array<MixRule, 4> AirplaneStandard = {{
        { 1.0f,  0.0f,  0.0f,  0.0f }, // Motor Throttle
        { 0.0f,  1.0f,  0.0f,  0.0f }, // Aileron Servo
        { 0.0f,  0.0f,  1.0f,  0.0f }, // Elevator Servo
        { 0.0f,  0.0f,  0.0f,  1.0f }  // Rudder Servo
    }};

    // 5. Fixed-Wing Flying Wing / Delta Wing (1 Motor + 2 Elevon Servos)
    constexpr std::array<MixRule, 3> FlyingWing = {{
        { 1.0f,  0.0f,  0.0f,  0.0f }, // Motor Throttle
        { 0.0f,  0.5f,  0.5f,  0.0f }, // Left Elevon (Pitch + Roll)
        { 0.0f, -0.5f,  0.5f,  0.0f }  // Right Elevon (Pitch - Roll)
    }};

    // 6. Hexacopter X (6 Motors)
    constexpr std::array<MixRule, 6> HexX = {{
        { 1.0f, -0.5f,  0.866f, -1.0f },
        { 1.0f, -1.0f,  0.0f,    1.0f },
        { 1.0f, -0.5f, -0.866f, -1.0f },
        { 1.0f,  0.5f,  0.866f,  1.0f },
        { 1.0f,  1.0f,  0.0f,   -1.0f },
        { 1.0f,  0.5f, -0.866f,  1.0f }
    }};

    // 7. Octocopter X8 Coaxial (8 Motors)
    constexpr std::array<MixRule, 8> OctoX8 = {{
        { 1.0f, -1.0f,  1.0f, -1.0f },
        { 1.0f, -1.0f, -1.0f,  1.0f },
        { 1.0f,  1.0f,  1.0f,  1.0f },
        { 1.0f,  1.0f, -1.0f, -1.0f },
        { 1.0f, -1.0f,  1.0f,  1.0f },
        { 1.0f, -1.0f, -1.0f, -1.0f },
        { 1.0f,  1.0f,  1.0f, -1.0f },
        { 1.0f,  1.0f, -1.0f,  1.0f }
    }};

    // 8. VTOL QuadPlane Hybrid (4 Quad Motors + 1 Forward Pusher Motor)
    constexpr std::array<MixRule, 5> VtolQuadPlane = {{
        { 1.0f, -1.0f,  1.0f, -1.0f }, // Quad Motor 1
        { 1.0f, -1.0f, -1.0f,  1.0f }, // Quad Motor 2
        { 1.0f,  1.0f,  1.0f,  1.0f }, // Quad Motor 3
        { 1.0f,  1.0f, -1.0f, -1.0f }, // Quad Motor 4
        { 1.0f,  0.0f,  0.0f,  0.0f }  // Forward Pusher Motor (Throttle only)
    }};

} // namespace presets

} // namespace abstractx::flight

#endif // FLIGHT_MIXER_HPP
