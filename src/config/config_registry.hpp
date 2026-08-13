/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 Zero-Linker Configuration Registry
 */

#ifndef CONFIG_REGISTRY_HPP
#define CONFIG_REGISTRY_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <cstring>

namespace abstractx {

// Strongly typed PID Controller Settings
struct PidConfig {
    float kp[3]{0.40f, 0.40f, 0.85f}; // Roll, Pitch, Yaw
    float ki[3]{0.30f, 0.30f, 0.00f};
    float kd[3]{0.03f, 0.03f, 0.00f};
    uint16_t dterm_cutoff_hz{100};
    uint16_t yaw_lpf_hz{80};
};

// Strongly typed Motor & DShot Settings
struct MotorConfig {
    uint16_t min_throttle{1150};
    uint16_t max_throttle{1850};
    uint16_t min_command{1000};
    uint8_t  motor_count{4};
    uint8_t  dshot_telemetry_enable{1};
};

// Strongly typed Navigation Settings
struct NavConfig {
    uint16_t max_speed_cms{1500};      // 15 m/s max horizontal speed
    uint16_t max_climb_rate_cms{500};  // 5 m/s max climb rate
    uint16_t rth_altitude_cm{3000};    // 30 m RTH altitude
    uint8_t  user_control_mode{0};     // Angle / Horizon / NavHold
};

// Master Configuration Container (Contiguous POD struct, Zero Linker Scripts)
struct alignas(64) MasterConfig {
    uint32_t magic{0x41535043}; // "ASPC" (AbstractX Flight Config)
    uint16_t version{1};
    uint16_t crc16{0};

    PidConfig   pid{};
    MotorConfig motor{};
    NavConfig   nav{};
};

// Configuration Registry Singleton Engine
class ConfigRegistry {
public:
    static MasterConfig& get() noexcept {
        static MasterConfig instance{};
        return instance;
    }

    static void reset_defaults() noexcept {
        get() = MasterConfig{};
    }

    static bool verify_magic() noexcept {
        return get().magic == 0x41535043;
    }
};

} // namespace abstractx

#endif // CONFIG_REGISTRY_HPP
