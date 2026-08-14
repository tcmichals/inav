/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Bare-Metal Safe C++20 Configuration Registry & Flash Abstraction
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/config/config.c, src/main/config/parameter_group.c
 */

#ifndef CONFIG_REGISTRY_HPP
#define CONFIG_REGISTRY_HPP


#include "flash_storage.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

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

// Strongly typed Mixer Settings
struct MixerConfig {
    uint8_t  mixer_mode{0};             // 0=QuadX, 1=QuadP, 2=HexX, 3=OctoX, 4=FixedWing
    uint8_t  reverse_motors{0};         // 0=Normal (Props In), 1=Reversed (Props Out)
    uint16_t yaw_jump_prevention{400};  // Yaw jump prevention limit
};

// Master Configuration Container (Contiguous POD struct, Zero Linker Scripts)
struct alignas(64) MasterConfig {
    uint32_t magic{0x41535043}; // "ASPC" (AbstractX Flight Config)
    uint16_t version{1};
    uint16_t crc16{0};

    PidConfig   pid{};
    MotorConfig motor{};
    NavConfig   nav{};
    MixerConfig mixer{};
};

// Configuration Registry Engine with Bare-Metal Safe Flash Storage API
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

    // Load MasterConfig from Flash storage
    static bool load_from_storage(storage::FlashStorageAdapter& flash) noexcept {
        MasterConfig temp_cfg{};
        std::span<uint8_t> rd_span(reinterpret_cast<uint8_t*>(&temp_cfg), sizeof(MasterConfig));
        
        if (flash.read(0, rd_span) && temp_cfg.magic == 0x41535043) {
            get() = temp_cfg;
            return true;
        }

        reset_defaults();
        return save_to_storage(flash);
    }

    // Save MasterConfig to Flash storage
    static bool save_to_storage(storage::FlashStorageAdapter& flash) noexcept {
        std::span<const uint8_t> wr_span(reinterpret_cast<const uint8_t*>(&get()), sizeof(MasterConfig));
        flash.erase_sector(0);
        return flash.write(0, wr_span);
    }

    // SITL File compatibility helper
    static bool load_from_file(const char* /*filepath*/ = "config.bin") noexcept {
        storage::FlashStorageAdapter flash{storage::FlashMediumType::PosixFile};
        return load_from_storage(flash);
    }

    static bool save_to_file(const char* /*filepath*/ = "config.bin") noexcept {
        storage::FlashStorageAdapter flash{storage::FlashMediumType::PosixFile};
        return save_to_storage(flash);
    }
};

} // namespace abstractx

#endif // CONFIG_REGISTRY_HPP
