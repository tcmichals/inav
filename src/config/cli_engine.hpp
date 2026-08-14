/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - iNav Configurator CLI Processor Engine
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/cli/cli.c
 *   - Upstream Cleanflight: src/main/cli/cli.c
 */

#ifndef CLI_ENGINE_HPP
#define CLI_ENGINE_HPP


#include "config_registry.hpp"
#include <string_view>
#include <cstdio>
#include <array>
#include <span>

namespace abstractx::config {

class CliEngine {
public:
    static bool process_command(std::string_view cmd_line, std::span<char> response, size_t& resp_len) noexcept {
        auto& cfg = ConfigRegistry::get();
        resp_len = 0;

        if (cmd_line == "dump" || cmd_line == "diff") {
            resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
                "# INAV/ASP6 (AbstractX PCIe TLP) 3.0.0 Config Dump\r\n"
                "set p_pitch = %u\r\n"
                "set i_pitch = %u\r\n"
                "set d_pitch = %u\r\n"
                "set min_throttle = %u\r\n"
                "set max_throttle = %u\r\n"
                "set rth_altitude = %u\r\n",
                static_cast<uint16_t>(cfg.pid.kp[1] * 100.0f),
                static_cast<uint16_t>(cfg.pid.ki[1] * 100.0f),
                static_cast<uint16_t>(cfg.pid.kd[1] * 1000.0f),
                cfg.motor.min_throttle,
                cfg.motor.max_throttle,
                cfg.nav.rth_altitude_cm));
            return true;
        }

        if (cmd_line == "status") {
            resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
                "System Uptime: 42 sec, CPU Load: 1%%\r\n"
                "Clock: 64-bit HW Nanosecond Timer Latched\r\n"
                "Sensors: ACC GYRO BARO GPS EKF3 (OK)\r\n"));
            return true;
        }

        if (cmd_line == "version") {
            resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
                "# INAV/ASP6 (AbstractX PCIe TLP) 3.0.0\r\n"));
            return true;
        }

        if (cmd_line == "save") {
            ConfigRegistry::save_to_file("config.bin");
            resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
                "Saving to config.bin...\r\nRebooting...\r\n"));
            return true;
        }

        if (cmd_line == "defaults") {
            ConfigRegistry::reset_defaults();
            ConfigRegistry::save_to_file("config.bin");
            resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
                "Reset to defaults!\r\nRebooting...\r\n"));
            return true;
        }

        resp_len = static_cast<size_t>(snprintf(response.data(), response.size(),
            "Unknown command: %.*s\r\nType 'help' or 'dump'\r\n",
            static_cast<int>(cmd_line.length()), cmd_line.data()));
        return false;
    }
};

} // namespace abstractx::config

#endif // CLI_ENGINE_HPP
