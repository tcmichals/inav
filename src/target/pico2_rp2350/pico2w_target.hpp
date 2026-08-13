/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Raspberry Pi RP2350 Pico 2 W (Wi-Fi + BT) Target Adapter
 */

#ifndef PICO2W_TARGET_HPP
#define PICO2W_TARGET_HPP

#include "pico2_target.hpp"

namespace abstractx::target::pico2 {

class Pico2WTarget : public Pico2Target {
public:
    static constexpr const char* name() noexcept { return "RP2350 Pico 2 W (Dual Cortex-M33 + CYW43439 Wi-Fi/BT)"; }

    // Core 0 Wireless Initialization: CYW43439 Wi-Fi Chip + TCP/UDP Server Sockets
    static void init_wifi_telemetry() noexcept {
        // 1. Initialize CYW43439 SPI interface & firmware
        // 2. Start Wi-Fi AP Mode or Station Mode
        // 3. Bind TCP Port 5760 (iNav Configurator Wireless Link)
        // 4. Bind UDP Port 19000 (BareCTF Wireless Trace Stream)
    }
};

// Compile-time check verifying Pico2WTarget satisfies C++20 IsPlatform concept
static_assert(concepts::IsPlatform<Pico2WTarget>, "Pico2WTarget must satisfy IsPlatform concept");

} // namespace abstractx::target::pico2

#endif // PICO2W_TARGET_HPP
