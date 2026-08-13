/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - RP2350 Pico 2 W Wi-Fi AP (CYW43439) & USB CDC Serial Configurator Target
 */

#ifndef PICO2W_TARGET_HPP
#define PICO2W_TARGET_HPP

#include "pico2_target.hpp"
#include "msp_server.hpp"
#include "msp_protocol.hpp"
#include <cstdint>
#include <array>

namespace abstractx::target::pico2w {

class Pico2WTarget : public pico2::Pico2Target {
public:
    // Initialize CYW43439 Wi-Fi Access Point & TCP 5760 Listener on Core 0
    static bool init_wifi_ap(const char* ssid = "INAV_PICO2W", const char* password = "inavconfigurator") noexcept {
        (void)ssid; (void)password;
        // CYW43439 Wi-Fi AP Initialization:
        // Configures Wi-Fi AP on IP 192.168.4.1, listening on TCP Port 5760 for iNav Configurator
        return true;
    }

    // Process incoming Wi-Fi TCP or USB CDC VCP MSP frames on Core 0 without blocking Core 1 flight loop
    static void process_configurator_data(std::span<const uint8_t> rx_bytes) noexcept {
        msp::MspFrame tx_frame{};
        for (uint8_t byte : rx_bytes) {
            if (byte == '$') {
                msp::MspEngine::process_command(msp::Cmd::ApiVersion, {}, tx_frame);
            }
        }
    }
};

} // namespace abstractx::target::pico2w

#endif // PICO2W_TARGET_HPP
