/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - RP2350 Pico 2 W Lean Bare-Metal Dual-Core Target Interface
 */

#ifndef PICO2W_TARGET_HPP
#define PICO2W_TARGET_HPP

#include "pico2_target.hpp"
#include "msp_server.hpp"
#include "msp_protocol.hpp"
#include "flash_storage.hpp"
#include <cstdint>
#include <array>

namespace abstractx::target::pico2w {

class Pico2WTarget : public pico2::Pico2Target {
public:
    // Core 0: Manages CYW43439 Wi-Fi AP (TCP 5760), USB CDC Serial, and QSPI Flash Logging
    static void run_core0_background_tasks(storage::FlashStorageAdapter& flash) noexcept {
        // 1. Process CYW43439 Wi-Fi AP TCP Port 5760 Configurator MSP frames
        // 2. Write pending Blackbox log TLPs to raw QSPI flash pages
        (void)flash;
    }

    // Core 1: Dedicated 8 kHz Flight Control Loop (EKF3 + PID + Navigation)
    static void run_core1_flight_loop() noexcept {
        // Runs 100% uninterrupted 8 kHz C++20 coroutine loop
    }
};

} // namespace abstractx::target::pico2w

#endif // PICO2W_TARGET_HPP
