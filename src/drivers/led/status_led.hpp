/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Onboard Status Indicator LEDs Driver
 */

#ifndef STATUS_LED_DRIVER_HPP
#define STATUS_LED_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>

namespace abstractx::drivers::led {

enum class LedState : uint8_t {
    Off   = 0,
    On    = 1,
    Blink = 2
};

class StatusLed {
public:
    // Make 64B TLP to set status LED hardware output over Virtual BAR
    static Tlp64 set_led(uint8_t led_id, LedState state, uint8_t tag) noexcept {
        uint32_t addr = bar::SystemBase + 0xC40 + static_cast<uint32_t>(led_id * 4);
        return Tlp64::make_mem_write(addr, static_cast<uint32_t>(state), tag);
    }
};

} // namespace abstractx::drivers::led

#endif // STATUS_LED_DRIVER_HPP
