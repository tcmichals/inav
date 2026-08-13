/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated FastPWM & Standard PWM ESC Driver
 */

#ifndef PWM_ESC_DRIVER_HPP
#define PWM_ESC_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <algorithm>

namespace abstractx::drivers::esc {

class PwmEsc {
public:
    static Tlp64 make_motor_write(uint8_t motor_index, uint16_t command, uint8_t tag) noexcept {
        uint32_t target_addr = bar::EscBase + static_cast<uint32_t>(motor_index * 4);
        uint32_t pwm_us = std::clamp<uint16_t>(command, 1000, 2000);
        return Tlp64::make_mem_write(target_addr, pwm_us, tag);
    }
};

} // namespace abstractx::drivers::esc

#endif // PWM_ESC_DRIVER_HPP
