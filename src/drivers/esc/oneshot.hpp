/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated OneShot125 / OneShot42 / MultiShot ESC Driver
 */

#ifndef ONESHOT_DRIVER_HPP
#define ONESHOT_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <algorithm>

namespace abstractx::drivers::esc {

enum class OneShotType : uint8_t {
    OneShot125 = 0, // 125..250 us
    OneShot42  = 1, // 42..84 us
    MultiShot  = 2  // 5..25 us
};

class OneShot {
public:
    static Tlp64 make_motor_write(uint8_t motor_index, uint16_t command, uint8_t tag, OneShotType mode = OneShotType::OneShot125) noexcept {
        uint32_t target_addr = bar::EscBase + static_cast<uint32_t>(motor_index * 4);
        uint32_t value = 125;

        if (mode == OneShotType::OneShot125) {
            value = std::clamp<uint16_t>(command, 125, 250);
        } else if (mode == OneShotType::OneShot42) {
            value = std::clamp<uint16_t>(command, 42, 84);
        } else {
            value = std::clamp<uint16_t>(command, 5, 25);
        }

        return Tlp64::make_mem_write(target_addr, value, tag);
    }
};

} // namespace abstractx::drivers::esc

#endif // ONESHOT_DRIVER_HPP
