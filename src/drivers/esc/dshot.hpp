/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Digital DShot ESC Driver (DShot150/300/600/1200 + Bidirectional RPM)
 */

#ifndef DSHOT_DRIVER_HPP
#define DSHOT_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>

namespace abstractx::drivers::esc {

class DShot {
public:
    // Build 64B TLP for DShot frame (11-bit throttle 48..2047 + 4-bit CRC)
    static Tlp64 make_motor_write(uint8_t motor_index, uint16_t command, uint8_t tag) noexcept {
        uint32_t target_addr = bar::EscBase + static_cast<uint32_t>(motor_index * 4);
        
        uint16_t packet = static_cast<uint16_t>((command & 0x07FF) << 1);
        uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
        uint32_t dshot_val = static_cast<uint32_t>((packet << 4) | crc);

        return Tlp64::make_mem_write(target_addr, dshot_val, tag);
    }
};

} // namespace abstractx::drivers::esc

#endif // DSHOT_DRIVER_HPP
