/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated MAX7456 Analog Video On-Screen Display (OSD) Driver
 */

#ifndef OSD_MAX7456_DRIVER_HPP
#define OSD_MAX7456_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>

namespace abstractx::drivers::display {

class OsdMax7456 {
public:
    static constexpr uint8_t COLS = 30;
    static constexpr uint8_t ROWS = 16;

    // Write character to OSD screen position (row, col)
    static Tlp64 write_char(uint8_t row, uint8_t col, uint8_t char_code, uint8_t tag) noexcept {
        uint32_t addr = bar::SystemBase + 0xE00 + static_cast<uint32_t>((row * COLS + col) * 4);
        return Tlp64::make_mem_write(addr, static_cast<uint32_t>(char_code), tag);
    }
};

} // namespace abstractx::drivers::display

#endif // OSD_MAX7456_DRIVER_HPP
