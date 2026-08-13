/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated SSD1306 OLED Display Driver (128x64 pixels)
 */

#ifndef OLED_SSD1306_DRIVER_HPP
#define OLED_SSD1306_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers::display {

class OledSsd1306 {
public:
    static constexpr uint16_t WIDTH = 128;
    static constexpr uint16_t HEIGHT = 64;

    // Draw string to 128x64 display buffer
    void draw_string(uint8_t x, uint8_t y, const char* str) noexcept {
        (void)x; (void)y; (void)str;
    }

    // Build 64B TLP to flush display frame over Virtual BAR
    Tlp64 make_flush_tlp(uint8_t page, uint8_t tag) const noexcept {
        uint32_t addr = bar::SystemBase + 0xD00 + static_cast<uint32_t>(page * 64);
        return Tlp64::make_mem_write(addr, 0xFFFFFFFF, tag);
    }
};

} // namespace abstractx::drivers::display

#endif // OLED_SSD1306_DRIVER_HPP
