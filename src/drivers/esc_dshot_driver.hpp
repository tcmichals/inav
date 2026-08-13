/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Platform-Agnostic PCIe BAR DShot / OneShot ESC Driver
 */

#ifndef ESC_DSHOT_DRIVER_HPP
#define ESC_DSHOT_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers {

class EscDshotDriver {
public:
    constexpr EscDshotDriver() noexcept = default;

    // Create 64B Memory Write TLP for dispatching DShot motor commands over PCIe BAR
    static Tlp64 make_motor_write_tlp(uint8_t motor_idx, uint16_t command, uint8_t tag) noexcept {
        uint32_t addr = bar::EscBase + reg::esc::Motor1 + (motor_idx * 4);
        return Tlp64::make_mem_write(addr, static_cast<uint32_t>(command), tag);
    }
};

} // namespace abstractx::drivers

#endif // ESC_DSHOT_DRIVER_HPP
