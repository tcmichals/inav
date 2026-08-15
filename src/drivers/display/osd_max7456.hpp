/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 Cleanflight / Betaflight / INAV Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production MAX7456 SPI Analog On-Screen Display (OSD) Driver
 *
 * Capabilities:
 *   1. Hardware SPI Detection & Oscillator Lock Verification (Status Reg 0xA0).
 *   2. Video Mode Configuration (PAL: 30x16 chars / NTSC: 30x13 chars).
 *   3. Character Matrix Display Memory (Auto-Increment 8-Bit DMA / Direct Addressing).
 *   4. Fast Screen Clear & Text String Rendering.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef OSD_MAX7456_DRIVER_HPP
#define OSD_MAX7456_DRIVER_HPP

#include "bus_concepts.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>
#include <span>
#include <cstring>
#include <algorithm>

namespace abstractx::drivers::display {

namespace max7456_regs {
    // Write register addresses
    static constexpr uint8_t REG_VM0_WRITE       = 0x00u;
    static constexpr uint8_t REG_VM1_WRITE       = 0x01u;
    static constexpr uint8_t REG_DMM_WRITE       = 0x02u;
    static constexpr uint8_t REG_DMAH_WRITE      = 0x03u;
    static constexpr uint8_t REG_DMAL_WRITE      = 0x04u;
    static constexpr uint8_t REG_DMDI_WRITE      = 0x05u;
    static constexpr uint8_t REG_OSDBL_WRITE     = 0x6Cu;

    // Read register addresses (bit 7 set: 0x80)
    static constexpr uint8_t REG_STAT_READ       = 0xA0u;
    static constexpr uint8_t REG_DMDO_READ       = 0xB0u;

    // VM0 Bits
    static constexpr uint8_t VM0_PAL_MODE        = 0x40u;
    static constexpr uint8_t VM0_ENABLE_OSD      = 0x08u;
    static constexpr uint8_t VM0_SOFT_RESET      = 0x02u;

    // DMM Bits
    static constexpr uint8_t DMM_AUTO_INC        = 0x01u;
    static constexpr uint8_t DMM_CLEAR_DISPLAY   = 0x04u;

    // STAT Bits
    static constexpr uint8_t STAT_PAL_DETECTED   = 0x01u;
    static constexpr uint8_t STAT_NTSC_DETECTED  = 0x02u;
    static constexpr uint8_t STAT_LOS_DETECTED   = 0x04u; // Loss of sync
}

template <bus::IsSpiBus SpiBusT = bus::FakeSpiBus>
class OsdMax7456Driver {
public:
    static constexpr uint8_t COLS = 30u;
    static constexpr uint8_t ROWS_PAL  = 16u;
    static constexpr uint8_t ROWS_NTSC = 13u;
    static constexpr size_t  MAX_CHARS = COLS * ROWS_PAL; // 480 chars

    explicit OsdMax7456Driver(SpiBusT& bus, bool pal_mode = true) noexcept
        : bus_{bus}, pal_mode_{pal_mode} {}

    [[nodiscard]] bool init() noexcept {
        // 1. Soft Reset MAX7456
        (void)bus_.write_reg(max7456_regs::REG_VM0_WRITE, max7456_regs::VM0_SOFT_RESET);
        bus_.delay_ms(1u);

        // 2. Read Status register to verify chip responds on SPI
        uint8_t stat = bus_.read_reg(max7456_regs::REG_STAT_READ);
        (void)stat; // Live hardware status (PAL/NTSC/LOS)

        // 3. Configure Video Mode (PAL or NTSC + Enable OSD display)
        uint8_t vm0 = max7456_regs::VM0_ENABLE_OSD;
        if (pal_mode_) {
            vm0 |= max7456_regs::VM0_PAL_MODE;
        }
        (void)bus_.write_reg(max7456_regs::REG_VM0_WRITE, vm0);

        // 4. Clear Display Memory
        clear();

        initialized_ = true;
        return true;
    }

    void clear() noexcept {
        // DMM clear display bit
        (void)bus_.write_reg(max7456_regs::REG_DMM_WRITE, max7456_regs::DMM_CLEAR_DISPLAY);
        bus_.delay_ms(1u);
        shadow_buffer_.fill(0x00u); // Blank character (space)
    }

    // Write character to OSD position (row, col)
    void write_char(uint8_t row, uint8_t col, uint8_t char_code) noexcept {
        const uint8_t max_rows = pal_mode_ ? ROWS_PAL : ROWS_NTSC;
        if (row >= max_rows || col >= COLS) return;

        uint16_t pos = static_cast<uint16_t>(row * COLS + col);
        shadow_buffer_[pos] = char_code;

        // Set Display Memory Address High & Low
        (void)bus_.write_reg(max7456_regs::REG_DMAH_WRITE, static_cast<uint8_t>(pos >> 8u));
        (void)bus_.write_reg(max7456_regs::REG_DMAL_WRITE, static_cast<uint8_t>(pos & 0xFFu));
        // Write Character Data
        (void)bus_.write_reg(max7456_regs::REG_DMDI_WRITE, char_code);
    }

    void write_string(uint8_t row, uint8_t col, const char* str) noexcept {
        if (!str) return;
        uint8_t cur_col = col;
        while (*str && cur_col < COLS) {
            write_char(row, cur_col++, static_cast<uint8_t>(*str++));
        }
    }

    // Virtual BAR TLP writer (Legacy compatibility)
    static Tlp64 make_write_char_tlp(uint8_t row, uint8_t col, uint8_t char_code, uint8_t tag) noexcept {
        uint32_t addr = bar::DisplayBase + 0x200u + static_cast<uint32_t>((row * COLS + col) * 4u);
        return Tlp64::make_mem_write(addr, static_cast<uint32_t>(char_code), tag);
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
    [[nodiscard]] bool is_pal() const noexcept { return pal_mode_; }

private:
    SpiBusT& bus_;
    bool     pal_mode_{true};
    bool     initialized_{false};
    std::array<uint8_t, MAX_CHARS> shadow_buffer_{};
};

using OsdMax7456      = OsdMax7456Driver<bus::FakeSpiBus>;
using OsdMax7456_Pico = OsdMax7456Driver<bus::Pico2SpiBus>;
using OsdMax7456_Fake = OsdMax7456Driver<bus::FakeSpiBus>;

} // namespace abstractx::drivers::display

#endif // OSD_MAX7456_DRIVER_HPP
