/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Futaba SBUS / SBUS2 Receiver Serial Protocol Parser
 *
 * 100,000 baud inverted UART (8E2).
 * Zero dynamic memory allocations. 25-byte frame bit-unpacking engine.
 */

#ifndef SBUS_HPP
#define SBUS_HPP

#include "rc_driver.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace abstractx::drivers::rc {

class SbusParser {
public:
    static constexpr uint8_t HEADER_BYTE = 0x0F;
    static constexpr size_t FRAME_LEN = 25;

    constexpr SbusParser() noexcept = default;

    void reset() noexcept {
        idx_ = 0;
        in_frame_ = false;
    }

    // Process incoming byte stream. Returns true if valid channels were updated.
    bool parse_byte(uint8_t byte) noexcept {
        if (!in_frame_) {
            if (byte == HEADER_BYTE) {
                frame_[0] = byte;
                idx_ = 1;
                in_frame_ = true;
            }
            return false;
        }

        frame_[idx_++] = byte;

        if (idx_ >= FRAME_LEN) {
            in_frame_ = false;
            // Validate footer byte (0x00 standard, 0x04 SBUS2)
            if (frame_[24] == 0x00 || (frame_[24] & 0x0F) == 0x04) {
                return decode_frame();
            }
        }

        return false;
    }

    const RcChannels& channels() const noexcept { return rc_channels_; }

private:
    std::array<uint8_t, FRAME_LEN> frame_{};
    size_t idx_{0};
    bool in_frame_{false};
    RcChannels rc_channels_{};

    bool decode_frame() noexcept {
        // Bit-unpack 16 11-bit channel values from 22 payload bytes (bytes 1..22)
        const uint8_t* p = &frame_[1];

        uint16_t raw[16];
        raw[0]  = static_cast<uint16_t>((p[0]       | (p[1] << 8)) & 0x07FF);
        raw[1]  = static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
        raw[2]  = static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
        raw[3]  = static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x07FF);
        raw[4]  = static_cast<uint16_t>(((p[5] >> 4) | (p[6] << 4)) & 0x07FF);
        raw[5]  = static_cast<uint16_t>(((p[6] >> 7) | (p[7] << 1) | (p[8] << 9)) & 0x07FF);
        raw[6]  = static_cast<uint16_t>(((p[8] >> 2) | (p[9] << 6)) & 0x07FF);
        raw[7]  = static_cast<uint16_t>(((p[9] >> 5) | (p[10] << 3)) & 0x07FF);
        raw[8]  = static_cast<uint16_t>((p[11]      | (p[12] << 8)) & 0x07FF);
        raw[9]  = static_cast<uint16_t>(((p[12] >> 3)| (p[13] << 5)) & 0x07FF);
        raw[10] = static_cast<uint16_t>(((p[13] >> 6)| (p[14] << 2) | (p[15] << 10)) & 0x07FF);
        raw[11] = static_cast<uint16_t>(((p[15] >> 1)| (p[16] << 7)) & 0x07FF);
        raw[12] = static_cast<uint16_t>(((p[16] >> 4)| (p[17] << 4)) & 0x07FF);
        raw[13] = static_cast<uint16_t>(((p[17] >> 7)| (p[18] << 1) | (p[19] << 9)) & 0x07FF);
        raw[14] = static_cast<uint16_t>(((p[19] >> 2)| (p[20] << 6)) & 0x07FF);
        raw[15] = static_cast<uint16_t>(((p[20] >> 5)| (p[21] << 3)) & 0x07FF);

        // Scale SBUS 11-bit raw (173..1812) to standard PWM (1000..2000 us)
        for (int i = 0; i < 16; ++i) {
            int32_t val = raw[i];
            if (val < 173) val = 173;
            if (val > 1812) val = 1812;
            rc_channels_.channels[i] = static_cast<uint16_t>(1000 + (val - 173) * 1000 / 1639);
        }

        // Parse flags byte (frame_[23])
        uint8_t flags = frame_[23];
        rc_channels_.failsafe = (flags & 0x08) != 0; // Bit 3: Failsafe
        rc_channels_.connected = !rc_channels_.failsafe;

        return true;
    }
};

} // namespace abstractx::drivers::rc

#endif // SBUS_HPP
