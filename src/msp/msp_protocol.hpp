/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MultiWii Serial Protocol (MSP v1/v2) Frame Processor
 */

#ifndef MSP_PROTOCOL_HPP
#define MSP_PROTOCOL_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace abstractx::msp {

enum class Cmd : uint16_t {
    ApiVersion   = 1,
    FcVariant    = 2,
    FcVersion    = 3,
    BoardInfo    = 4,
    RawImu       = 102,
    Attitude     = 108,
    Pid          = 112,
    SetPid       = 202,
    EepromWrite  = 250,
    Set4WayIf    = 245
};

struct MspFrame {
    uint8_t  direction{'$'}; // '$M<' or '$M>'
    uint8_t  flag{'<'};
    Cmd      command{Cmd::ApiVersion};
    uint16_t payload_len{0};
    std::array<uint8_t, 256> payload{};

    constexpr void reset() noexcept {
        direction = '$';
        flag = '<';
        command = Cmd::ApiVersion;
        payload_len = 0;
    }

    constexpr bool push_u8(uint8_t val) noexcept {
        if (payload_len >= 256) return false;
        payload[payload_len++] = val;
        return true;
    }

    constexpr bool push_u16(uint16_t val) noexcept {
        if (payload_len + 2 > 256) return false;
        payload[payload_len++] = static_cast<uint8_t>(val & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 8) & 0xFF);
        return true;
    }

    constexpr bool push_u32(uint32_t val) noexcept {
        if (payload_len + 4 > 256) return false;
        payload[payload_len++] = static_cast<uint8_t>(val & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 8) & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 16) & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 24) & 0xFF);
        return true;
    }
};

class MspEngine {
public:
    static bool process_command(Cmd cmd, const std::span<const uint8_t>& rx_payload, MspFrame& tx_frame) noexcept;
};

} // namespace abstractx::msp

#endif // MSP_PROTOCOL_HPP
