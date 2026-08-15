/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `AbstractX` - Fixed 64-Byte Transaction Layer Packet (TLP64 / ASP-64) Wire Format
 */

#ifndef ASP_TLP64_HPP
#define ASP_TLP64_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <span>

namespace abstractx {

enum class TlpType : uint8_t {
    DmaStream   = 0x00u,
    MemRead     = 0x01u,
    MemWrite    = 0x02u,
    Completion  = 0x03u,
    Interrupt   = 0x04u,
    Config      = 0x05u
};

enum class Channel : uint8_t {
    Telemetry   = 0x00u,
    Logging     = 0x01u,
    Command     = 0x02u,
    Actuator    = 0x03u
};

#pragma pack(push, 1)
struct alignas(64) TlpWire64 {
    uint8_t  type{static_cast<uint8_t>(TlpType::MemWrite)};
    uint8_t  channel{static_cast<uint8_t>(Channel::Telemetry)};
    uint8_t  tag{0u};
    uint8_t  flags{0u};
    uint16_t sequence{0u};
    uint16_t length{64u};
    uint32_t target_address{0u};
    uint64_t timestamp_ns{0u};
    uint8_t  payload[44]{0u};
};
#pragma pack(pop)

static_assert(sizeof(TlpWire64) == 64u, "TlpWire64 MUST be exactly 64 bytes");

class alignas(64) Tlp64 {
public:
    TlpWire64 wire{};

    constexpr Tlp64() noexcept = default;

    static constexpr Tlp64 make_mem_write(uint32_t addr, uint32_t val, uint8_t tag = 0u) noexcept {
        Tlp64 tlp{};
        tlp.wire.type = static_cast<uint8_t>(TlpType::MemWrite);
        tlp.wire.channel = static_cast<uint8_t>(Channel::Actuator);
        tlp.wire.tag = tag;
        tlp.wire.target_address = addr;
        tlp.wire.payload[0] = static_cast<uint8_t>(val & 0xFFu);
        tlp.wire.payload[1] = static_cast<uint8_t>((val >> 8u) & 0xFFu);
        tlp.wire.payload[2] = static_cast<uint8_t>((val >> 16u) & 0xFFu);
        tlp.wire.payload[3] = static_cast<uint8_t>((val >> 24u) & 0xFFu);
        return tlp;
    }

    static constexpr Tlp64 make_mem_read(uint32_t addr, uint8_t tag = 0u) noexcept {
        Tlp64 tlp{};
        tlp.wire.type = static_cast<uint8_t>(TlpType::MemRead);
        tlp.wire.tag = tag;
        tlp.wire.target_address = addr;
        return tlp;
    }

    [[nodiscard]] constexpr uint32_t target_address() const noexcept {
        return wire.target_address;
    }

    [[nodiscard]] constexpr uint8_t type() const noexcept {
        return wire.type;
    }

    [[nodiscard]] constexpr uint8_t tag() const noexcept {
        return wire.tag;
    }

    [[nodiscard]] constexpr uint16_t length() const noexcept {
        return wire.length;
    }

    [[nodiscard]] constexpr uint64_t timestamp_ns() const noexcept {
        return wire.timestamp_ns;
    }
};

static_assert(sizeof(Tlp64) == 64u, "Tlp64 MUST be exactly 64 bytes");

} // namespace abstractx

#endif // ASP_TLP64_HPP
