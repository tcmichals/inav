/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `AbstractX` - Fixed 64-Byte Transaction Layer Packet (TLP64 / ASP-64) Wire Format
 */

#ifndef ASP_TLP64_HPP
#define ASP_TLP64_HPP

#include "asp_tlp64.h"
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <span>
#include <algorithm>

namespace abstractx {

enum class TlpType : uint8_t {
    DmaStream   = ASP_TLP_TYPE_DMA_STREAM, // 0x10
    MemRead     = ASP_TLP_TYPE_MEM_RD,     // 0x01
    MemWrite    = ASP_TLP_TYPE_MEM_WR,     // 0x02
    Completion  = ASP_TLP_TYPE_CPL,        // 0x03
    Interrupt   = 0x04u,
    Config      = 0x05u,
    DmaConfig   = ASP_TLP_TYPE_DMA_CFG     // 0x11
};

enum class Channel : uint8_t {
    Telemetry   = ASP_CHANNEL_TELEMETRY,   // 0x02 (IMU/Sensor Auto-DMA stream)
    Logging     = 0x01u,
    Command     = 0x00u,
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

    static Tlp64 make_stream_tx(Channel ch, uint32_t target_addr, std::span<const uint8_t> data, uint8_t tag = 0u) noexcept {
        Tlp64 tlp{};
        tlp.wire.type = ASP_TLP_TYPE_DMA_STREAM;
        tlp.wire.channel = static_cast<uint8_t>(ch);
        tlp.wire.tag = tag;
        tlp.wire.target_address = target_addr;
        const size_t copy_n = std::min(data.size(), sizeof(tlp.wire.payload));
        tlp.wire.length = static_cast<uint16_t>(copy_n);
        std::memcpy(tlp.wire.payload, data.data(), copy_n);
        return tlp;
    }

    static Tlp64 make_hw_fusion_setup(const asp_tlp_auto_dma_cfg_t& cfg, uint8_t tag = 0u) noexcept {
        Tlp64 packet{};
        packet.wire.type = ASP_TLP_TYPE_DMA_CFG;
        packet.wire.tag = tag;
        packet.wire.channel = static_cast<uint8_t>(Channel::Command);
        auto* p = reinterpret_cast<asp_tlp_auto_dma_cfg_t*>(packet.wire.payload);
        *p = cfg;
        p->cmd = ASP_DMA_CMD_SETUP;
        return packet;
    }

    static Tlp64 make_hw_fusion_control(uint8_t channel_id, bool enable, uint8_t tag = 0u) noexcept {
        Tlp64 packet{};
        packet.wire.type = ASP_TLP_TYPE_DMA_CFG;
        packet.wire.tag = tag;
        packet.wire.channel = static_cast<uint8_t>(Channel::Command);
        auto* p = reinterpret_cast<asp_tlp_auto_dma_cfg_t*>(packet.wire.payload);
        p->cmd = enable ? ASP_DMA_CMD_ENABLE : ASP_DMA_CMD_DISABLE;
        p->channel_id = channel_id;
        return packet;
    }

    [[nodiscard]] std::span<const uint8_t> payload() const noexcept {
        return std::span<const uint8_t>(wire.payload, sizeof(wire.payload));
    }

    [[nodiscard]] std::span<uint8_t> payload() noexcept {
        return std::span<uint8_t>(wire.payload, sizeof(wire.payload));
    }

    [[nodiscard]] constexpr Channel channel() const noexcept {
        return static_cast<Channel>(wire.channel);
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
