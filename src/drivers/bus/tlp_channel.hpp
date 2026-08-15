/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Asynchronous TLP Communication Channel
 *
 * Top-level drivers use TlpChannel to emit 64-byte Tlp64 requests (MemRead, MemWrite)
 * to the Bottom-Half PCIe TLP Scheduler and asynchronously await completions via
 * zero-allocation C++20 awaiters.
 *
 * MISRA C++:2023 Rules:
 *   - Zero dynamic heap allocation
 *   - All methods [[nodiscard]] noexcept
 *   - Bounds-checked std::span and std::array buffers
 */

#ifndef TLP_CHANNEL_HPP
#define TLP_CHANNEL_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include "coroutine_task.hpp"
#include <cstdint>
#include <span>
#include <optional>
#include <algorithm>

namespace abstractx::drivers::bus {

class TlpChannel;

// Custom C++20 Awaiter: Single Register Read
struct RegReadAwaiter {
    TlpChannel& channel;
    uint32_t bar_base;
    uint8_t reg;
    uint8_t tag;

    [[nodiscard]] constexpr bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<>) noexcept;
    [[nodiscard]] std::optional<uint8_t> await_resume() noexcept;
};

// Custom C++20 Awaiter: Single Register Write
struct RegWriteAwaiter {
    TlpChannel& channel;
    uint32_t bar_base;
    uint8_t reg;
    uint8_t val;
    uint8_t tag;
    bool success{false};

    [[nodiscard]] constexpr bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<>) noexcept;
    [[nodiscard]] constexpr bool await_resume() const noexcept { return success; }
};

// Custom C++20 Awaiter: Multi-Byte Burst Read
struct RegBurstReadAwaiter {
    TlpChannel& channel;
    uint32_t bar_base;
    uint8_t reg;
    std::span<uint8_t> out_buffer;
    uint8_t tag;

    [[nodiscard]] constexpr bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<>) noexcept;
    [[nodiscard]] bool await_resume() noexcept;
};

class TlpChannel {
public:
    explicit TlpChannel(SpscTlpRing<64u>& outbound_ring,
                        SpscTlpRing<64u>& inbound_ring) noexcept
        : outbound_{outbound_ring}, inbound_{inbound_ring} {}

    // Submit an Outbound TLP Request (MemRead, MemWrite, Config)
    [[nodiscard]] bool send_request(const Tlp64& req) noexcept {
        return outbound_.push(req);
    }

    // Try reading an Inbound TLP Completion
    [[nodiscard]] std::optional<Tlp64> poll_completion() noexcept {
        return inbound_.pop();
    }

    // Asynchronous register write: returns custom Awaiter
    [[nodiscard]] RegWriteAwaiter async_write_reg(uint32_t bar_base, uint8_t reg, uint8_t val) noexcept {
        return RegWriteAwaiter{*this, bar_base, reg, val, next_tag_++};
    }

    // Asynchronous single register read: returns custom Awaiter
    [[nodiscard]] RegReadAwaiter async_read_reg(uint32_t bar_base, uint8_t reg) noexcept {
        return RegReadAwaiter{*this, bar_base, reg, next_tag_++};
    }

    // Asynchronous multi-byte burst read: returns custom Awaiter
    [[nodiscard]] RegBurstReadAwaiter async_read_burst(uint32_t bar_base, uint8_t reg, std::span<uint8_t> out_buffer) noexcept {
        return RegBurstReadAwaiter{*this, bar_base, reg, out_buffer, next_tag_++};
    }

    // Direct synchronous loopback (used for fast SITL / unit tests without separate worker thread)
    void process_sync_loopback() noexcept {
        while (auto opt_req = outbound_.pop()) {
            Tlp64 resp = *opt_req;
            resp.wire.type = static_cast<uint8_t>(TlpType::Completion);
            (void)inbound_.push(resp);
        }
    }

private:
    SpscTlpRing<64u>& outbound_;
    SpscTlpRing<64u>& inbound_;
    uint8_t next_tag_{0u};
};

inline void RegReadAwaiter::await_suspend(std::coroutine_handle<>) noexcept {
    Tlp64 tlp = Tlp64::make_mem_read(bar_base | static_cast<uint32_t>(reg), tag);
    tlp.wire.length = 1u;
    (void)channel.send_request(tlp);
}

inline std::optional<uint8_t> RegReadAwaiter::await_resume() noexcept {
    while (auto opt_comp = channel.poll_completion()) {
        if (opt_comp->tag() == tag) {
            return opt_comp->wire.payload[0];
        }
    }
    return std::nullopt;
}

inline void RegWriteAwaiter::await_suspend(std::coroutine_handle<>) noexcept {
    Tlp64 tlp = Tlp64::make_mem_write(bar_base | static_cast<uint32_t>(reg), static_cast<uint32_t>(val), tag);
    tlp.wire.payload[0] = val;
    tlp.wire.length = 1u;
    success = channel.send_request(tlp);
}

inline void RegBurstReadAwaiter::await_suspend(std::coroutine_handle<>) noexcept {
    Tlp64 tlp = Tlp64::make_mem_read(bar_base | static_cast<uint32_t>(reg), tag);
    tlp.wire.length = static_cast<uint16_t>(out_buffer.size());
    (void)channel.send_request(tlp);
}

inline bool RegBurstReadAwaiter::await_resume() noexcept {
    while (auto opt_comp = channel.poll_completion()) {
        if (opt_comp->tag() == tag) {
            const size_t copy_len = std::min(out_buffer.size(), static_cast<size_t>(opt_comp->wire.length));
            for (size_t i = 0; i < copy_len; ++i) {
                out_buffer[i] = opt_comp->wire.payload[i];
            }
            return true;
        }
    }
    return false;
}

} // namespace abstractx::drivers::bus

#endif // TLP_CHANNEL_HPP
