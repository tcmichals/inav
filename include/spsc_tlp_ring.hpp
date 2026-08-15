/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `AbstractX` - Lock-Free Single-Producer Single-Consumer (SPSC) TLP64 Ring Buffer
 */

#ifndef SPSC_TLP_RING_HPP
#define SPSC_TLP_RING_HPP

#include "asp_tlp64.hpp"
#include <atomic>
#include <array>
#include <optional>
#include <cstddef>

namespace abstractx {

template <size_t Capacity = 64u>
class alignas(64) SpscTlpRing {
    static_assert((Capacity & (Capacity - 1u)) == 0u, "Capacity MUST be a power of two");

public:
    using value_type = Tlp64;

    constexpr SpscTlpRing() noexcept = default;

    [[nodiscard]] bool push(const Tlp64& item) noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_acquire);

        if ((head - tail) >= Capacity) {
            return false; // Ring full
        }

        buffer_[head & (Capacity - 1u)] = item;
        head_.store(head + 1u, std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::optional<Tlp64> pop() noexcept {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        const size_t head = head_.load(std::memory_order_acquire);

        if (tail == head) {
            return std::nullopt; // Ring empty
        }

        Tlp64 item = buffer_[tail & (Capacity - 1u)];
        tail_.store(tail + 1u, std::memory_order_release);
        return item;
    }

    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] size_t size() const noexcept {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        return (head >= tail) ? (head - tail) : 0u;
    }

    [[nodiscard]] constexpr size_t capacity() const noexcept {
        return Capacity;
    }

    void reset() noexcept {
        head_.store(0u, std::memory_order_relaxed);
        tail_.store(0u, std::memory_order_relaxed);
    }

private:
    alignas(64) std::array<Tlp64, Capacity> buffer_{};
    alignas(64) std::atomic<size_t> head_{0u};
    alignas(64) std::atomic<size_t> tail_{0u};
};

} // namespace abstractx

#endif // SPSC_TLP_RING_HPP
