/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MSP Server — Templated on MspTransport Concept
 *
 * Portable server logic. The transport (Boost.Asio or lwIP) is injected
 * via the template parameter. Zero #ifdefs in protocol handling.
 */

#ifndef MSP_SERVER_HPP
#define MSP_SERVER_HPP

#include "msp_protocol.hpp"
#include "msp_transport.hpp"
#include <cstdint>
#include <atomic>

namespace abstractx::msp {

// Freestanding, zero-heap spinlock using std::atomic_flag (C++20)
struct SpinLock {
    std::atomic_flag flag = ATOMIC_FLAG_INIT;

    void lock() noexcept {
        while (flag.test_and_set(std::memory_order_acquire)) {
#if defined(__x86_64__) || defined(_M_X64)
            __builtin_ia32_pause();
#endif
        }
    }

    void unlock() noexcept {
        flag.clear(std::memory_order_release);
    }
};

struct SpinLockGuard {
    SpinLock& l_;
    explicit SpinLockGuard(SpinLock& l) noexcept : l_(l) { l_.lock(); }
    ~SpinLockGuard() noexcept { l_.unlock(); }
};

template <MspTransport Transport>
class MspServer {
public:
    explicit MspServer(uint16_t port = 5760) noexcept : port_(port) {
        // Wire transport's byte callback into our parser
        transport_.set_on_bytes_received(
            [this](std::span<const uint8_t> data) {
                on_bytes_received(data);
            });
    }

    bool start() noexcept {
        return transport_.start(port_);
    }

    void stop() noexcept {
        transport_.stop();
    }

    // Non-blocking poll — call from flight loop each tick
    void poll() noexcept {
        transport_.poll();
    }

    bool is_running() const noexcept {
        return transport_.has_client();
    }

    // Update live state reference — called by flight loop with current EKF3/sensor data.
    // Preserves received RC channel overrides across telemetry ticks.
    void update_live_state(const MspLiveState& state) noexcept {
        SpinLockGuard guard(lock_);
        auto rc_copy = live_state_.rc_channels;
        live_state_ = state;
        live_state_.rc_channels = rc_copy;
    }

    // Read current 16-channel RC pulse widths (1000..2000 µs)
    [[nodiscard]] std::array<uint16_t, 16> rc_channels() const noexcept {
        SpinLockGuard guard(const_cast<SpinLock&>(lock_));
        return live_state_.rc_channels;
    }

    // Read full live state snapshot
    [[nodiscard]] MspLiveState live_state() const noexcept {
        SpinLockGuard guard(const_cast<SpinLock&>(lock_));
        return live_state_;
    }

    Transport& transport() noexcept { return transport_; }

private:
    Transport transport_;
    MspV2FrameParser parser_;
    MspLiveState live_state_;
    SpinLock lock_;
    uint16_t port_;

    void on_bytes_received(std::span<const uint8_t> data) noexcept {
        parser_.feed(data);

        while (auto parsed = parser_.next_frame()) {
            MspFrame response{};
            Cmd cmd = static_cast<Cmd>(parsed->command);

            bool handled = false;
            {
                SpinLockGuard guard(lock_);
                handled = MspEngine::process_command(cmd, parsed->payload_span(), response, live_state_);
            }

            if (handled) {
                // Serialize response in same protocol version as request
                if (parsed->is_v2) {
                    auto wire = MspFrameSerializer::serialize_v2(response);
                    transport_.send(std::span<const uint8_t>(wire.data.data(), wire.len));
                } else {
                    auto wire = MspFrameSerializer::serialize_v1(response);
                    transport_.send(std::span<const uint8_t>(wire.data.data(), wire.len));
                }
            }
            // Unknown commands are silently dropped (per MSP spec)
        }
    }
};

} // namespace abstractx::msp

#endif // MSP_SERVER_HPP
