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

namespace abstractx::msp {

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

    // Update live state reference — called by flight loop with current EKF3/sensor data
    void update_live_state(const MspLiveState& state) noexcept {
        live_state_ = state;
    }

    Transport& transport() noexcept { return transport_; }

private:
    Transport transport_;
    MspV2FrameParser parser_;
    MspLiveState live_state_;
    uint16_t port_;

    void on_bytes_received(std::span<const uint8_t> data) noexcept {
        parser_.feed(data);

        while (auto parsed = parser_.next_frame()) {
            MspFrame response{};
            Cmd cmd = static_cast<Cmd>(parsed->command);

            if (MspEngine::process_command(cmd, parsed->payload_span(), response, live_state_)) {
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
