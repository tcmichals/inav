/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - BareCTF Live UDP/TCP Binary Trace Stream Server
 */

#ifndef CTF_STREAM_SERVER_HPP
#define CTF_STREAM_SERVER_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <array>
#include <string_view>

namespace abstractx::logging {

enum class CtfTransportProtocol : uint8_t {
    Disabled = 0,
    UdpBroadcast = 1,
    TcpServer    = 2
};

struct CtfStreamConfig {
    CtfTransportProtocol protocol{CtfTransportProtocol::UdpBroadcast};
    uint16_t port{19000}; // Default BareCTF live streaming port
    std::string_view target_ip{"255.255.255.255"}; // UDP Broadcast IP
};

class CtfStreamServer {
public:
    explicit CtfStreamServer(const CtfStreamConfig& config = {}) noexcept : config_(config) {}

    bool start() noexcept;
    void stop() noexcept;

    // Stream ready CTF 64B TLPs from SPSC ring over UDP/TCP
    void poll_and_stream(SpscTlpRing<64>& log_ring) noexcept;

    bool is_running() const noexcept { return running_; }

private:
    CtfStreamConfig config_{};
    bool running_{false};
    int socket_fd_{-1};
    int client_fd_{-1};
};

} // namespace abstractx::logging

#endif // CTF_STREAM_SERVER_HPP
