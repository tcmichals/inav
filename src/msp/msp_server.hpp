/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Configurable MSP Serial & TCP Server Header
 */

#ifndef MSP_SERVER_HPP
#define MSP_SERVER_HPP

#include "msp_protocol.hpp"
#include <cstdint>
#include <array>
#include <span>

namespace abstractx::msp {

enum class TransportMode : uint8_t {
    Disabled = 0,
    Serial   = 1,
    Tcp      = 2,
    Both     = 3
};

struct MspServerConfig {
    TransportMode mode{TransportMode::Tcp};
    uint16_t tcp_port{5760}; // Default iNav Configurator SITL TCP port
    uint32_t serial_baud{115200};
};

class MspServer {
public:
    explicit MspServer(const MspServerConfig& config = {}) noexcept : config_(config) {}

    bool start() noexcept;
    void stop() noexcept;
    void poll() noexcept;

    bool is_running() const noexcept { return running_; }

private:
    MspServerConfig config_{};
    bool running_{false};
    int server_fd_{-1};
    int client_fd_{-1};

    void process_incoming_stream(const uint8_t* data, size_t len) noexcept;
};

} // namespace abstractx::msp

#endif // MSP_SERVER_HPP
