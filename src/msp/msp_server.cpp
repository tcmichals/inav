/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Configurable MSP Serial & TCP Server Implementation
 */

#include "msp_server.hpp"
#include <iostream>
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace abstractx::msp {

bool MspServer::start() noexcept {
    if (config_.mode == TransportMode::Disabled) return true;

#if defined(__linux__) || defined(__APPLE__)
    if (config_.mode == TransportMode::Tcp || config_.mode == TransportMode::Both) {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) return false;

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Make non-blocking
        int flags = fcntl(server_fd_, F_GETFL, 0);
        fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(config_.tcp_port);

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, 1) < 0) {
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        running_ = true;
    }
#else
    running_ = true;
#endif

    return running_;
}

void MspServer::stop() noexcept {
#if defined(__linux__) || defined(__APPLE__)
    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
    }
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
#endif
    running_ = false;
}

void MspServer::poll() noexcept {
    if (!running_) return;

#if defined(__linux__) || defined(__APPLE__)
    if (server_fd_ >= 0 && client_fd_ < 0) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        int new_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);
        if (new_fd >= 0) {
            client_fd_ = new_fd;
            int flags = fcntl(client_fd_, F_GETFL, 0);
            fcntl(client_fd_, F_SETFL, flags | O_NONBLOCK);
        }
    }

    if (client_fd_ >= 0) {
        uint8_t rx_buf[256];
        ssize_t bytes_read = read(client_fd_, rx_buf, sizeof(rx_buf));
        if (bytes_read > 0) {
            process_incoming_stream(rx_buf, static_cast<size_t>(bytes_read));
        } else if (bytes_read == 0) {
            close(client_fd_);
            client_fd_ = -1;
        }
    }
#endif
}

void MspServer::process_incoming_stream(const uint8_t* data, size_t len) noexcept {
    if (len < 6) return;

    // Check for $M< MSP Header
    if (data[0] == '$' && data[1] == 'M' && data[2] == '<') {
        MspFrame rx_frame{};
        rx_frame.command = static_cast<Cmd>(data[4]);

        MspFrame tx_frame{};
        if (MspEngine::process_command(rx_frame.command, std::span<const uint8_t>(data + 5, len - 5), tx_frame)) {
            uint8_t tx_buf[256];
            tx_buf[0] = '$';
            tx_buf[1] = 'M';
            tx_buf[2] = '>';
            tx_buf[3] = static_cast<uint8_t>(tx_frame.payload_len);
            tx_buf[4] = static_cast<uint8_t>(tx_frame.command);

            uint8_t checksum = tx_buf[3] ^ tx_buf[4];
            for (size_t i = 0; i < tx_frame.payload_len; ++i) {
                tx_buf[5 + i] = tx_frame.payload[i];
                checksum ^= tx_frame.payload[i];
            }
            tx_buf[5 + tx_frame.payload_len] = checksum;

#if defined(__linux__) || defined(__APPLE__)
            if (client_fd_ >= 0) {
                write(client_fd_, tx_buf, 6 + tx_frame.payload_len);
            }
#endif
        }
    }
}

} // namespace abstractx::msp
