/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - BareCTF Live UDP/TCP Binary Trace Stream Server Implementation
 */

#include "ctf_stream_server.hpp"
#include <iostream>
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace abstractx::logging {

bool CtfStreamServer::start() noexcept {
    if (config_.protocol == CtfTransportProtocol::Disabled) return true;

#if defined(__linux__) || defined(__APPLE__)
    if (config_.protocol == CtfTransportProtocol::UdpBroadcast) {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) return false;

        int broadcast_opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt));

        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        running_ = true;
    } else if (config_.protocol == CtfTransportProtocol::TcpServer) {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) return false;

        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(config_.port);

        if (bind(socket_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        if (listen(socket_fd_, 1) < 0) {
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        running_ = true;
    }
#else
    running_ = true;
#endif

    return running_;
}

void CtfStreamServer::stop() noexcept {
#if defined(__linux__) || defined(__APPLE__)
    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
    }
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
#endif
    running_ = false;
}

void CtfStreamServer::poll_and_stream(SpscTlpRing<64>& log_ring) noexcept {
    if (!running_) return;

#if defined(__linux__) || defined(__APPLE__)
    if (config_.protocol == CtfTransportProtocol::TcpServer && socket_fd_ >= 0 && client_fd_ < 0) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        int new_fd = accept(socket_fd_, (struct sockaddr*)&client_addr, &addr_len);
        if (new_fd >= 0) {
            client_fd_ = new_fd;
            int flags = fcntl(client_fd_, F_GETFL, 0);
            fcntl(client_fd_, F_SETFL, flags | O_NONBLOCK);
        }
    }

    while (!log_ring.empty()) {
        auto opt_tlp = log_ring.pop();
        if (!opt_tlp.has_value()) break;

        const auto& tlp = opt_tlp.value();

        if (config_.protocol == CtfTransportProtocol::UdpBroadcast && socket_fd_ >= 0) {
            sockaddr_in udp_addr{};
            udp_addr.sin_family = AF_INET;
            udp_addr.sin_port = htons(config_.port);
            inet_pton(AF_INET, config_.target_ip.data(), &udp_addr.sin_addr);

            sendto(socket_fd_, &tlp.wire, sizeof(TlpWire64), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
        } else if (config_.protocol == CtfTransportProtocol::TcpServer && client_fd_ >= 0) {
            ssize_t written = write(client_fd_, &tlp.wire, sizeof(TlpWire64));
            if (written <= 0) {
                close(client_fd_);
                client_fd_ = -1;
            }
        }
    }
#endif
}

} // namespace abstractx::logging
