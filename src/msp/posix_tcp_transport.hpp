/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Native POSIX TCP Transport for Linux (Zero External Dependencies)
 *
 * Satisfies MspTransport concept. Uses standard POSIX sockets with a dedicated background thread.
 * Compatible with SITL Desktop, Linux SBC, and HIL test harnesses.
 */

#ifndef POSIX_TCP_TRANSPORT_HPP
#define POSIX_TCP_TRANSPORT_HPP

#if defined(__linux__) || defined(__APPLE__) || defined(__unix__)

#include "msp_transport.hpp"
#include <functional>
#include <thread>
#include <atomic>
#include <cstring>
#include <array>
#include <span>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>

namespace abstractx::msp {

class PosixTcpTransport {
public:
    using OnBytesReceivedCb = std::function<void(std::span<const uint8_t>)>;

    PosixTcpTransport() noexcept = default;
    ~PosixTcpTransport() { stop(); }

    void set_on_bytes_received(OnBytesReceivedCb cb) noexcept {
        on_bytes_received_ = std::move(cb);
    }

    bool start(uint16_t port) noexcept {
        stop();

        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) { return false; }

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, 1) < 0) {
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        running_.store(true, std::memory_order_relaxed);

        worker_thread_ = std::thread([this]() {
            run_worker();
        });

        return true;
    }

    void stop() noexcept {
        if (!running_.exchange(false, std::memory_order_relaxed)) {
            return;
        }

        if (client_fd_ >= 0) {
            shutdown(client_fd_, SHUT_RDWR);
            close(client_fd_);
            client_fd_ = -1;
        }

        if (server_fd_ >= 0) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        client_connected_.store(false, std::memory_order_relaxed);
    }

    void poll() noexcept {}

    bool send(std::span<const uint8_t> data) noexcept {
        if (!client_connected_.load(std::memory_order_relaxed) || client_fd_ < 0) {
            return false;
        }
        ssize_t sent = ::send(client_fd_, data.data(), data.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            close_client();
            return false;
        }
        return (sent == static_cast<ssize_t>(data.size()));
    }

    [[nodiscard]] bool has_client() const noexcept {
        return client_connected_.load(std::memory_order_relaxed);
    }

private:
    int server_fd_{-1};
    int client_fd_{-1};
    std::atomic<bool> running_{false};
    std::atomic<bool> client_connected_{false};
    std::thread worker_thread_;
    OnBytesReceivedCb on_bytes_received_;

    void close_client() noexcept {
        if (client_fd_ >= 0) {
            close(client_fd_);
            client_fd_ = -1;
        }
        client_connected_.store(false, std::memory_order_relaxed);
    }

    void run_worker() noexcept {
        std::array<uint8_t, 512> rx_buf{};

        while (running_.load(std::memory_order_relaxed)) {
            if (client_fd_ < 0) {
                // Wait for client connection with timeout
                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(server_fd_, &read_fds);

                timeval tv{0, 50000}; // 50ms timeout
                int sel = select(server_fd_ + 1, &read_fds, nullptr, nullptr, &tv);

                if (sel > 0 && FD_ISSET(server_fd_, &read_fds)) {
                    sockaddr_in client_addr{};
                    socklen_t client_len = sizeof(client_addr);
                    int new_client = accept(server_fd_, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);

                    if (new_client >= 0) {
                        int nodelay = 1;
                        setsockopt(new_client, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
                        client_fd_ = new_client;
                        client_connected_.store(true, std::memory_order_relaxed);
                    }
                }
                continue;
            }

            // Client is connected - check for readable data
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(client_fd_, &read_fds);

            timeval tv{0, 20000}; // 20ms timeout
            int sel = select(client_fd_ + 1, &read_fds, nullptr, nullptr, &tv);

            if (sel > 0 && FD_ISSET(client_fd_, &read_fds)) {
                ssize_t bytes_read = recv(client_fd_, rx_buf.data(), rx_buf.size(), 0);
                if (bytes_read > 0) {
                    if (on_bytes_received_) {
                        on_bytes_received_(std::span<const uint8_t>(rx_buf.data(), static_cast<size_t>(bytes_read)));
                    }
                } else if (bytes_read == 0 || (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    close_client();
                }
            }
        }
    }
};

static_assert(MspTransport<PosixTcpTransport>, "PosixTcpTransport must satisfy MspTransport concept");

} // namespace abstractx::msp

#endif // __linux__ || __APPLE__ || __unix__

#endif // POSIX_TCP_TRANSPORT_HPP
