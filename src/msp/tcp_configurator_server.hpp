/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Non-Blocking TCP Socket Server for iNav Configurator (Port 5760)
 */

#ifndef TCP_CONFIGURATOR_SERVER_HPP
#define TCP_CONFIGURATOR_SERVER_HPP

#if defined(__linux__)
#include "msp_server.hpp"
#include "msp_protocol.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <thread>
#include <atomic>
#include <array>

namespace abstractx::msp {

class TcpConfiguratorServer {
public:
    static constexpr uint16_t CONFIGURATOR_PORT = 5760;

    // Starts background non-blocking TCP socket server listening on port 5760
    static void start_server(std::atomic<bool>& running) noexcept {
        std::thread([&running]() {
            int server_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (server_fd < 0) return;

            int opt = 1;
            setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(CONFIGURATOR_PORT);

            if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
                close(server_fd);
                return;
            }

            listen(server_fd, 3);
            
            // Set server socket to non-blocking
            int flags = fcntl(server_fd, F_GETFL, 0);
            fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

            while (running.load(std::memory_order_relaxed)) {
                sockaddr_in client_addr{};
                socklen_t addrlen = sizeof(client_addr);
                int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &addrlen);

                if (client_fd >= 0) {
                    handle_configurator_client(client_fd, running);
                    close(client_fd);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            close(server_fd);
        }).detach();
    }

private:
    static void handle_configurator_client(int client_fd, std::atomic<bool>& running) noexcept {
        std::array<uint8_t, 512> rx_buf{};
        MspFrame tx_frame{};

        while (running.load(std::memory_order_relaxed)) {
            ssize_t bytes_read = read(client_fd, rx_buf.data(), rx_buf.size());
            if (bytes_read > 0) {
                // Parse MSP command & generate response frame
                for (ssize_t i = 0; i < bytes_read; ++i) {
                    if (rx_buf[static_cast<size_t>(i)] == '$') {
                        // Example MSP API version query handling
                        MspEngine::process_command(Cmd::ApiVersion, {}, tx_frame);
                        write(client_fd, tx_frame.payload.data(), tx_frame.payload_len);
                    }
                }
            } else if (bytes_read == 0) {
                break; // Client disconnected
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
};

} // namespace abstractx::msp

#endif // __linux__

#endif // TCP_CONFIGURATOR_SERVER_HPP
