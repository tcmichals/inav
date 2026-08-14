/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Boost.Asio Background Thread TCP Transport for Linux (Shared SITL & SBC)
 *
 * Satisfies MspTransport concept. Runs io_context.run() on a dedicated background thread (Core 0/1/2).
 * Flight engine thread on Core 3 does ZERO polling or network work!
 */

#ifndef BOOST_ASIO_TRANSPORT_HPP
#define BOOST_ASIO_TRANSPORT_HPP

#if defined(__linux__) || defined(__APPLE__)

#include "msp_transport.hpp"
#include <boost/asio.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <cstring>
#include <memory>
#include <span>

namespace abstractx::msp {

class BoostAsioTransport {
public:
    using OnBytesReceivedCb = std::function<void(std::span<const uint8_t>)>;

    BoostAsioTransport() noexcept
        : acceptor_(io_ctx_), socket_(io_ctx_) {}

    ~BoostAsioTransport() { stop(); }

    // Set callback invoked when bytes arrive from client
    void set_on_bytes_received(OnBytesReceivedCb cb) noexcept {
        on_bytes_received_ = std::move(cb);
    }

    bool start(uint16_t port) noexcept {
        try {
            boost::asio::ip::tcp::endpoint ep(boost::asio::ip::tcp::v4(), port);
            acceptor_.open(ep.protocol());
            acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
            acceptor_.bind(ep);
            acceptor_.listen(1);
            start_accept();

            // Create work guard so io_context doesn't exit when idle
            work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(
                io_ctx_.get_executor());

            running_ = true;

            // Spawn dedicated background networking thread (Core 0 / non-RT)
            io_thread_ = std::thread([this]() {
                try {
                    io_ctx_.run();
                } catch (...) {}
            });

            return true;
        } catch (...) {
            return false;
        }
    }

    void stop() noexcept {
        if (!running_) return;
        running_ = false;

        boost::system::error_code ec;
        if (socket_.is_open()) {
            socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
            socket_.close(ec);
        }
        if (acceptor_.is_open()) {
            acceptor_.close(ec);
        }

        work_guard_.reset();
        io_ctx_.stop();

        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        client_connected_ = false;
    }

    // No-op — io_context runs on dedicated background thread
    void poll() noexcept {
        // Boost.Asio events are processed asynchronously on background io_thread_
    }

    bool send(std::span<const uint8_t> data) noexcept {
        if (!client_connected_ || !socket_.is_open()) return false;
        try {
            boost::asio::write(socket_, boost::asio::buffer(data.data(), data.size()));
            return true;
        } catch (...) {
            handle_disconnect();
            return false;
        }
    }

    bool has_client() const noexcept {
        return client_connected_;
    }

private:
    boost::asio::io_context io_ctx_;
    std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    std::thread io_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> client_connected_{false};
    std::array<uint8_t, 512> rx_buf_{};
    OnBytesReceivedCb on_bytes_received_;

    void start_accept() noexcept {
        acceptor_.async_accept(
            [this](boost::system::error_code ec, boost::asio::ip::tcp::socket new_socket) {
                if (!ec) {
                    if (client_connected_ && socket_.is_open()) {
                        boost::system::error_code close_ec;
                        socket_.close(close_ec);
                    }
                    socket_ = std::move(new_socket);
                    socket_.set_option(boost::asio::ip::tcp::no_delay(true));
                    client_connected_ = true;
                    start_read();
                }
                if (running_ && acceptor_.is_open()) {
                    start_accept();
                }
            });
    }

    void start_read() noexcept {
        if (!client_connected_ || !socket_.is_open()) return;

        socket_.async_read_some(
            boost::asio::buffer(rx_buf_),
            [this](boost::system::error_code ec, std::size_t bytes_read) {
                if (!ec && bytes_read > 0) {
                    if (on_bytes_received_) {
                        on_bytes_received_(std::span<const uint8_t>(rx_buf_.data(), bytes_read));
                    }
                    start_read();
                } else {
                    handle_disconnect();
                }
            });
    }

    void handle_disconnect() noexcept {
        client_connected_ = false;
        boost::system::error_code ec;
        if (socket_.is_open()) {
            socket_.close(ec);
        }
    }
};

// Static assertion: BoostAsioTransport satisfies MspTransport concept
static_assert(MspTransport<BoostAsioTransport>,
    "BoostAsioTransport must satisfy MspTransport concept");

} // namespace abstractx::msp

#endif // __linux__ || __APPLE__

#endif // BOOST_ASIO_TRANSPORT_HPP
