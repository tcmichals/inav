/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Boost.Asio Background Thread Serial Port Transport for Linux
 *
 * Supports hardware UARTs (/dev/ttyUSB0, /dev/ttyAMA0, /dev/ttyTHS1, /dev/ttyACM0)
 * at 115200 to 921600+ baud with zero blocking on the real-time flight loop.
 */

#ifndef BOOST_ASIO_SERIAL_TRANSPORT_HPP
#define BOOST_ASIO_SERIAL_TRANSPORT_HPP

#if defined(__linux__) || defined(__APPLE__)

#include "msp_transport.hpp"
#include <boost/asio.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <string>
#include <memory>
#include <span>

namespace abstractx::msp {

class BoostAsioSerialTransport {
public:
    using OnBytesReceivedCb = std::function<void(std::span<const uint8_t>)>;

    BoostAsioSerialTransport() noexcept
        : serial_(io_ctx_) {}

    ~BoostAsioSerialTransport() { stop(); }

    void set_on_bytes_received(OnBytesReceivedCb cb) noexcept {
        on_bytes_received_ = std::move(cb);
    }

    // Default start implementing MspTransport concept (port is mapped or ignored)
    bool start(uint16_t /*port*/ = 0) noexcept {
        return open("/dev/ttyACM0", 115200);
    }

    // Explicit hardware device & baud rate opening
    bool open(const std::string& device_path, uint32_t baud_rate = 115200) noexcept {
        try {
            boost::system::error_code ec;
            serial_.open(device_path, ec);
            if (ec) return false;

            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), ec);
            serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);

            work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(
                io_ctx_.get_executor());

            running_ = true;
            is_connected_ = true;

            start_read();

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
        if (serial_.is_open()) {
            serial_.cancel(ec);
            serial_.close(ec);
        }

        work_guard_.reset();
        io_ctx_.stop();

        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        is_connected_ = false;
    }

    void poll() noexcept {
        // Boost.Asio events processed asynchronously in io_thread_
    }

    bool send(std::span<const uint8_t> data) noexcept {
        if (!is_connected_ || !serial_.is_open()) return false;
        try {
            boost::asio::write(serial_, boost::asio::buffer(data.data(), data.size()));
            return true;
        } catch (...) {
            is_connected_ = false;
            return false;
        }
    }

    bool has_client() const noexcept {
        return is_connected_;
    }

private:
    boost::asio::io_context io_ctx_;
    std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;
    boost::asio::serial_port serial_;
    std::thread io_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> is_connected_{false};
    std::array<uint8_t, 512> rx_buf_{};
    OnBytesReceivedCb on_bytes_received_;

    void start_read() noexcept {
        if (!running_ || !serial_.is_open()) return;

        serial_.async_read_some(
            boost::asio::buffer(rx_buf_),
            [this](boost::system::error_code ec, std::size_t bytes_read) {
                if (!ec && bytes_read > 0) {
                    if (on_bytes_received_) {
                        on_bytes_received_(std::span<const uint8_t>(rx_buf_.data(), bytes_read));
                    }
                    start_read();
                } else {
                    is_connected_ = false;
                }
            });
    }
};

static_assert(MspTransport<BoostAsioSerialTransport>,
    "BoostAsioSerialTransport must satisfy MspTransport concept");

} // namespace abstractx::msp

#endif // __linux__ || __APPLE__

#endif // BOOST_ASIO_SERIAL_TRANSPORT_HPP
