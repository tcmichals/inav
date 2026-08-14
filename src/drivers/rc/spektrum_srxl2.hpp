/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Spektrum SRXL2 Receiver Serial Protocol Parser
 *
 * Single-wire half-duplex UART at 115,200 baud.
 * Zero dynamic memory allocations. Stateful byte parser with CRC16-CCITT validation.
 */

#ifndef SPEKTRUM_SRXL2_HPP
#define SPEKTRUM_SRXL2_HPP

#include "rc_driver.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <optional>

namespace abstractx::drivers::rc {

// CRC16-CCITT lookup-free calculation (Polynomial 0x1021)
constexpr uint16_t srxl2_crc16(std::span<const uint8_t> data) noexcept {
    uint16_t crc = 0;
    for (uint8_t byte : data) {
        crc ^= (static_cast<uint16_t>(byte) << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

class SpektrumSrxl2 {
public:
    static constexpr uint8_t HEADER_SYNC = 0xA6;
    static constexpr uint8_t PKT_TYPE_CHANNEL = 0x10;
    static constexpr uint8_t PKT_TYPE_HANDSHAKE = 0xCD;

    constexpr SpektrumSrxl2() noexcept = default;

    void reset() noexcept {
        state_ = State::Sync;
        idx_ = 0;
        len_ = 0;
    }

    // Process incoming byte stream. Returns true if valid channels were updated.
    bool parse_byte(uint8_t byte) noexcept {
        switch (state_) {
            case State::Sync:
                if (byte == HEADER_SYNC) {
                    buf_[0] = byte;
                    idx_ = 1;
                    state_ = State::PacketType;
                }
                break;

            case State::PacketType:
                buf_[idx_++] = byte;
                pkt_type_ = byte;
                state_ = State::Length;
                break;

            case State::Length:
                buf_[idx_++] = byte;
                len_ = byte;
                if (len_ < 5 || len_ > buf_.size()) {
                    state_ = State::Sync; // Length out of bounds
                } else {
                    state_ = State::Payload;
                }
                break;

            case State::Payload:
                buf_[idx_++] = byte;
                if (idx_ >= len_) {
                    state_ = State::Sync;
                    return validate_and_decode();
                }
                break;
        }
        return false;
    }

    const RcChannels& channels() const noexcept { return rc_channels_; }

private:
    enum class State : uint8_t {
        Sync,
        PacketType,
        Length,
        Payload
    };

    State state_{State::Sync};
    uint8_t pkt_type_{0};
    uint8_t len_{0};
    uint8_t idx_{0};
    std::array<uint8_t, 64> buf_{};
    RcChannels rc_channels_{};

    bool validate_and_decode() noexcept {
        // Validate CRC16-CCITT over entire packet (last 2 bytes are CRC)
        uint16_t calc_crc = srxl2_crc16(std::span<const uint8_t>(buf_.data(), len_ - 2));
        uint16_t packet_crc = static_cast<uint16_t>((buf_[len_ - 2] << 8) | buf_[len_ - 1]);

        if (calc_crc != packet_crc) return false;

        // Channel Data Packet (0x10)
        if (pkt_type_ == PKT_TYPE_CHANNEL && len_ >= 14) {
            // Unpack 11-bit channels starting at payload offset 5
            size_t payload_offset = 5;
            size_t num_channels = (len_ - 7) / 2;
            if (num_channels > 16) num_channels = 16;

            for (size_t i = 0; i < num_channels; ++i) {
                uint16_t raw_val = static_cast<uint16_t>(
                    (buf_[payload_offset + i * 2] << 8) | buf_[payload_offset + i * 2 + 1]);
                uint16_t channel_bits = raw_val & 0x07FF; // 11-bit value (0..2047)

                // Scale Spektrum 11-bit (0..2047) to standard PWM (1000..2000 us)
                uint16_t pwm_us = static_cast<uint16_t>(1000 + (channel_bits * 1000 / 2048));
                rc_channels_.channels[i] = pwm_us;
            }

            rc_channels_.connected = true;
            rc_channels_.failsafe = false;
            return true;
        }

        return false;
    }
};

} // namespace abstractx::drivers::rc

#endif // SPEKTRUM_SRXL2_HPP
