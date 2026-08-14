/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Portable MSP Transport Concept, V2 Frame Parser & Serializer
 *
 * This file contains ZERO platform dependencies. It defines:
 *   1. MspTransport C++20 concept — the common hooks for Linux/Boost.Asio and Pico2W/lwIP
 *   2. MspV2FrameParser — stateful byte-by-byte parser for MSP v1 ($M<) and v2 ($X<) frames
 *   3. MspV2FrameSerializer — serializes MspFrame to wire bytes with checksum
 *   4. CRC8-DVB-S2 — standard MSP v2 checksum
 */

#ifndef MSP_TRANSPORT_HPP
#define MSP_TRANSPORT_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <optional>
#include <concepts>

namespace abstractx::msp {

// ---------------------------------------------------------------------------
// C++20 Concept: MspTransport
// Both BoostAsioTransport (Linux) and LwipMspTransport (Pico 2 W) satisfy this.
// The MSP server is templated on this concept — zero #ifdefs in protocol layer.
// ---------------------------------------------------------------------------
template <typename T>
concept MspTransport = requires(T transport, std::span<const uint8_t> data, uint16_t port) {
    { transport.start(port) } -> std::same_as<bool>;
    { transport.stop() } -> std::same_as<void>;
    { transport.poll() } -> std::same_as<void>;
    { transport.send(data) } -> std::same_as<bool>;
    { transport.has_client() } -> std::same_as<bool>;
};

// ---------------------------------------------------------------------------
// CRC8-DVB-S2 — Standard MSP v2 Checksum (Portable, no lookup table needed)
// ---------------------------------------------------------------------------
constexpr uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte) noexcept {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
        } else {
            crc = static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

constexpr uint8_t crc8_dvb_s2_buf(std::span<const uint8_t> data) noexcept {
    uint8_t crc = 0;
    for (auto b : data) {
        crc = crc8_dvb_s2(crc, b);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// MSP v1/v2 Wire Format Constants
// ---------------------------------------------------------------------------
static constexpr uint8_t MSP_V1_HEADER_M   = 'M';
static constexpr uint8_t MSP_V2_HEADER_X   = 'X';
static constexpr uint8_t MSP_DIR_REQUEST    = '<';
static constexpr uint8_t MSP_DIR_RESPONSE   = '>';
static constexpr uint8_t MSP_DIR_ERROR      = '!';

// Maximum serialized frame size: header(3) + flag(1) + cmd(2) + len(2) + payload(256) + crc(1) = 265
static constexpr size_t MSP_MAX_WIRE_SIZE = 265;

// ---------------------------------------------------------------------------
// Parsed MSP Frame (output of parser, input to command processor)
// ---------------------------------------------------------------------------
struct MspParsedFrame {
    uint16_t command{0};
    uint8_t  flag{0};
    uint16_t payload_len{0};
    std::array<uint8_t, 256> payload{};
    bool     is_v2{false};

    constexpr std::span<const uint8_t> payload_span() const noexcept {
        return std::span<const uint8_t>(payload.data(), payload_len);
    }
};

// ---------------------------------------------------------------------------
// MSP v2 Frame Parser — Stateful byte-by-byte parser
//
// Feed raw TCP/lwIP bytes via feed(). Poll complete frames via next_frame().
// Handles both MSP v1 ($M<) and MSP v2 ($X<) frames.
// ---------------------------------------------------------------------------
class MspV2FrameParser {
public:
    constexpr MspV2FrameParser() noexcept = default;

    // Feed raw bytes from transport (TCP recv / lwIP pbuf)
    void feed(std::span<const uint8_t> data) noexcept {
        for (auto byte : data) {
            parse_byte(byte);
        }
    }

    // Pop next complete parsed frame, or nullopt if none ready
    std::optional<MspParsedFrame> next_frame() noexcept {
        if (!frame_ready_) return std::nullopt;
        frame_ready_ = false;
        return current_frame_;
    }

    void reset() noexcept {
        state_ = State::Idle;
        frame_ready_ = false;
        crc_ = 0;
        payload_idx_ = 0;
    }

private:
    enum class State : uint8_t {
        Idle,
        GotDollar,      // received '$'
        GotHeader,      // received 'M' or 'X'
        // MSP v1 states
        V1PayloadLen,
        V1Command,
        V1Payload,
        V1Checksum,
        // MSP v2 states
        V2Flag,
        V2CmdLo,
        V2CmdHi,
        V2LenLo,
        V2LenHi,
        V2Payload,
        V2Checksum
    };

    State state_{State::Idle};
    MspParsedFrame current_frame_{};
    bool frame_ready_{false};
    bool is_v2_{false};
    uint8_t crc_{0};
    uint8_t v1_checksum_{0};
    uint16_t payload_idx_{0};

    void parse_byte(uint8_t byte) noexcept {
        switch (state_) {
            case State::Idle:
                if (byte == '$') state_ = State::GotDollar;
                break;

            case State::GotDollar:
                if (byte == MSP_V1_HEADER_M) {
                    is_v2_ = false;
                    state_ = State::GotHeader;
                } else if (byte == MSP_V2_HEADER_X) {
                    is_v2_ = true;
                    state_ = State::GotHeader;
                } else {
                    state_ = State::Idle;
                }
                break;

            case State::GotHeader:
                if (byte == MSP_DIR_REQUEST || byte == MSP_DIR_RESPONSE || byte == MSP_DIR_ERROR) {
                    current_frame_ = MspParsedFrame{};
                    current_frame_.is_v2 = is_v2_;
                    if (is_v2_) {
                        crc_ = 0;
                        state_ = State::V2Flag;
                    } else {
                        v1_checksum_ = 0;
                        state_ = State::V1PayloadLen;
                    }
                } else {
                    state_ = State::Idle;
                }
                break;

            // ----- MSP v1 ($M<) -----
            case State::V1PayloadLen:
                current_frame_.payload_len = byte;
                v1_checksum_ ^= byte;
                state_ = State::V1Command;
                break;

            case State::V1Command:
                current_frame_.command = byte;
                v1_checksum_ ^= byte;
                payload_idx_ = 0;
                if (current_frame_.payload_len > 0) {
                    state_ = State::V1Payload;
                } else {
                    state_ = State::V1Checksum;
                }
                break;

            case State::V1Payload:
                if (payload_idx_ < 256) {
                    current_frame_.payload[payload_idx_] = byte;
                }
                v1_checksum_ ^= byte;
                payload_idx_++;
                if (payload_idx_ >= current_frame_.payload_len) {
                    state_ = State::V1Checksum;
                }
                break;

            case State::V1Checksum:
                if (byte == v1_checksum_) {
                    frame_ready_ = true;
                }
                state_ = State::Idle;
                break;

            // ----- MSP v2 ($X<) -----
            case State::V2Flag:
                current_frame_.flag = byte;
                crc_ = crc8_dvb_s2(crc_, byte);
                state_ = State::V2CmdLo;
                break;

            case State::V2CmdLo:
                current_frame_.command = byte;
                crc_ = crc8_dvb_s2(crc_, byte);
                state_ = State::V2CmdHi;
                break;

            case State::V2CmdHi:
                current_frame_.command |= static_cast<uint16_t>(byte) << 8;
                crc_ = crc8_dvb_s2(crc_, byte);
                state_ = State::V2LenLo;
                break;

            case State::V2LenLo:
                current_frame_.payload_len = byte;
                crc_ = crc8_dvb_s2(crc_, byte);
                state_ = State::V2LenHi;
                break;

            case State::V2LenHi:
                current_frame_.payload_len |= static_cast<uint16_t>(byte) << 8;
                crc_ = crc8_dvb_s2(crc_, byte);
                payload_idx_ = 0;
                if (current_frame_.payload_len > 0 && current_frame_.payload_len <= 256) {
                    state_ = State::V2Payload;
                } else if (current_frame_.payload_len == 0) {
                    state_ = State::V2Checksum;
                } else {
                    state_ = State::Idle; // Payload too large, reject
                }
                break;

            case State::V2Payload:
                if (payload_idx_ < 256) {
                    current_frame_.payload[payload_idx_] = byte;
                }
                crc_ = crc8_dvb_s2(crc_, byte);
                payload_idx_++;
                if (payload_idx_ >= current_frame_.payload_len) {
                    state_ = State::V2Checksum;
                }
                break;

            case State::V2Checksum:
                if (byte == crc_) {
                    frame_ready_ = true;
                }
                state_ = State::Idle;
                break;
        }
    }
};

// ---------------------------------------------------------------------------
// MSP v1 Frame Serializer — Serialize response frame to $M> wire bytes
// ---------------------------------------------------------------------------
struct MspV1WireFrame {
    std::array<uint8_t, MSP_MAX_WIRE_SIZE> data{};
    size_t len{0};
};

struct MspV2WireFrame {
    std::array<uint8_t, MSP_MAX_WIRE_SIZE> data{};
    size_t len{0};
};

class MspFrameSerializer {
public:
    // Serialize MSP v1 response: $M> <len> <cmd> <payload...> <xor_checksum>
    template <typename Frame>
    static MspV1WireFrame serialize_v1(const Frame& frame) noexcept {
        MspV1WireFrame wire{};
        wire.data[0] = '$';
        wire.data[1] = 'M';
        wire.data[2] = '>';
        wire.data[3] = static_cast<uint8_t>(frame.payload_len);
        wire.data[4] = static_cast<uint8_t>(frame.command);

        uint8_t checksum = wire.data[3] ^ wire.data[4];
        for (uint16_t i = 0; i < frame.payload_len && i < 256; ++i) {
            wire.data[5 + i] = frame.payload[i];
            checksum ^= frame.payload[i];
        }
        wire.data[5 + frame.payload_len] = checksum;
        wire.len = 6 + frame.payload_len;
        return wire;
    }

    // Serialize MSP v2 response: $X> <flag> <cmd16> <len16> <payload...> <crc8>
    template <typename Frame>
    static MspV2WireFrame serialize_v2(const Frame& frame) noexcept {
        MspV2WireFrame wire{};
        wire.data[0] = '$';
        wire.data[1] = 'X';
        wire.data[2] = '>';

        // Fields after header are CRC'd
        wire.data[3] = frame.flag;                                          // flag
        wire.data[4] = static_cast<uint8_t>(frame.command & 0xFF);          // cmd lo
        wire.data[5] = static_cast<uint8_t>((frame.command >> 8) & 0xFF);   // cmd hi
        wire.data[6] = static_cast<uint8_t>(frame.payload_len & 0xFF);      // len lo
        wire.data[7] = static_cast<uint8_t>((frame.payload_len >> 8) & 0xFF); // len hi

        for (uint16_t i = 0; i < frame.payload_len && i < 256; ++i) {
            wire.data[8 + i] = frame.payload[i];
        }

        // CRC8-DVB-S2 over flag + cmd + len + payload (bytes 3..end)
        uint8_t crc = 0;
        for (size_t i = 3; i < 8 + static_cast<size_t>(frame.payload_len); ++i) {
            crc = crc8_dvb_s2(crc, wire.data[i]);
        }
        wire.data[8 + frame.payload_len] = crc;
        wire.len = 9 + frame.payload_len;
        return wire;
    }
};

} // namespace abstractx::msp

#endif // MSP_TRANSPORT_HPP
