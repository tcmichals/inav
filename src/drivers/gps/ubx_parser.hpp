/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - U-Blox UBX Binary Protocol Parser & Config Frame Serializer
 *
 * Supports U-Blox 6 / 7 / 8 / 9 / M10 GNSS modules (UBX-NAV-PVT, UBX-NAV-DOP).
 * Zero dynamic memory allocations. Stateful FSM parser with Fletcher-16 CRC.
 */

#ifndef UBX_PARSER_HPP
#define UBX_PARSER_HPP

#include "gps_types.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <optional>
#include <cstring>

namespace abstractx::drivers::gps {

// ---------------------------------------------------------------------------
// Fletcher-16 Checksum Calculator (UBX standard)
// ---------------------------------------------------------------------------
constexpr std::pair<uint8_t, uint8_t> ubx_fletcher16(std::span<const uint8_t> data) noexcept {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (uint8_t byte : data) {
        ck_a += byte;
        ck_b += ck_a;
    }
    return {ck_a, ck_b};
}

// ---------------------------------------------------------------------------
// Encoded UBX Packet Container for Configuration Wire Frames
// ---------------------------------------------------------------------------
struct UbxWirePacket {
    std::array<uint8_t, 128> data{};
    size_t len{0};
};

// ---------------------------------------------------------------------------
// UBX Binary Protocol Parser & Serializer Class
// ---------------------------------------------------------------------------
class UbxParser {
public:
    static constexpr uint8_t SYNC1 = 0xB5;
    static constexpr uint8_t SYNC2 = 0x62;

    constexpr UbxParser() noexcept = default;

    void reset() noexcept {
        state_ = State::Sync1;
        ck_a_ = 0;
        ck_b_ = 0;
        payload_idx_ = 0;
        payload_len_ = 0;
        msg_class_ = 0;
        msg_id_ = 0;
    }

    // Process a single byte from stream. Returns true if a valid sample was decoded.
    bool parse_byte(uint8_t byte, uint64_t timestamp_ns = 0) noexcept {
        switch (state_) {
            case State::Sync1:
                if (byte == SYNC1) state_ = State::Sync2;
                break;

            case State::Sync2:
                if (byte == SYNC2) {
                    state_ = State::Class;
                    ck_a_ = 0;
                    ck_b_ = 0;
                } else {
                    state_ = State::Sync1;
                }
                break;

            case State::Class:
                msg_class_ = byte;
                update_crc(byte);
                state_ = State::Id;
                break;

            case State::Id:
                msg_id_ = byte;
                update_crc(byte);
                state_ = State::LenLo;
                break;

            case State::LenLo:
                payload_len_ = byte;
                update_crc(byte);
                state_ = State::LenHi;
                break;

            case State::LenHi:
                payload_len_ |= static_cast<uint16_t>(byte) << 8;
                update_crc(byte);
                payload_idx_ = 0;
                if (payload_len_ > 256) {
                    state_ = State::Sync1; // Overlong payload reject
                } else if (payload_len_ == 0) {
                    state_ = State::CkA;
                } else {
                    state_ = State::Payload;
                }
                break;

            case State::Payload:
                if (payload_idx_ < payload_buf_.size()) {
                    payload_buf_[payload_idx_] = byte;
                }
                update_crc(byte);
                payload_idx_++;
                if (payload_idx_ >= payload_len_) {
                    state_ = State::CkA;
                }
                break;

            case State::CkA:
                expected_ck_a_ = byte;
                state_ = State::CkB;
                break;

            case State::CkB:
                state_ = State::Sync1;
                if (expected_ck_a_ == ck_a_ && byte == ck_b_) {
                    return decode_packet(timestamp_ns);
                }
                break;
        }
        return false;
    }

    const GpsSample& latest_sample() const noexcept { return sample_; }

    // -----------------------------------------------------------------------
    // Configuration Packet Serializers (U-Blox 6/7/8/9 & M10)
    // -----------------------------------------------------------------------

    // UBX-CFG-PRT: Set UART baudrate (Class 0x06, ID 0x00, Len 20)
    static UbxWirePacket make_cfg_prt(uint32_t baudrate) noexcept {
        std::array<uint8_t, 20> payload{};
        payload[0] = 1; // PortID = UART1
        // Mode: 8N1 (0x08D0)
        payload[4] = 0xD0;
        payload[5] = 0x08;
        // Baud rate (uint32_t)
        payload[8]  = static_cast<uint8_t>(baudrate & 0xFF);
        payload[9]  = static_cast<uint8_t>((baudrate >> 8) & 0xFF);
        payload[10] = static_cast<uint8_t>((baudrate >> 16) & 0xFF);
        payload[11] = static_cast<uint8_t>((baudrate >> 24) & 0xFF);
        // inProtoMask: UBX | NMEA (0x0003)
        payload[12] = 0x03;
        // outProtoMask: UBX (0x0001)
        payload[14] = 0x01;

        return wrap_packet(0x06, 0x00, payload);
    }

    // UBX-CFG-RATE: Set measurement rate (Class 0x06, ID 0x08, Len 6)
    static UbxWirePacket make_cfg_rate(uint16_t rate_ms = 100) noexcept {
        std::array<uint8_t, 6> payload{};
        payload[0] = static_cast<uint8_t>(rate_ms & 0xFF); // 100ms = 10Hz
        payload[1] = static_cast<uint8_t>((rate_ms >> 8) & 0xFF);
        payload[2] = 1; // NavRate = 1 cycle
        payload[3] = 0;
        payload[4] = 1; // TimeRef = GPS time
        payload[5] = 0;

        return wrap_packet(0x06, 0x08, payload);
    }

    // UBX-CFG-NAV5: Set dynamic platform model (Class 0x06, ID 0x24, Len 36)
    static UbxWirePacket make_cfg_nav5(uint8_t model = 4) noexcept { // 4 = Airborne 4G
        std::array<uint8_t, 36> payload{};
        payload[0] = 0x01; // Mask: apply dynModel
        payload[2] = model; // 4 = Airborne 4G
        payload[3] = 3;    // FixMode: Auto 2D/3D
        return wrap_packet(0x06, 0x24, payload);
    }

private:
    enum class State : uint8_t {
        Sync1,
        Sync2,
        Class,
        Id,
        LenLo,
        LenHi,
        Payload,
        CkA,
        CkB
    };

    State state_{State::Sync1};
    uint8_t msg_class_{0};
    uint8_t msg_id_{0};
    uint16_t payload_len_{0};
    uint16_t payload_idx_{0};
    uint8_t ck_a_{0};
    uint8_t ck_b_{0};
    uint8_t expected_ck_a_{0};
    std::array<uint8_t, 256> payload_buf_{};
    GpsSample sample_{};

    void update_crc(uint8_t byte) noexcept {
        ck_a_ += byte;
        ck_b_ += ck_a_;
    }

    bool decode_packet(uint64_t timestamp_ns) noexcept {
        // NAV-PVT (Class 0x01, ID 0x07, min length 84 bytes)
        if (msg_class_ == 0x01 && msg_id_ == 0x07 && payload_len_ >= 84) {
            uint8_t fix_type_raw = payload_buf_[20];
            uint8_t flags = payload_buf_[21];
            bool gnss_fix_ok = (flags & 0x01) != 0;

            sample_.timestamp_ns = timestamp_ns;
            sample_.num_sats = payload_buf_[23];

            // Extract position (int32_t)
            int32_t lon_1e7 = 0;
            int32_t lat_1e7 = 0;
            int32_t h_msl_mm = 0;
            std::memcpy(&lon_1e7, &payload_buf_[24], 4);
            std::memcpy(&lat_1e7, &payload_buf_[28], 4);
            std::memcpy(&h_msl_mm, &payload_buf_[36], 4);

            sample_.longitude_1e7 = lon_1e7;
            sample_.latitude_1e7 = lat_1e7;
            sample_.altitude_cm = h_msl_mm / 10; // mm to cm

            // Extract velocities (int32_t mm/s -> int16_t cm/s)
            int32_t vel_n_mms = 0;
            int32_t vel_e_mms = 0;
            int32_t vel_d_mms = 0;
            int32_t head_1e5 = 0;
            std::memcpy(&vel_n_mms, &payload_buf_[48], 4);
            std::memcpy(&vel_e_mms, &payload_buf_[52], 4);
            std::memcpy(&vel_d_mms, &payload_buf_[56], 4);
            std::memcpy(&head_1e5, &payload_buf_[64], 4);

            sample_.vel_n_cms = static_cast<int16_t>(vel_n_mms / 10);
            sample_.vel_e_cms = static_cast<int16_t>(vel_e_mms / 10);
            sample_.vel_d_cms = static_cast<int16_t>(vel_d_mms / 10);

            // Heading (1e-5 deg -> 0.1 deg)
            if (head_1e5 < 0) head_1e5 += 36000000;
            sample_.ground_course_decideg = static_cast<uint16_t>((head_1e5 / 10000) % 3600);

            // Fix Type mapping
            if (gnss_fix_ok) {
                if (fix_type_raw == 3 || fix_type_raw == 4) {
                    sample_.fix_type = GpsFixType::Fix3D;
                } else if (fix_type_raw == 5) {
                    sample_.fix_type = GpsFixType::Fix3DDgps;
                } else if (fix_type_raw == 2) {
                    sample_.fix_type = GpsFixType::Fix2D;
                } else {
                    sample_.fix_type = GpsFixType::NoFix;
                }
            } else {
                sample_.fix_type = GpsFixType::NoFix;
            }

            // Extract pDOP (uint16_t, 0.01)
            uint16_t pdop = 9900;
            std::memcpy(&pdop, &payload_buf_[76], 2);
            sample_.hdop_centi = pdop;

            sample_.valid = (sample_.fix_type != GpsFixType::NoFix);
            return true;
        }

        // NAV-DOP (Class 0x01, ID 0x04, length 18 bytes)
        if (msg_class_ == 0x01 && msg_id_ == 0x04 && payload_len_ >= 18) {
            uint16_t hdop = 9900;
            std::memcpy(&hdop, &payload_buf_[12], 2);
            sample_.hdop_centi = hdop;
            return false;
        }

        return false;
    }

    template <size_t N>
    static UbxWirePacket wrap_packet(uint8_t cls, uint8_t id, const std::array<uint8_t, N>& payload) noexcept {
        UbxWirePacket wire{};
        wire.data[0] = SYNC1;
        wire.data[1] = SYNC2;
        wire.data[2] = cls;
        wire.data[3] = id;
        wire.data[4] = static_cast<uint8_t>(N & 0xFF);
        wire.data[5] = static_cast<uint8_t>((N >> 8) & 0xFF);

        std::memcpy(&wire.data[6], payload.data(), N);

        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < 6 + N; ++i) {
            ck_a += wire.data[i];
            ck_b += ck_a;
        }
        wire.data[6 + N] = ck_a;
        wire.data[7 + N] = ck_b;
        wire.len = 8 + N;
        return wire;
    }
};

} // namespace abstractx::drivers::gps

#endif // UBX_PARSER_HPP
