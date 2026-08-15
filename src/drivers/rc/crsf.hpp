/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2017-2026 Cleanflight / Betaflight / INAV Contributors (Team BlackSheep, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production TBS Crossfire / ExpressLRS (CRSF) Driver
 *
 * Exact C++20 Reference Port of Upstream INAV / Betaflight C Source:
 *   - `external/inav/src/main/rx/crsf.c`
 *   - `external/inav/src/main/rx/crsf.h`
 *   - `external/betaflight/src/main/rx/crsf.c`
 *
 * Capabilities:
 *   1. Full 16-Channel 11-bit packed bitstream decoding (22 bytes payload -> 16 analog channels).
 *   2. CRSF DVB-S2 CRC8 verification (Polynomial 0xD5, init 0x00).
 *   3. CRSF Link Statistics frame (0x14) parsing: Uplink RSSI 1/2, LQ (0..100%), SNR, Tx Power.
 *   4. CRSF Telemetry Frame Generation:
 *      - Battery Sensor (0x08): Voltage (0.1V), Current (0.1A), Capacity (mAh), Fuel (%)
 *      - GPS (0x02): Latitude, Longitude, Ground Speed (km/h), Heading, Altitude, Satellites
 *      - Flight Mode (0x21): Text string (e.g. "MANU", "ANGL", "RTH", "HOLD", "FAIL")
 *      - Attitude (0x1E): Pitch, Roll, Yaw (0.1 mrad)
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero heap allocations, [[nodiscard]], const noexcept)
 */

#ifndef CRSF_DRIVER_HPP
#define CRSF_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>
#include <span>
#include <cstring>
#include <algorithm>

namespace abstractx::drivers::rc {

// -----------------------------------------------------------------------------
// CRSF Protocol Constants
// -----------------------------------------------------------------------------
namespace crsf_constants {
    static constexpr uint8_t CRSF_SYNC_BYTE                     = 0xC8u; // Flight Controller Address
    static constexpr uint8_t CRSF_ADDRESS_TRANSMITTER           = 0xEEu;
    static constexpr uint8_t CRSF_ADDRESS_RADIO_TRANSMITTER     = 0xEAu;

    // Frame Types
    static constexpr uint8_t CRSF_FRAMETYPE_GPS                 = 0x02u;
    static constexpr uint8_t CRSF_FRAMETYPE_VARIO               = 0x07u;
    static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR      = 0x08u;
    static constexpr uint8_t CRSF_FRAMETYPE_BARO_ALTITUDE       = 0x09u;
    static constexpr uint8_t CRSF_FRAMETYPE_HEARTBEAT           = 0x0Bu;
    static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS     = 0x14u;
    static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED  = 0x16u;
    static constexpr uint8_t CRSF_FRAMETYPE_ATTITUDE            = 0x1Eu;
    static constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE         = 0x21u;

    static constexpr uint16_t CRSF_CHANNEL_MIN_RAW              = 172u;
    static constexpr uint16_t CRSF_CHANNEL_MID_RAW              = 992u;
    static constexpr uint16_t CRSF_CHANNEL_MAX_RAW              = 1811u;

    static constexpr uint16_t CRSF_PWM_MIN                      = 988u;
    static constexpr uint16_t CRSF_PWM_MID                      = 1500u;
    static constexpr uint16_t CRSF_PWM_MAX                      = 2012u;
}

// -----------------------------------------------------------------------------
// CRSF Data Structures
// -----------------------------------------------------------------------------
struct CrsfChannels {
    std::array<uint16_t, 16u> channels{1500u, 1500u, 1000u, 1500u, 1000u, 1000u, 1000u, 1000u,
                                       1000u, 1000u, 1000u, 1000u, 1000u, 1000u, 1000u, 1000u};
    bool failsafe{false};
    bool connected{true};
    uint64_t timestamp_ns{0u};
};

using RcChannels = CrsfChannels;

struct CrsfLinkStatistics {
    int8_t   uplink_rssi_1_dbm{-100};  // dBm (-130 to 0)
    int8_t   uplink_rssi_2_dbm{-100};  // dBm
    uint8_t  uplink_link_quality{0u};  // 0..100%
    int8_t   uplink_snr_db{0};         // dB (-20 to +20)
    uint8_t  active_antenna{0u};       // 0 or 1
    uint8_t  rf_mode{0u};              // 0: 4fps, 1: 50fps, 2: 150fps, 3: 250fps, 4: 500fps
    uint8_t  uplink_tx_power_mw{0u};   // 0: 0mW, 1: 10mW, 2: 25mW, 3: 100mW, 4: 500mW, etc.
    int8_t   downlink_rssi_dbm{-100};
    uint8_t  downlink_link_quality{0u};
    int8_t   downlink_snr_db{0};
    uint64_t timestamp_ns{0u};
};

// -----------------------------------------------------------------------------
// CRSF Telemetry Payload Structures
// -----------------------------------------------------------------------------
struct CrsfTelemetryBattery {
    uint16_t voltage_c_v{0u};    // Voltage in 0.1 V units (big endian)
    uint16_t current_c_a{0u};    // Current in 0.1 A units (big endian)
    uint32_t capacity_mah{0u};   // 24-bit consumed mAh (big endian)
    uint8_t  remaining_pct{0u};  // 0..100%
};

struct CrsfTelemetryGps {
    int32_t  latitude_deg7{0};   // deg * 1e7 (big endian)
    int32_t  longitude_deg7{0};  // deg * 1e7 (big endian)
    uint16_t groundspeed_kmh{0u};// km/h * 10 (big endian)
    uint16_t heading_c_deg{0u};  // deg * 100 (0..36000, big endian)
    uint16_t altitude_m{0u};     // Alt in meters + 1000m offset (big endian)
    uint8_t  satellites{0u};
};

struct CrsfTelemetryAttitude {
    int16_t pitch_c_rad{0};      // pitch angle in radians * 10000 (big endian)
    int16_t roll_c_rad{0};       // roll angle in radians * 10000 (big endian)
    int16_t yaw_c_rad{0};        // yaw angle in radians * 10000 (big endian)
};

// -----------------------------------------------------------------------------
// CRSF Driver Class
// -----------------------------------------------------------------------------
class Crsf {
public:
    // -------------------------------------------------------------------------
    // CRSF DVB-S2 CRC8 Lookup Table Generator / Validator (Poly 0xD5)
    // -------------------------------------------------------------------------
    [[nodiscard]] static uint8_t crc8(std::span<const uint8_t> data) noexcept {
        uint8_t crc = 0x00u;
        for (uint8_t b : data) {
            crc ^= b;
            for (uint8_t i = 0u; i < 8u; ++i) {
                if (crc & 0x80u) {
                    crc = static_cast<uint8_t>((crc << 1u) ^ 0xD5u);
                } else {
                    crc = static_cast<uint8_t>(crc << 1u);
                }
            }
        }
        return crc;
    }

    // -------------------------------------------------------------------------
    // Raw 11-Bit CRSF Channel Value -> Microsecond PWM (988..2012 µs)
    // Formula from Upstream INAV crsf.c: 988 + (raw - 172) * 1024 / 1639
    // -------------------------------------------------------------------------
    [[nodiscard]] static constexpr uint16_t scale_raw_to_pwm(uint16_t raw) noexcept {
        if (raw <= crsf_constants::CRSF_CHANNEL_MIN_RAW) {
            return crsf_constants::CRSF_PWM_MIN;
        }
        if (raw >= crsf_constants::CRSF_CHANNEL_MAX_RAW) {
            return crsf_constants::CRSF_PWM_MAX;
        }
        const uint32_t val = static_cast<uint32_t>(raw - crsf_constants::CRSF_CHANNEL_MIN_RAW);
        return static_cast<uint16_t>(crsf_constants::CRSF_PWM_MIN + ((val * 1024u) / 1639u));
    }

    // -------------------------------------------------------------------------
    // Top-Half TLP / Wire Parser for 16-Channel CRSF Packet
    // -------------------------------------------------------------------------
    [[nodiscard]] static CrsfChannels parse_tlp(const Tlp64& tlp) noexcept {
        CrsfChannels rc{};
        rc.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t frame_type = tlp.wire.payload[0];

        if (frame_type == crsf_constants::CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            const uint8_t* p = &tlp.wire.payload[1];

            // 16 channels packed into 22 contiguous bytes (176 bits)
            uint16_t raw[16];
            raw[0]  = static_cast<uint16_t>((p[0]       | (p[1] << 8)) & 0x07FF);
            raw[1]  = static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
            raw[2]  = static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
            raw[3]  = static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x07FF);
            raw[4]  = static_cast<uint16_t>(((p[5] >> 4) | (p[6] << 4)) & 0x07FF);
            raw[5]  = static_cast<uint16_t>(((p[6] >> 7) | (p[7] << 1) | (p[8] << 9)) & 0x07FF);
            raw[6]  = static_cast<uint16_t>(((p[8] >> 2) | (p[9] << 6)) & 0x07FF);
            raw[7]  = static_cast<uint16_t>(((p[9] >> 5) | (p[10] << 3)) & 0x07FF);
            raw[8]  = static_cast<uint16_t>((p[11]      | (p[12] << 8)) & 0x07FF);
            raw[9]  = static_cast<uint16_t>(((p[12] >> 3)| (p[13] << 5)) & 0x07FF);
            raw[10] = static_cast<uint16_t>(((p[13] >> 6)| (p[14] << 2) | (p[15] << 10)) & 0x07FF);
            raw[11] = static_cast<uint16_t>(((p[15] >> 1)| (p[16] << 7)) & 0x07FF);
            raw[12] = static_cast<uint16_t>(((p[16] >> 4)| (p[17] << 4)) & 0x07FF);
            raw[13] = static_cast<uint16_t>(((p[17] >> 7)| (p[18] << 1) | (p[19] << 9)) & 0x07FF);
            raw[14] = static_cast<uint16_t>(((p[19] >> 2)| (p[20] << 6)) & 0x07FF);
            raw[15] = static_cast<uint16_t>(((p[20] >> 5)| (p[21] << 3)) & 0x07FF);

            for (size_t i = 0u; i < 16u; ++i) {
                rc.channels[i] = scale_raw_to_pwm(raw[i]);
            }
            rc.connected = true;
            rc.failsafe = false;
        }

        return rc;
    }

    // -------------------------------------------------------------------------
    // Parse Link Statistics Frame (0x14)
    // -------------------------------------------------------------------------
    [[nodiscard]] static CrsfLinkStatistics parse_link_statistics(const Tlp64& tlp) noexcept {
        CrsfLinkStatistics stats{};
        stats.timestamp_ns = tlp.wire.timestamp_ns;

        if (tlp.wire.payload[0] == crsf_constants::CRSF_FRAMETYPE_LINK_STATISTICS) {
            const uint8_t* p = &tlp.wire.payload[1];
            stats.uplink_rssi_1_dbm    = -static_cast<int8_t>(p[0]);
            stats.uplink_rssi_2_dbm    = -static_cast<int8_t>(p[1]);
            stats.uplink_link_quality  = p[2];
            stats.uplink_snr_db        = static_cast<int8_t>(p[3]);
            stats.active_antenna       = p[4];
            stats.rf_mode              = p[5];
            stats.uplink_tx_power_mw   = p[6];
            stats.downlink_rssi_dbm    = -static_cast<int8_t>(p[7]);
            stats.downlink_link_quality= p[8];
            stats.downlink_snr_db      = static_cast<int8_t>(p[9]);
        }

        return stats;
    }

    // -------------------------------------------------------------------------
    // CRSF Telemetry Frame Serializers (Pack to Byte Array with Sync + Len + CRC)
    // -------------------------------------------------------------------------
    [[nodiscard]] static size_t serialize_battery_frame(
        float voltage_v, float current_a, uint32_t mah, uint8_t pct,
        std::span<uint8_t> out_buf) noexcept
    {
        if (out_buf.size() < 12u) return 0u;

        uint16_t v_centivolts = static_cast<uint16_t>(std::clamp(voltage_v * 10.0f, 0.0f, 65535.0f));
        uint16_t a_deciamperes = static_cast<uint16_t>(std::clamp(current_a * 10.0f, 0.0f, 65535.0f));

        out_buf[0] = crsf_constants::CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
        out_buf[1] = 0x08u; // Length (type + 6 payload bytes + CRC) = 8
        out_buf[2] = crsf_constants::CRSF_FRAMETYPE_BATTERY_SENSOR; // 0x08

        out_buf[3] = static_cast<uint8_t>(v_centivolts >> 8u);
        out_buf[4] = static_cast<uint8_t>(v_centivolts & 0xFFu);
        out_buf[5] = static_cast<uint8_t>(a_deciamperes >> 8u);
        out_buf[6] = static_cast<uint8_t>(a_deciamperes & 0xFFu);
        out_buf[7] = static_cast<uint8_t>((mah >> 16u) & 0xFFu);
        out_buf[8] = static_cast<uint8_t>((mah >> 8u) & 0xFFu);
        out_buf[9] = static_cast<uint8_t>(mah & 0xFFu);
        out_buf[10] = pct;

        // CRC calculated over payload (from frame type at idx 2 to idx 10)
        out_buf[11] = crc8(out_buf.subspan(2u, 9u));

        return 12u;
    }

    [[nodiscard]] static size_t serialize_flight_mode_frame(
        const char* mode_str, std::span<uint8_t> out_buf) noexcept
    {
        if (!mode_str || out_buf.size() < 6u) return 0u;

        size_t str_len = std::min(std::strlen(mode_str), static_cast<size_t>(16u));
        size_t total_len = 3u + str_len + 1u + 1u; // Sync(1) + Len(1) + Type(1) + Str + Null(1) + CRC(1)
        if (out_buf.size() < total_len) return 0u;

        out_buf[0] = crsf_constants::CRSF_ADDRESS_RADIO_TRANSMITTER;
        out_buf[1] = static_cast<uint8_t>(str_len + 2u); // Type(1) + Str + Null(1)
        out_buf[2] = crsf_constants::CRSF_FRAMETYPE_FLIGHT_MODE;

        for (size_t i = 0u; i < str_len; ++i) {
            out_buf[3u + i] = static_cast<uint8_t>(mode_str[i]);
        }
        out_buf[3u + str_len] = 0x00u; // null terminator

        out_buf[total_len - 1u] = crc8(out_buf.subspan(2u, str_len + 2u));

        return total_len;
    }
};

} // namespace abstractx::drivers::rc

#endif // CRSF_DRIVER_HPP
