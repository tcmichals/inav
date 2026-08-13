/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - BareCTF / Common Trace Format (CTF) Binary Logger
 */

#ifndef BLACKBOX_LOGGER_HPP
#define BLACKBOX_LOGGER_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <array>
#include <cstring>

namespace abstractx::logging {

// CTF (Common Trace Format) Stream Header Magic: 0xC1FC1FC1 ("CTF")
constexpr uint32_t CtfMagicNumber = 0xC1FC1FC1;

enum class LogLevel : uint8_t {
    Off       = 0,
    Error     = 1,
    Info      = 2,
    Debug     = 3,
    FlightData= 4
};

// CTF Packet Header (Compact 16-byte header aligned for CTF babeltrace analysis)
struct alignas(16) CtfPacketHeader {
    uint32_t magic{CtfMagicNumber}; // CTF Magic (0xC1FC1FC1)
    uint32_t stream_id{1};          // Flight Data Stream ID
    uint64_t timestamp_ns{0};       // 64-bit nanosecond hardware clock
};

// Compact CTF Flight Event Record (24 bytes payload)
struct alignas(8) CtfFlightEvent {
    int16_t accel[3]{0, 0, 0};
    int16_t gyro[3]{0, 0, 0};
    int16_t roll_deg_x10{0};
    int16_t pitch_deg_x10{0};
    uint16_t yaw_deg{0};
    uint16_t motor1{0};
    uint16_t motor2{0};
    uint16_t motor3{0};
    uint16_t motor4{0};
};

class BlackboxLogger {
public:
    static void set_level(LogLevel level) noexcept {
        current_level_ = level;
    }

    static LogLevel level() noexcept { return current_level_; }

    // Log a CTF (Common Trace Format) binary event packet into SPSC ring as a 64B TLP
    static bool log_ctf_event(SpscTlpRing<64>& log_ring, 
                              uint64_t timestamp_ns, 
                              const CtfFlightEvent& event) noexcept {
        if (current_level_ < LogLevel::FlightData) return false;

        Tlp64 tlp{};
        tlp.wire.type = static_cast<uint8_t>(TlpType::DmaStream);
        tlp.wire.channel = static_cast<uint8_t>(Channel::FlightLog); // 0x03
        tlp.wire.timestamp_ns = timestamp_ns;
        tlp.wire.length_dw = (sizeof(CtfPacketHeader) + sizeof(CtfFlightEvent)) / 4;

        // Pack CTF Header + CTF Event payload into 64B TLP
        CtfPacketHeader ctf_hdr{CtfMagicNumber, 1, timestamp_ns};
        std::memcpy(tlp.wire.payload, &ctf_hdr, sizeof(CtfPacketHeader));
        std::memcpy(tlp.wire.payload + sizeof(CtfPacketHeader), &event, sizeof(CtfFlightEvent));

        return log_ring.push(tlp);
    }

    // Log a CTF binary string event message into SPSC ring (Zero std::cout!)
    static bool log_info(SpscTlpRing<64>& log_ring, const char* msg, uint64_t timestamp_ns) noexcept {
        if (current_level_ < LogLevel::Info) return false;

        Tlp64 tlp{};
        tlp.wire.type = static_cast<uint8_t>(TlpType::Status);
        tlp.wire.channel = static_cast<uint8_t>(Channel::FlightLog);
        tlp.wire.timestamp_ns = timestamp_ns;

        size_t len = 0;
        while (msg[len] != '\0' && len < ASP_TLP64_PAYLOAD_SIZE - 1) {
            tlp.wire.payload[len] = static_cast<uint8_t>(msg[len]);
            len++;
        }
        tlp.wire.payload[len] = '\0';
        tlp.wire.length_dw = static_cast<uint16_t>((len + 4) / 4);

        return log_ring.push(tlp);
    }

private:
    static inline LogLevel current_level_{LogLevel::Info};
};

} // namespace abstractx::logging

#endif // BLACKBOX_LOGGER_HPP
