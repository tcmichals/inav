/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MultiWii Serial Protocol (MSP v1/v2) C++20 Header
 */

#ifndef MSP_PROTOCOL_HPP
#define MSP_PROTOCOL_HPP

#include <cstdint>
#include <cstddef>
#include <span>
#include <array>

namespace abstractx::msp {

// MSP Framing Constants
constexpr uint8_t MspHeaderByte1 = '$';
constexpr uint8_t MspHeaderByte2 = 'M';
constexpr uint8_t MspHeaderDirectionToFc = '<';
constexpr uint8_t MspHeaderDirectionFromFc = '>';
constexpr uint8_t MspHeaderDirectionError = '!';

// Core MSP Message Identifiers (Compatible with iNav & Betaflight Configurator)
enum class Cmd : uint16_t {
    ApiVersion   = 1,   // MSP_API_VERSION
    FcVariant    = 2,   // MSP_FC_VARIANT ("INAV")
    FcVersion    = 3,   // MSP_FC_VERSION
    BoardInfo    = 4,   // MSP_BOARD_INFO
    BuildInfo    = 5,   // MSP_BUILD_INFO
    
    RawImu       = 102, // MSP_RAW_IMU (Accel, Gyro, Mag)
    Attitude     = 108, // MSP_ATTITUDE (Roll, Pitch, Yaw in 0.1 deg)
    Altitude     = 109, // MSP_ALTITUDE (Est altitude in cm)
    Analog       = 110, // MSP_ANALOG (VBat, Amperage)
    
    Motor        = 104, // MSP_MOTOR (Read motor outputs)
    SetMotor     = 214, // MSP_SET_MOTOR (Drive motors from Configurator)
    
    Pid          = 112, // MSP_PID (Read PID settings)
    SetPid       = 202, // MSP_SET_PID (Save tuned PIDs from Configurator)
    
    NavStatus    = 121, // MSP_NAV_STATUS
    PositionHold = 122  // MSP_NAV_POSHOLD
};

// MSP Frame Buffer Container
struct MspFrame {
    Cmd command{Cmd::ApiVersion};
    uint8_t payload_len{0};
    std::array<uint8_t, 256> payload{};

    constexpr void reset() noexcept {
        payload_len = 0;
    }

    constexpr bool push_u8(uint8_t val) noexcept {
        if (payload_len >= 256) return false;
        payload[payload_len++] = val;
        return true;
    }

    constexpr bool push_u16(uint16_t val) noexcept {
        if (payload_len + 2 > 256) return false;
        payload[payload_len++] = static_cast<uint8_t>(val & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 8) & 0xFF);
        return true;
    }

    constexpr bool push_u32(uint32_t val) noexcept {
        if (payload_len + 4 > 256) return false;
        payload[payload_len++] = static_cast<uint8_t>(val & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 8) & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 16) & 0xFF);
        payload[payload_len++] = static_cast<uint8_t>((val >> 24) & 0xFF);
        return true;
    }
};

// Zero-allocation MSP Processor Engine
class MspEngine {
public:
    static bool process_command(Cmd cmd, const std::span<const uint8_t>& rx_payload, MspFrame& tx_frame) noexcept;
};

} // namespace abstractx::msp

#endif // MSP_PROTOCOL_HPP
