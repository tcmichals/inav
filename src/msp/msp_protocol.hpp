/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 * Copyright (C) 2010-2014 MultiWii Contributors (Alexinparis, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MultiWii Serial Protocol (MSP v1/v2) Frame Processor
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/io/msp.c, src/main/msp/msp_protocol.h
 *   - Upstream Cleanflight: src/main/io/msp.c
 *
 * Portable — zero I/O, zero platform dependencies.
 * Command handlers accept live state references; no hardcoded values.
 */

#ifndef MSP_PROTOCOL_HPP
#define MSP_PROTOCOL_HPP


#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace abstractx::msp {

// ---------------------------------------------------------------------------
// MSP Command IDs — Complete set required by iNav Configurator handshake
// ---------------------------------------------------------------------------
enum class Cmd : uint16_t {
    // Identity & Handshake (Configurator connects with these first)
    ApiVersion   = 1,
    FcVariant    = 2,
    FcVersion    = 3,
    BoardInfo    = 4,
    BuildInfo    = 5,

    // Status & Sensor Data
    Status       = 101,
    RawImu       = 102,
    Servo        = 103,
    Motor        = 104,
    Rc           = 105,
    Attitude     = 108,
    Altitude     = 109,
    Analog       = 110,
    RcTuning     = 111,

    // Configuration
    Pid          = 112,
    Misc         = 114,
    BoxNames     = 116,
    BoxIds       = 119,
    StatusEx     = 150,
    Uid          = 160,

    // Write Commands
    SetRawRc       = 200,
    SetRawGps      = 201,
    SetPid         = 202,
    AccCalibration = 205, // Trigger Accelerometer Zero Calibration
    MagCalibration = 206, // Trigger Magnetometer 3D Calibration
    SetMisc        = 207,
    SetMotor       = 214, // Set isolated bench motor PWM test outputs
    Set4WayIf      = 245,
    EepromWrite    = 250, // Save config to flash memory

    // MSP2 INAV Commands
    EzTuneGet      = 0x2405, // MSP2_COMMON_GET_EZ_TUNE
    EzTuneSet      = 0x2406, // MSP2_COMMON_SET_EZ_TUNE

    // System
    Reboot       = 68,

    // Navigation
    RawGps       = 106,
    CompGps      = 107,
    NavStatus    = 121,
    NavConfig    = 122,
    WpGetInfo    = 20,
};

// ---------------------------------------------------------------------------
// Live Flight State — passed by reference to MspEngine so responses
// contain real data, not hardcoded zeros. Populated by the flight loop.
// ---------------------------------------------------------------------------
struct MspLiveState {
    // IMU (raw sensor units)
    int16_t accel_x{0}, accel_y{0}, accel_z{512}; // 512 = 1G
    int16_t gyro_x{0}, gyro_y{0}, gyro_z{0};
    int16_t mag_x{0}, mag_y{0}, mag_z{0};

    // Attitude (0.1 deg units for roll/pitch, 1 deg for yaw)
    int16_t roll_decideg{0};
    int16_t pitch_decideg{0};
    int16_t yaw_deg{0};

    // Altitude (cm)
    int32_t altitude_cm{0};
    int16_t vario_cms{0};

    // Motors (microseconds)
    std::array<uint16_t, 8> motor_us{1000, 1000, 1000, 1000, 0, 0, 0, 0};

    // Status
    uint32_t cycle_time_us{1000};
    uint16_t i2c_error_count{0};
    uint16_t sensor_flags{0x31}; // ACC | BARO | GPS
    uint32_t arming_flags{0};
    uint8_t  profile_index{0};

    // Power
    uint8_t  vbat_cells{0};
    uint16_t vbat_mv{0};
    uint16_t current_ma{0};
    uint16_t mah_drawn{0};
    uint16_t rssi{0};

    // RC Channels (16 microsecond PWM channels)
    std::array<uint16_t, 16> rc_channels{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
                                         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
};

// ---------------------------------------------------------------------------
// MSP Response Frame — populated by process_command()
// ---------------------------------------------------------------------------
struct MspFrame {
    uint8_t  flag{0};
    uint16_t command{0};
    uint16_t payload_len{0};
    std::array<uint8_t, 256> payload{};

    constexpr void reset() noexcept {
        flag = 0;
        command = 0;
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

    constexpr bool push_string(const char* str) noexcept {
        while (*str && payload_len < 256) {
            payload[payload_len++] = static_cast<uint8_t>(*str++);
        }
        return true;
    }

    constexpr std::span<const uint8_t> payload_span() const noexcept {
        return std::span<const uint8_t>(payload.data(), payload_len);
    }
};

// ---------------------------------------------------------------------------
// MSP Command Processor — Portable, takes live state by const reference
// ---------------------------------------------------------------------------
class MspEngine {
public:
    static bool process_command(Cmd cmd,
                                const std::span<const uint8_t>& rx_payload,
                                MspFrame& tx_frame,
                                const MspLiveState& live_state) noexcept;
};

} // namespace abstractx::msp

#endif // MSP_PROTOCOL_HPP
