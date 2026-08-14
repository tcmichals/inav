/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Comprehensive ESC Driver (DShot, OneShot, FastPWM, Standard PWM)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/drivers/pwm_output_dshot.c
 *   - Upstream INAV: src/main/drivers/pwm_output_dshot.c
 */

#ifndef ESC_DSHOT_DRIVER_HPP
#define ESC_DSHOT_DRIVER_HPP


#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>
#include <algorithm>

namespace abstractx::drivers {

// Supported ESC Output Protocols (Betaflight & iNav Full Suite)
enum class EscProtocol : uint8_t {
    StandardPwm   = 0, // 50 Hz .. 400 Hz Standard Servo/ESC PWM (1000..2000 us)
    FastPwm       = 1, // 500 Hz .. 2 kHz High-Frequency Analog PWM
    OneShot125    = 2, // 125 us .. 250 us Fast Analog PWM
    OneShot42     = 3, // 42 us .. 84 us Ultra-Fast Analog PWM
    MultiShot     = 4, // 5 us .. 25 us High-Speed Pulse Protocol
    DShot150      = 5, // 150 kbit/s Digital Protocol
    DShot300      = 6, // 300 kbit/s Digital Protocol
    DShot600      = 7, // 600 kbit/s Digital Protocol (Default)
    DShot1200     = 8, // 1.2 Mbit/s High-Speed Digital Protocol
    BiDirectionalDShot = 9 // DShot600 with hardware ERPM telemetry return packet
};

class EscDshotDriver {
public:
    constexpr EscDshotDriver() noexcept = default;

    // Make 64B TLP write packet for specified motor and protocol
    static Tlp64 make_motor_write_tlp(uint8_t motor_index, 
                                      uint16_t command, 
                                      uint8_t tag,
                                      EscProtocol protocol = EscProtocol::DShot600) noexcept {
        uint32_t target_addr = bar::EscBase + static_cast<uint32_t>(motor_index * 4);
        uint32_t value = 0;

        switch (protocol) {
            case EscProtocol::StandardPwm:
            case EscProtocol::FastPwm:
                // Standard/FastPWM command range: 1000..2000 us
                value = std::clamp<uint16_t>(command, 1000, 2000);
                break;

            case EscProtocol::OneShot125:
                // OneShot125 range: 125..250 us
                value = std::clamp<uint16_t>(command, 125, 250);
                break;

            case EscProtocol::OneShot42:
                // OneShot42 range: 42..84 us
                value = std::clamp<uint16_t>(command, 42, 84);
                break;

            case EscProtocol::MultiShot:
                // MultiShot range: 5..25 us
                value = std::clamp<uint16_t>(command, 5, 25);
                break;

            case EscProtocol::DShot150:
            case EscProtocol::DShot300:
            case EscProtocol::DShot600:
            case EscProtocol::DShot1200:
            case EscProtocol::BiDirectionalDShot:
            default:
                // DShot digital frame (11-bit throttle value 48..2047 + telemetry bit + 4-bit CRC)
                uint16_t packet = static_cast<uint16_t>((command & 0x07FF) << 1);
                uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
                value = static_cast<uint32_t>((packet << 4) | crc);
                break;
        }

        return Tlp64::make_mem_write(target_addr, value, tag);
    }
};

} // namespace abstractx::drivers

#endif // ESC_DSHOT_DRIVER_HPP
