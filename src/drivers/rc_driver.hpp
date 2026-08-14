/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Multi-Protocol RC Receiver Driver (CRSF, SBUS, IBUS, SRXL2, PWM/PPM)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/rx/crsf.c, src/main/rx/sbus.c
 *   - Upstream INAV: src/main/rx/crsf.c, src/main/rx/sbus.c
 */

#ifndef RC_DRIVER_HPP
#define RC_DRIVER_HPP


#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>
#include <span>

namespace abstractx::drivers {

enum class RcProtocol : uint8_t {
    Crsf  = 0, // ExpressLRS / TBS Crossfire (200 Hz / 500 Hz)
    Sbus  = 1, // Futaba / FrSky SBUS (100 Hz inverted serial)
    Ibus  = 2, // FlySky IBUS (115200 baud serial)
    Srxl2 = 3, // Spektrum SRXL2 / DSMX (115200 baud serial)
    Pwm   = 4, // Multi-channel parallel PWM input
    Ppm   = 5  // Pulse-Position Modulation single-wire pulse stream
};

struct RcChannels {
    std::array<uint16_t, 16> channels{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
                                      1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Roll, Pitch, Throttle, Yaw, Aux1..12
    bool failsafe{false};
    bool connected{true};
    uint8_t link_quality{100}; // 0..100% Signal Link Quality (LQ)
    int8_t  rssi_dbm{-60};      // Received Signal Strength Indicator (dBm)
};

class RcDriver {
public:
    constexpr RcDriver() noexcept = default;

    // Parse CRSF 64B TLP frame (ExpressLRS / Crossfire)
    static RcChannels parse_crsf_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        if (tlp.wire.payload[0] == 0x16) { // CRSF_FRAMETYPE_RC_CHANNELS_PACKED
            const uint8_t* p = &tlp.wire.payload[1];
            rc.channels[0] = static_cast<uint16_t>((p[0] | (p[1] << 8)) & 0x07FF);
            rc.channels[1] = static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
            rc.channels[2] = static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
            rc.channels[3] = static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x07FF);

            // Scale 11-bit CRSF range (172..1811) to microseconds (988..2012 us)
            for (size_t i = 0; i < 4; ++i) {
                rc.channels[i] = static_cast<uint16_t>(988 + (rc.channels[i] - 172) * 5 / 8);
            }
        }
        return rc;
    }

    // Parse SBUS 64B TLP frame (Futaba / FrSky)
    static RcChannels parse_sbus_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        if (tlp.wire.payload[0] == 0x0F) { // SBUS Header Byte
            const uint8_t* p = &tlp.wire.payload[1];
            rc.channels[0] = static_cast<uint16_t>((p[0] | (p[1] << 8)) & 0x07FF);
            rc.channels[1] = static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
            rc.channels[2] = static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
            rc.channels[3] = static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x07FF);

            // Scale 11-bit SBUS range (172..1811) to microseconds
            for (size_t i = 0; i < 4; ++i) {
                rc.channels[i] = static_cast<uint16_t>(988 + (rc.channels[i] - 172) * 5 / 8);
            }

            // Check Failsafe bit in byte 23
            rc.failsafe = (p[22] & 0x08) != 0;
        }
        return rc;
    }

    // Parse IBUS 64B TLP frame (FlySky)
    static RcChannels parse_ibus_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        if (tlp.wire.payload[0] == 0x20 && tlp.wire.payload[1] == 0x40) { // IBUS Header
            for (size_t i = 0; i < 6; ++i) {
                rc.channels[i] = static_cast<uint16_t>(tlp.wire.payload[2 + i*2] | (tlp.wire.payload[3 + i*2] << 8));
            }
        }
        return rc;
    }

    // Parse Multi-Channel Parallel PWM / PPM Input TLP frame
    static RcChannels parse_pwm_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        for (size_t i = 0; i < 8; ++i) {
            uint16_t raw_us = static_cast<uint16_t>(tlp.wire.payload[i*2] | (tlp.wire.payload[i*2 + 1] << 8));
            if (raw_us >= 800 && raw_us <= 2200) {
                rc.channels[i] = raw_us;
            }
        }
        return rc;
    }
};

} // namespace abstractx::drivers

#endif // RC_DRIVER_HPP
