/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Futaba / FrSky SBUS RC Receiver Driver
 */

#ifndef SBUS_DRIVER_HPP
#define SBUS_DRIVER_HPP

#include "crsf.hpp"

namespace abstractx::drivers::rc {

class Sbus {
public:
    static RcChannels parse_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        if (tlp.wire.payload[0] == 0x0F) { // SBUS Header
            const uint8_t* p = &tlp.wire.payload[1];
            rc.channels[0] = static_cast<uint16_t>((p[0] | (p[1] << 8)) & 0x07FF);
            rc.channels[1] = static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x07FF);
            rc.channels[2] = static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x07FF);
            rc.channels[3] = static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x07FF);

            for (size_t i = 0; i < 4; ++i) {
                rc.channels[i] = static_cast<uint16_t>(988 + (rc.channels[i] - 172) * 5 / 8);
            }

            rc.failsafe = (p[22] & 0x08) != 0;
        }
        return rc;
    }
};

} // namespace abstractx::drivers::rc

#endif // SBUS_DRIVER_HPP
