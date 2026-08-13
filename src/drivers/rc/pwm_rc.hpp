/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Multi-Channel Parallel PWM / PPM RC Receiver Driver
 */

#ifndef PWM_RC_DRIVER_HPP
#define PWM_RC_DRIVER_HPP

#include "crsf.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <algorithm>

namespace abstractx::drivers::rc {

class PwmRc {
public:
    static constexpr uint8_t MAX_PWM_CHANNELS = 8;

    static RcChannels parse_tlp(const Tlp64& tlp) noexcept {
        RcChannels rc{};
        const uint8_t* p = tlp.wire.payload;

        for (size_t i = 0; i < MAX_PWM_CHANNELS && (i * 2 + 1) < 40; ++i) {
            uint16_t pulse_us = static_cast<uint16_t>((p[i * 2] << 8) | p[i * 2 + 1]);
            rc.channels[i] = std::clamp<uint16_t>(pulse_us, 800, 2200);
        }

        rc.connected = (rc.channels[2] > 900);
        rc.failsafe = !rc.connected;
        return rc;
    }
};

} // namespace abstractx::drivers::rc

#endif // PWM_RC_DRIVER_HPP
