/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Infineon DPS310 High-Precision Barometer Driver
 */

#ifndef DPS310_HPP
#define DPS310_HPP

#include "asp_tlp64.hpp"
#include "baro_driver.hpp"
#include <cstdint>

namespace abstractx::drivers::baro {

class Dps310 {
public:
    constexpr Dps310() noexcept = default;

    static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        // 24-bit raw pressure (Payload 0..2)
        int32_t raw_p = static_cast<int32_t>(
            ((tlp.wire.payload[0] & 0x80 ? 0xFF000000 : 0) |
             (tlp.wire.payload[0] << 16) |
             (tlp.wire.payload[1] << 8) |
              tlp.wire.payload[2]));

        // Scale raw pressure to Pascals & meters
        sample.pressure_pa = 101325.0f + (static_cast<float>(raw_p) / 524288.0f) * 100.0f;
        sample.temperature_c = 25.0f;
        sample.altitude_cm = 44330.0f * (1.0f - std::pow(sample.pressure_pa / 101325.0f, 0.190295f)) * 100.0f;

        return sample;
    }
};

} // namespace abstractx::drivers::baro

#endif // DPS310_HPP
