/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - iSentek IST8310 3-Axis Magnetometer Driver
 */

#ifndef IST8310_HPP
#define IST8310_HPP

#include "asp_tlp64.hpp"
#include "mag_driver.hpp"
#include <cstdint>

namespace abstractx::drivers::mag {

class Ist8310 {
public:
    constexpr Ist8310() noexcept = default;

    static MagSample parse_tlp(const Tlp64& tlp) noexcept {
        MagSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        // IST8310 16-bit little-endian output (0.3 uT/LSB)
        int16_t raw_x = static_cast<int16_t>(tlp.wire.payload[0] | (tlp.wire.payload[1] << 8));
        int16_t raw_y = static_cast<int16_t>(tlp.wire.payload[2] | (tlp.wire.payload[3] << 8));
        int16_t raw_z = static_cast<int16_t>(tlp.wire.payload[4] | (tlp.wire.payload[5] << 8));

        sample.mag_gauss[0] = static_cast<float>(raw_x) * 0.003f; // uT to Gauss
        sample.mag_gauss[1] = static_cast<float>(raw_y) * 0.003f;
        sample.mag_gauss[2] = static_cast<float>(raw_z) * 0.003f;

        return sample;
    }
};

} // namespace abstractx::drivers::mag

#endif // IST8310_HPP
