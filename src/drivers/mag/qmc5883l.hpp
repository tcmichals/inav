/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated QST QMC5883L Magnetometer / Compass Driver
 */

#ifndef QMC5883L_DRIVER_HPP
#define QMC5883L_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers::mag {

struct MagSample {
    std::array<float, 3> mag_gauss{0.0f, 0.0f, 0.0f};
    uint64_t timestamp_ns{0};
};

class Qmc5883L {
public:
    static constexpr uint8_t CHIP_ID = 0xFF;

    static MagSample parse_tlp(const Tlp64& tlp) noexcept {
        MagSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_x = static_cast<int16_t>((p[1] << 8) | p[0]);
        int16_t raw_y = static_cast<int16_t>((p[3] << 8) | p[2]);
        int16_t raw_z = static_cast<int16_t>((p[5] << 8) | p[4]);

        sample.mag_gauss[0] = static_cast<float>(raw_x) / 3000.0f;
        sample.mag_gauss[1] = static_cast<float>(raw_y) / 3000.0f;
        sample.mag_gauss[2] = static_cast<float>(raw_z) / 3000.0f;

        return sample;
    }
};

} // namespace abstractx::drivers::mag

#endif // QMC5883L_DRIVER_HPP
