/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated MEAS MS5611 Barometer Driver
 */

#ifndef MS5611_DRIVER_HPP
#define MS5611_DRIVER_HPP

#include "bmp280.hpp"

namespace abstractx::drivers::baro {

class Ms5611 {
public:
    static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        uint32_t raw_press = (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2];
        uint32_t raw_temp  = (static_cast<uint32_t>(p[3]) << 16) | (static_cast<uint32_t>(p[4]) << 8) | p[5];

        sample.pressure_pa = static_cast<float>(raw_press);
        sample.temperature_c = static_cast<float>(raw_temp) / 100.0f;

        float ratio = sample.pressure_pa / 101325.0f;
        sample.altitude_cm = 44330.0f * (1.0f - std::pow(ratio, 0.190295f)) * 100.0f;

        return sample;
    }
};

} // namespace abstractx::drivers::baro

#endif // MS5611_DRIVER_HPP
