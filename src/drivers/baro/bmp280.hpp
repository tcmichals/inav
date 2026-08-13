/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Bosch BMP280 Barometer Driver
 */

#ifndef BMP280_DRIVER_HPP
#define BMP280_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <cmath>

namespace abstractx::drivers::baro {

struct BaroSample {
    float pressure_pa{101325.0f};
    float altitude_cm{0.0f};
    float temperature_c{25.0f};
    uint64_t timestamp_ns{0};
};

class Bmp280 {
public:
    static constexpr uint8_t CHIP_ID = 0x58;

    static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        uint32_t raw_press = (static_cast<uint32_t>(p[0]) << 12) | (static_cast<uint32_t>(p[1]) << 4) | (p[2] >> 4);
        uint32_t raw_temp  = (static_cast<uint32_t>(p[3]) << 12) | (static_cast<uint32_t>(p[4]) << 4) | (p[5] >> 4);

        sample.pressure_pa = 101325.0f - (static_cast<float>(raw_press) / 16.0f);
        sample.temperature_c = 25.0f + (static_cast<float>(raw_temp) / 5120.0f);

        float ratio = sample.pressure_pa / 101325.0f;
        sample.altitude_cm = 44330.0f * (1.0f - std::pow(ratio, 0.190295f)) * 100.0f;

        return sample;
    }
};

} // namespace abstractx::drivers::baro

#endif // BMP280_DRIVER_HPP
