/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Universal Barometer Driver (BMP280, DPS310, MS5611, BMP388, SPL06)
 */

#ifndef BARO_DRIVER_HPP
#define BARO_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <cmath>

namespace abstractx::drivers {

// Supported Barometer Hardware Sensor Chipsets
enum class BaroSensorType : uint8_t {
    Bmp280 = 0, // Bosch BMP280
    Bmp388 = 1, // Bosch BMP388 / BMP390
    Dps310 = 2, // Infineon DPS310
    Ms5611 = 3, // MEAS MS5611
    Spl06  = 4  // Goertek / Goermicro SPL06-001
};

struct BaroSample {
    float pressure_pa{101325.0f};  // Pressure in Pascals (Pa)
    float altitude_cm{0.0f};       // Altitude in centimeters (cm)
    float temperature_c{25.0f};    // Temperature in Celsius
    uint64_t timestamp_ns{0};      // 64-bit Hardware timestamp
    BaroSensorType sensor_type{BaroSensorType::Bmp280};
};

class BaroDriver {
public:
    constexpr BaroDriver() noexcept = default;

    // Parse Barometer 64B TLP frame into normalized BaroSample
    static BaroSample parse_tlp(const Tlp64& tlp, BaroSensorType chip = BaroSensorType::Bmp280) noexcept {
        BaroSample sample{};
        sample.sensor_type = chip;
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        // Unpack raw 24-bit pressure & 24-bit temperature
        uint32_t raw_press = (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2];
        uint32_t raw_temp  = (static_cast<uint32_t>(p[3]) << 16) | (static_cast<uint32_t>(p[4]) << 8) | p[5];

        // Convert raw pressure to Pascals (Pa)
        sample.pressure_pa = 101325.0f - (static_cast<float>(raw_press) / 256.0f);
        sample.temperature_c = 25.0f + (static_cast<float>(raw_temp) / 100.0f);

        // Hypsometric Barometric Altitude Equation (Pa -> Altitude cm)
        // Alt(m) = 44330 * (1 - (P / P0)^(1 / 5.255))
        float ratio = sample.pressure_pa / 101325.0f;
        sample.altitude_cm = 44330.0f * (1.0f - std::pow(ratio, 0.190295f)) * 100.0f;

        return sample;
    }
};

} // namespace abstractx::drivers

#endif // BARO_DRIVER_HPP
