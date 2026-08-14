/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Portable GPS Sample & Data Structures
 *
 * Zero-allocation POD structs for 3D GNSS position, velocity, and satellite status.
 * Matches iNav 7.x and Betaflight 4.x GPS parameter standards.
 */

#ifndef GPS_TYPES_HPP
#define GPS_TYPES_HPP

#include <cstdint>

namespace abstractx::drivers::gps {

enum class GpsProvider : uint8_t {
    Ublox = 0,
    Nmea  = 1,
    Msp   = 2
};

enum class GpsFixType : uint8_t {
    NoFix    = 0,
    Fix2D    = 1,
    Fix3D    = 2,
    Fix3DDgps = 3
};

struct alignas(8) GpsSample {
    int32_t  latitude_1e7{0};          // Latitude in 1e-7 degrees (e.g. 377749000 = 37.7749 deg)
    int32_t  longitude_1e7{0};         // Longitude in 1e-7 degrees (e.g. -1224194000 = -122.4194 deg)
    int32_t  altitude_cm{0};           // Altitude above Mean Sea Level (MSL) in cm
    int16_t  vel_n_cms{0};             // Velocity North in cm/s
    int16_t  vel_e_cms{0};             // Velocity East in cm/s
    int16_t  vel_d_cms{0};             // Velocity Down in cm/s
    uint16_t ground_course_decideg{0}; // Ground course / heading in 0.1 deg (0..3599)
    uint16_t hdop_centi{9900};         // Horizontal Dilution of Precision in 0.01 units (e.g. 150 = 1.50)
    uint8_t  num_sats{0};              // Number of satellites used in navigation solution
    GpsFixType fix_type{GpsFixType::NoFix}; // Fix quality status
    bool     valid{false};             // Data validity flag
    uint64_t timestamp_ns{0};          // Hardware nanosecond timestamp
};

} // namespace abstractx::drivers::gps

#endif // GPS_TYPES_HPP
