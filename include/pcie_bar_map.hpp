/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `AbstractX` - PCIe BAR and Register Address Map for Sensor & Actuator Message Channels
 */

#ifndef PCIE_BAR_MAP_HPP
#define PCIE_BAR_MAP_HPP

#include <cstdint>

namespace abstractx {

namespace bar {
    static constexpr uint32_t ImuBase       = 0x1000u;
    static constexpr uint32_t BaroBase      = 0x2000u;
    static constexpr uint32_t MagBase       = 0x3000u;
    static constexpr uint32_t EscBase       = 0x4000u;
    static constexpr uint32_t GpsBase       = 0x5000u;
    static constexpr uint32_t RcBase        = 0x6000u;
    static constexpr uint32_t DisplayBase   = 0x7000u;
    static constexpr uint32_t LedBase       = 0x8000u;
    static constexpr uint32_t PowerBase     = 0x9000u;
    static constexpr uint32_t PitotBase     = 0xA000u;
    static constexpr uint32_t TelemetryBase = 0xB000u;
} // namespace bar

namespace reg {
    namespace imu {
        static constexpr uint32_t Control        = 0x00u;
        static constexpr uint32_t Status         = 0x04u;
        static constexpr uint32_t ContinuousAddr = 0x10u;
    } // namespace imu
    namespace baro {
        static constexpr uint32_t Control        = 0x00u;
        static constexpr uint32_t Status         = 0x04u;
        static constexpr uint32_t ContinuousAddr = 0x10u;
    } // namespace baro
    namespace mag {
        static constexpr uint32_t Control        = 0x00u;
        static constexpr uint32_t Status         = 0x04u;
        static constexpr uint32_t ContinuousAddr = 0x10u;
    } // namespace mag
} // namespace reg

} // namespace abstractx

#endif // PCIE_BAR_MAP_HPP
