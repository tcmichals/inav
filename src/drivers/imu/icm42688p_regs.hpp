/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TDK ICM-42688-P: registers, configuration encoding and sample decoding. No bus, no runtime: shared by
 * the driver (icm42688p.hpp).
 * Upstream references: iNavFlight/inav accgyro_icm42605.c, Betaflight accgyro_spi_icm42688p.c.
 */

#ifndef ICM42688P_REGS_HPP
#define ICM42688P_REGS_HPP

#include "imu_base.hpp"
#include <cstdint>
#include <span>

namespace abstractx::drivers::imu {

// @impl [SPEC-DRV-01] docs/HARDWARE_DRIVERS.md (one copy of the chip's facts)
namespace icm42688p_regs {
    // Bank 0
    static constexpr uint8_t REG_TEMP_DATA1      = 0x1Du;
    static constexpr uint8_t REG_ACCEL_DATA_X1   = 0x1Fu;
    static constexpr uint8_t REG_GYRO_DATA_X1    = 0x25u;
    static constexpr uint8_t REG_INT_CONFIG       = 0x14u;
    static constexpr uint8_t REG_FIFO_CONFIG      = 0x16u;
    static constexpr uint8_t REG_INTF_CONFIG1     = 0x4Du;
    static constexpr uint8_t REG_PWR_MGMT0        = 0x4Eu;
    static constexpr uint8_t REG_GYRO_CONFIG0     = 0x4Fu;
    static constexpr uint8_t REG_ACCEL_CONFIG0    = 0x50u;
    static constexpr uint8_t REG_GYRO_ACCEL_CFG0  = 0x52u;
    static constexpr uint8_t REG_INT_CONFIG0      = 0x63u;
    static constexpr uint8_t REG_INT_CONFIG1      = 0x64u;
    static constexpr uint8_t REG_INT_SOURCE0      = 0x65u;
    static constexpr uint8_t REG_WHO_AM_I         = 0x75u;
    static constexpr uint8_t REG_DEVICE_CONFIG    = 0x11u; // soft reset bit
    static constexpr uint8_t REG_BANK_SEL         = 0x76u;

    // Bank 1: gyro anti-alias filter
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC3 = 0x0Cu;
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC4 = 0x0Du;
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC5 = 0x0Eu;

    // Bank 2: accel anti-alias filter
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC2 = 0x03u;
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC3 = 0x04u;
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC4 = 0x05u;

    static constexpr uint8_t WHO_AM_I_ICM42688P = 0x47u;
    static constexpr uint8_t WHO_AM_I_ICM42605  = 0x42u;

    // PWR_MGMT0
    static constexpr uint8_t ACCEL_MODE_LN = (3u << 0u); // low-noise
    static constexpr uint8_t GYRO_MODE_LN  = (3u << 2u); // low-noise

    // GYRO_CONFIG0 / ACCEL_CONFIG0: ODR [3:0], FS_SEL [7:5]
    static constexpr uint8_t GYRO_FS_2000DPS = (0u << 5u);
    static constexpr uint8_t GYRO_ODR_8KHZ   = 0x03u;
    static constexpr uint8_t ACCEL_FS_16G    = (0u << 5u);
    static constexpr uint8_t ACCEL_ODR_8KHZ  = 0x03u;

    // INT1: push-pull, active-high, pulsed
    static constexpr uint8_t INT1_POLARITY_HIGH = (1u << 0u);
    static constexpr uint8_t INT1_DRIVE_PP      = (1u << 1u);
    static constexpr uint8_t INT1_MODE_PULSED   = (0u << 2u);

    // INT_CONFIG1: 8 us pulse, async reset off
    static constexpr uint8_t INT_TPULSE_8US         = (1u << 6u);
    static constexpr uint8_t INT_TDEASSERT_DISABLED  = (1u << 5u);
    static constexpr uint8_t INT_ASYNC_RESET_BIT     = (1u << 4u);

    // INT_SOURCE0: DRDY to INT1
    static constexpr uint8_t UI_DRDY_INT1_EN = (1u << 3u);

    // INTF_CONFIG1: disable AFSR (adaptive frequency scaling causes ODR jitter)
    static constexpr uint8_t AFSR_DISABLE = 0x40u;

    // AAF for a 536 Hz corner at 8 kHz ODR (TDK application note): delt 6, deltsqr 36, bitshift 4
    static constexpr uint8_t AAF_DELT       = 6u;
    static constexpr uint8_t AAF_DELTSQR_L  = 36u;
    static constexpr uint8_t AAF_DELTSQR_H  = 0u;
    static constexpr uint8_t AAF_BITSHIFT    = 4u;
}

namespace icm42688p {

[[nodiscard]] constexpr uint8_t odr_from_hz(uint32_t hz) noexcept {
    if (hz >= 8000u) { return 0x03u; }
    if (hz >= 4000u) { return 0x04u; }
    if (hz >= 2000u) { return 0x05u; }
    return 0x06u; // 1 kHz
}

[[nodiscard]] constexpr uint8_t gyro_fs_from_dps(uint16_t dps) noexcept {
    if (dps <= 250u)  { return 3u; }
    if (dps <= 500u)  { return 2u; }
    if (dps <= 1000u) { return 1u; }
    return 0u; // 2000 dps
}

[[nodiscard]] constexpr uint8_t accel_fs_from_g(uint8_t g) noexcept {
    if (g <= 2u)  { return 3u; }
    if (g <= 4u)  { return 2u; }
    if (g <= 8u)  { return 1u; }
    return 0u; // 16 g
}

inline constexpr uint8_t kBurstReg = icm42688p_regs::REG_TEMP_DATA1;   // temp, accel, gyro
inline constexpr uint8_t kBurstLen = 14u;

[[nodiscard]] constexpr int16_t be16(const uint8_t* p) noexcept {
    return static_cast<int16_t>(static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8u) | p[1]));
}

// The 14-byte burst from TEMP_DATA1, big-endian: temp, accel X Y Z, gyro X Y Z. Scaled for the
// full-scale ranges the driver sets by default (16 g: 2048 LSB/g; 2000 dps: 16.4 LSB/dps).
[[nodiscard]] inline ImuSample parse_burst(std::span<const uint8_t> raw, uint64_t timestamp_ns) noexcept {
    ImuSample s{};
    s.timestamp_ns = timestamp_ns;
    if (raw.size() < kBurstLen) {
        return s;
    }
    const uint8_t* p = raw.data();
    for (size_t i = 0; i < 3u; ++i) {
        s.accel_g[i] = static_cast<float>(be16(p + 2u + 2u * i)) * (1.0f / 2048.0f);
        s.gyro_deg_s[i] = static_cast<float>(be16(p + 8u + 2u * i)) * (1.0f / 16.4f);
    }
    s.temperature_c = (static_cast<float>(be16(p)) / 132.48f) + 25.0f;   // datasheet 4.13
    return s;
}

} // namespace icm42688p
} // namespace abstractx::drivers::imu

#endif // ICM42688P_REGS_HPP
