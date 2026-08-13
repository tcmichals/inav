/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Universal AbstractX PCIe TLP IMU Driver Engine
 */

#ifndef IMU_PCIE_DRIVER_HPP
#define IMU_PCIE_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>
#include <array>
#include <span>

namespace abstractx::drivers {

enum class ImuSensorType : uint8_t {
    Icm42688P = 0,
    Bmi088    = 1,
    Mpu6000   = 2,
    Bmi270    = 3,
    Lsm6dso   = 4
};

struct RegisterInitPair {
    uint8_t reg_addr;
    uint8_t value;
};

struct ImuSample {
    std::array<float, 3> accel_g{0.0f, 0.0f, 1.0f};
    std::array<float, 3> gyro_deg_s{0.0f, 0.0f, 0.0f};
    float temperature_c{25.0f};
    uint64_t timestamp_ns{0};
    ImuSensorType sensor_type{ImuSensorType::Icm42688P};
};

class ImuPcieDriver {
public:
    constexpr ImuPcieDriver() noexcept = default;

    static std::span<const RegisterInitPair> get_init_sequence(ImuSensorType sensor) noexcept {
        if (sensor == ImuSensorType::Icm42688P) {
            static constexpr RegisterInitPair seq[] = {
                { 0x11, 0x01 }, { 0x4E, 0x0F }, { 0x4F, 0x06 }, { 0x50, 0x06 }
            };
            return std::span<const RegisterInitPair>(seq);
        } else if (sensor == ImuSensorType::Bmi088) {
            static constexpr RegisterInitPair seq[] = {
                { 0x7E, 0xB6 }, { 0x40, 0xA8 }, { 0x41, 0x03 }
            };
            return std::span<const RegisterInitPair>(seq);
        } else {
            static constexpr RegisterInitPair seq[] = {
                { 0x6B, 0x80 }, { 0x6B, 0x03 }, { 0x1A, 0x02 }, { 0x1B, 0x18 }
            };
            return std::span<const RegisterInitPair>(seq);
        }
    }

    static ImuSample parse_tlp(const Tlp64& tlp, ImuSensorType chip = ImuSensorType::Icm42688P) noexcept {
        ImuSample sample{};
        sample.sensor_type = chip;
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_ax = static_cast<int16_t>((p[0] << 8) | p[1]);
        int16_t raw_ay = static_cast<int16_t>((p[2] << 8) | p[3]);
        int16_t raw_az = static_cast<int16_t>((p[4] << 8) | p[5]);

        int16_t raw_gx = static_cast<int16_t>((p[6] << 8) | p[7]);
        int16_t raw_gy = static_cast<int16_t>((p[8] << 8) | p[9]);
        int16_t raw_gz = static_cast<int16_t>((p[10] << 8) | p[11]);

        int16_t raw_temp = static_cast<int16_t>((p[12] << 8) | p[13]);

        if (chip == ImuSensorType::Icm42688P) {
            sample.accel_g[0] = static_cast<float>(raw_ax) / 2048.0f;
            sample.accel_g[1] = static_cast<float>(raw_ay) / 2048.0f;
            sample.accel_g[2] = static_cast<float>(raw_az) / 2048.0f;

            sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.4f;
            sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.4f;
            sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.4f;

            sample.temperature_c = 25.0f + (static_cast<float>(raw_temp) / 326.8f);
        } else {
            sample.accel_g[0] = static_cast<float>(raw_ax) / 4096.0f;
            sample.accel_g[1] = static_cast<float>(raw_ay) / 4096.0f;
            sample.accel_g[2] = static_cast<float>(raw_az) / 4096.0f;

            sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.4f;
            sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.4f;
            sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.4f;
        }

        return sample;
    }
};

} // namespace abstractx::drivers

#endif // IMU_PCIE_DRIVER_HPP
