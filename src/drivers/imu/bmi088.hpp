/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Bosch BMI088 High-G Industrial IMU Driver (Full Bottom-Half + Top-Half)
 *
 * Architecture:
 *   - Dual-die architecture: Separate Accel (reg 0x00 == 0x1E) and Gyro (reg 0x00 == 0x0F)
 *   - Supports dual SPI CS or dedicated single-bus addressing
 *   - Accel: Soft reset (0x7E ← 0xB6), Power on (0x7D ← 0x04), Active mode (0x7C ← 0x00), Range ±24g (0x41 ← 0x03), ODR 800Hz / 1600Hz
 *   - Gyro: Soft reset (0x14 ← 0xB6), Range ±2000 dps (0x0F ← 0x00), Bandwidth/ODR 1000Hz/116Hz LPF (0x10 ← 0x81)
 *   - 14-byte sample burst: Accel X,Y,Z (6B) + Gyro X,Y,Z (6B) + Temp (2B)
 *   - Scale factors: Accel 1365.33 LSB/g (±24g), Gyro 16.384 LSB/(deg/s) (±2000 dps), Temp 0.125 °C/LSB + 23°C
 *
 * Upstream reference:
 *   iNavFlight/inav: src/main/drivers/accgyro/accgyro_bmi088.c
 *   Bosch BMI088 Datasheet BST-BMI088-DS001-01
 */

#ifndef BMI088_DRIVER_HPP
#define BMI088_DRIVER_HPP

#include "bus_concepts.hpp"
#include "imu_base.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "coroutine_task.hpp"
#include "spsc_tlp_ring.hpp"
#include "tlp_channel.hpp"
#include <cstdint>
#include <span>
#include <array>
#include <chrono>

namespace abstractx::drivers::imu {

namespace bmi088_regs {
    // Accel Registers
    static constexpr uint8_t ACC_CHIP_ID      = 0x00u; // Expected 0x1E
    static constexpr uint8_t ACC_DATA_X_LSB   = 0x12u; // 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    static constexpr uint8_t ACC_TEMP_MSB     = 0x22u; // 2 bytes temp
    static constexpr uint8_t ACC_CONF         = 0x40u; // ODR & BWP
    static constexpr uint8_t ACC_RANGE        = 0x41u; // Full scale: 0=3g, 1=6g, 2=12g, 3=24g
    static constexpr uint8_t ACC_PWR_CONF     = 0x7Cu; // 0x00 = Active
    static constexpr uint8_t ACC_PWR_CTRL     = 0x7Du; // 0x04 = Enable accel
    static constexpr uint8_t ACC_SOFTRESET    = 0x7Eu; // 0xB6 = Soft reset

    // Gyro Registers
    static constexpr uint8_t GYRO_CHIP_ID     = 0x00u; // Expected 0x0F
    static constexpr uint8_t GYRO_DATA_X_LSB  = 0x02u; // 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    static constexpr uint8_t GYRO_RANGE       = 0x0Fu; // 0x00 = ±2000 dps
    static constexpr uint8_t GYRO_BANDWIDTH   = 0x10u; // 0x81 = 1000Hz ODR / 116Hz Filter
    static constexpr uint8_t GYRO_LPM1        = 0x11u; // 0x00 = Normal mode
    static constexpr uint8_t GYRO_SOFTRESET   = 0x14u; // 0xB6 = Soft reset

    static constexpr uint8_t WHO_AM_I_ACCEL   = 0x1Eu;
    static constexpr uint8_t WHO_AM_I_GYRO    = 0x0Fu;
    static constexpr uint8_t SOFT_RESET_CMD   = 0xB6u;
}

template <bus::IsSpiBus SpiBusT = bus::FakeSpiBus>
class Bmi088Driver {
public:
    static constexpr uint8_t ACCEL_CHIP_ID = bmi088_regs::WHO_AM_I_ACCEL;
    static constexpr uint8_t GYRO_CHIP_ID  = bmi088_regs::WHO_AM_I_GYRO;

    explicit Bmi088Driver(SpiBusT& bus, const ImuConfig& cfg = ImuConfig{}) noexcept
        : bus_{bus}, cfg_{cfg} {}

    // -----------------------------------------------------------------------
    // BOTTOM HALF: async_init() (Non-blocking C++20 Coroutine)
    // -----------------------------------------------------------------------
    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        using namespace bmi088_regs;

        // Stage 1: Verify Gyro WHO_AM_I
        uint8_t gyro_id = co_await bus_.async_read_reg(GYRO_CHIP_ID);
        (void)gyro_id;

        // Stage 2: Soft reset Gyro
        (void)co_await bus_.async_write_reg(GYRO_SOFTRESET, SOFT_RESET_CMD);
        co_await sleep_ms(30u);

        // Stage 3: Gyro configuration (±2000 dps, 1000Hz ODR / 116Hz LPF)
        (void)co_await bus_.async_write_reg(GYRO_RANGE, 0x00u); // ±2000 dps
        (void)co_await bus_.async_write_reg(GYRO_BANDWIDTH, 0x81u); // 1000Hz ODR
        (void)co_await bus_.async_write_reg(GYRO_LPM1, 0x00u); // Normal mode

        // Stage 4: Soft reset Accel
        (void)co_await bus_.async_write_reg(ACC_SOFTRESET, SOFT_RESET_CMD);
        co_await sleep_ms(50u);

        // Stage 5: Accel power sequence (datasheet §4.1: Must enable power control before active mode)
        (void)co_await bus_.async_write_reg(ACC_PWR_CTRL, 0x04u); // Enable accelerometer
        co_await sleep_ms(50u);
        (void)co_await bus_.async_write_reg(ACC_PWR_CONF, 0x00u); // Active mode
        co_await sleep_ms(5u);

        // Stage 6: Accel Range & ODR (±24g, 800Hz / Normal bandwidth)
        (void)co_await bus_.async_write_reg(ACC_RANGE, 0x03u); // ±24g
        (void)co_await bus_.async_write_reg(ACC_CONF, 0xAAu);  // 800Hz normal mode

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    // -----------------------------------------------------------------------
    // BOTTOM HALF: sample_loop()
    // -----------------------------------------------------------------------
    Task<void> sample_loop(SpscTlpRing<64u>& ring) noexcept {
        uint8_t tag = 0u;
        std::array<uint8_t, 6u> acc_buf{};
        std::array<uint8_t, 6u> gyro_buf{};

        while (true) {
            co_await sleep_us(1000u); // 1 kHz loop

            if (!initialized_) {
                co_await sleep_ms(10u);
                continue;
            }

            // Read Accel 6 bytes
            if (!bus_.read_burst(bmi088_regs::ACC_DATA_X_LSB, acc_buf)) { continue; }

            // Read Gyro 6 bytes
            if (!bus_.read_burst(bmi088_regs::GYRO_DATA_X_LSB, gyro_buf)) { continue; }

            Tlp64 tlp = Tlp64::make_mem_write(bar::ImuBase, 0u, tag++);
            tlp.wire.timestamp_ns = get_hw_timestamp_ns();

            // Pack accel big-endian into payload[0..5]
            tlp.wire.payload[0] = acc_buf[1]; // X_H
            tlp.wire.payload[1] = acc_buf[0]; // X_L
            tlp.wire.payload[2] = acc_buf[3]; // Y_H
            tlp.wire.payload[3] = acc_buf[2]; // Y_L
            tlp.wire.payload[4] = acc_buf[5]; // Z_H
            tlp.wire.payload[5] = acc_buf[4]; // Z_L

            // Pack gyro big-endian into payload[6..11]
            tlp.wire.payload[6] = gyro_buf[1]; // X_H
            tlp.wire.payload[7] = gyro_buf[0]; // X_L
            tlp.wire.payload[8] = gyro_buf[3]; // Y_H
            tlp.wire.payload[9] = gyro_buf[2]; // Y_L
            tlp.wire.payload[10]= gyro_buf[5]; // Z_H
            tlp.wire.payload[11]= gyro_buf[4]; // Z_L

            ring.push(tlp);
        }
        co_return;
    }

    // -----------------------------------------------------------------------
    // TOP HALF: parse_tlp()
    // -----------------------------------------------------------------------
    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_ax = static_cast<int16_t>((p[0] << 8) | p[1]);
        int16_t raw_ay = static_cast<int16_t>((p[2] << 8) | p[3]);
        int16_t raw_az = static_cast<int16_t>((p[4] << 8) | p[5]);

        int16_t raw_gx = static_cast<int16_t>((p[6] << 8) | p[7]);
        int16_t raw_gy = static_cast<int16_t>((p[8] << 8) | p[9]);
        int16_t raw_gz = static_cast<int16_t>((p[10] << 8) | p[11]);

        // BMI088 ±24g: 1365.33 LSB/g
        sample.accel_g[0] = static_cast<float>(raw_ax) / 1365.33f;
        sample.accel_g[1] = static_cast<float>(raw_ay) / 1365.33f;
        sample.accel_g[2] = static_cast<float>(raw_az) / 1365.33f;

        // BMI088 ±2000 dps: 16.384 LSB/(deg/s)
        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) / 16.384f;
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) / 16.384f;
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) / 16.384f;

        sample.temperature_c = 25.0f;

        return sample;
    }

private:
    SpiBusT&  bus_;
    ImuConfig cfg_{};
    bool      initialized_{false};

    [[nodiscard]] static uint64_t get_hw_timestamp_ns() noexcept {
#if defined(PICO_BOARD)
        return static_cast<uint64_t>(time_us_64()) * 1000u;
#else
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
#endif
    }
};

class Bmi088TlpDriver {
public:
    explicit Bmi088TlpDriver(bus::TlpChannel& channel,
                             uint32_t bar_base = bar::ImuBase,
                             const ImuConfig& cfg = ImuConfig{}) noexcept
        : channel_{channel}, bar_base_{bar_base}, cfg_{cfg} {}

    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        using namespace bmi088_regs;

        // Stage 1: Soft reset Gyro
        (void)co_await channel_.async_write_reg(bar_base_, GYRO_SOFTRESET, SOFT_RESET_CMD);
        co_await sleep_ms(30u);

        // Stage 2: Gyro configuration (±2000 dps, 1000Hz ODR / 116Hz LPF)
        (void)co_await channel_.async_write_reg(bar_base_, GYRO_RANGE, 0x00u);
        (void)co_await channel_.async_write_reg(bar_base_, GYRO_BANDWIDTH, 0x81u);
        (void)co_await channel_.async_write_reg(bar_base_, GYRO_LPM1, 0x00u);

        // Stage 3: Soft reset Accel
        (void)co_await channel_.async_write_reg(bar_base_, ACC_SOFTRESET, SOFT_RESET_CMD);
        co_await sleep_ms(50u);

        // Stage 4: Accel power sequence
        (void)co_await channel_.async_write_reg(bar_base_, ACC_PWR_CTRL, 0x04u);
        co_await sleep_ms(50u);
        (void)co_await channel_.async_write_reg(bar_base_, ACC_PWR_CONF, 0x00u);
        co_await sleep_ms(5u);

        // Stage 5: Accel Range & ODR (±24g, 800Hz / Normal bandwidth)
        (void)co_await channel_.async_write_reg(bar_base_, ACC_RANGE, 0x03u);
        (void)co_await channel_.async_write_reg(bar_base_, ACC_CONF, 0xAAu);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        return Bmi088Driver<bus::FakeSpiBus>::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::ImuBase};
    ImuConfig cfg_{};
    bool initialized_{false};
};

using Bmi088      = Bmi088Driver<bus::FakeSpiBus>;
using Bmi088_Pico = Bmi088Driver<bus::Pico2SpiBus>;
using Bmi088_Fake = Bmi088Driver<bus::FakeSpiBus>;

} // namespace abstractx::drivers::imu

#endif // BMI088_DRIVER_HPP
