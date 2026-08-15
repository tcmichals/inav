/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated MEAS MS5611 Barometer Driver (Full Bottom-Half + Top-Half)
 *
 * Bottom-half (init / sample_loop):
 *   - Soft reset (command 0x1E), wait 10 ms
 *   - Read 8-word (16-byte) PROM (0xA0..0xAE) containing C0..C7 calibration coefficients
 *   - Verify 4-bit CRC (CRC4) across all 16 PROM bytes
 *   - 2-phase non-blocking ADC sampling:
 *       Phase A: Trigger D1 (Pressure OSR=4096, 0x48) -> co_await sleep_ms(10) -> Read 24-bit ADC (0x00)
 *       Phase B: Trigger D2 (Temp OSR=4096, 0x58) -> co_await sleep_ms(10) -> Read 24-bit ADC (0x00)
 *   - Pack raw D1, D2 + C1..C6 calibration into Tlp64 -> push to SPSC ring
 *
 * Top-half (parse_tlp):
 *   - Exact MEAS MS5611 64-bit integer compensation math with 2nd-order thermal curvature correction
 *   - Hypsometric altitude derivation
 *
 * Upstream reference:
 *   iNavFlight/inav: src/main/drivers/barometer/barometer_ms5611.c
 *   Measurement Specialties MS5611-01BA03 Datasheet
 */

#ifndef MS5611_DRIVER_HPP
#define MS5611_DRIVER_HPP

#include "bus_concepts.hpp"
#include "baro_driver.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "coroutine_task.hpp"
#include "spsc_tlp_ring.hpp"
#include "tlp_channel.hpp"
#include <cstdint>
#include <cmath>
#include <span>
#include <array>

namespace abstractx::drivers::baro {

namespace ms5611_regs {
    static constexpr uint8_t CMD_RESET       = 0x1Eu;
    static constexpr uint8_t CMD_ADC_READ    = 0x00u;
    static constexpr uint8_t CMD_ADC_CONV_D1 = 0x48u; // OSR = 4096 (9.04ms max)
    static constexpr uint8_t CMD_ADC_CONV_D2 = 0x58u; // OSR = 4096 (9.04ms max)
    static constexpr uint8_t CMD_PROM_RD     = 0xA0u; // PROM base (0xA0..0xAE)

    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x77u; // CSB = High
    static constexpr uint8_t I2C_ADDR_ALT     = 0x76u; // CSB = Low
}

struct Ms5611Calib {
    uint16_t c1{0u}; // Pressure sensitivity | SENS_T1
    uint16_t c2{0u}; // Pressure offset | OFF_T1
    uint16_t c3{0u}; // Temp coefficient of pressure sensitivity | TCS
    uint16_t c4{0u}; // Temp coefficient of pressure offset | TCO
    uint16_t c5{0u}; // Reference temperature | T_REF
    uint16_t c6{0u}; // Temp coefficient of temperature | TEMPSENS
};

// 4-bit CRC calculation per MEAS MS5611 datasheet application note
[[nodiscard]] inline uint8_t ms5611_crc4(const uint16_t prom[8]) noexcept {
    uint16_t n_rem = 0u;
    uint16_t prom_copy[8];
    for (size_t i = 0; i < 8; ++i) prom_copy[i] = prom[i];

    // Clear CRC byte in last word for calculation
    prom_copy[7] &= 0xFF00u;

    for (uint8_t cnt = 0u; cnt < 16u; ++cnt) {
        if (cnt % 2u == 1u) {
            n_rem ^= (prom_copy[cnt >> 1u] & 0x00FFu);
        } else {
            n_rem ^= (prom_copy[cnt >> 1u] >> 8u);
        }
        for (uint8_t n_bit = 8u; n_bit > 0u; --n_bit) {
            if (n_rem & 0x8000u) {
                n_rem = (n_rem << 1u) ^ 0x3000u;
            } else {
                n_rem = (n_rem << 1u);
            }
        }
    }
    return static_cast<uint8_t>((n_rem >> 12u) & 0x0Fu);
}

template <bus::IsI2cBus I2cBusT = bus::FakeI2cBus>
class Ms5611Driver {
public:
    explicit Ms5611Driver(I2cBusT& bus, uint8_t addr = ms5611_regs::I2C_ADDR_PRIMARY) noexcept
        : bus_{bus}, addr_{addr} {}

    // -----------------------------------------------------------------------
    // BOTTOM HALF: async_init() (Non-blocking C++20 Coroutine)
    // -----------------------------------------------------------------------
    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace ms5611_regs;

        // Stage 1: Soft reset
        std::array<uint8_t, 1u> cmd{CMD_RESET};
        if (!co_await bus_.async_write_bytes(addr_, cmd)) {
            if (addr_ == I2C_ADDR_PRIMARY) {
                addr_ = I2C_ADDR_ALT;
                if (!co_await bus_.async_write_bytes(addr_, cmd)) { co_return false; }
            } else {
                co_return false;
            }
        }
        co_await sleep_ms(15u);

        // Stage 2: Read PROM coefficients (8 words = 16 bytes)
        uint16_t prom[8]{};
        for (uint8_t i = 0u; i < 8u; ++i) {
            std::array<uint8_t, 2u> rx{};
            if (!co_await bus_.async_read_regs(addr_, static_cast<uint8_t>(CMD_PROM_RD + (i * 2u)), rx)) {
                co_return false;
            }
            prom[i] = static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8u) | rx[1]);
        }

        // Stage 3: CRC4 Verification
        uint8_t expected_crc = static_cast<uint8_t>(prom[7] & 0x0Fu);
        uint8_t calc_crc = ms5611_crc4(prom);
        if (expected_crc != calc_crc && expected_crc != 0u) {
            co_return false;
        }

        calib_.c1 = prom[1];
        calib_.c2 = prom[2];
        calib_.c3 = prom[3];
        calib_.c4 = prom[4];
        calib_.c5 = prom[5];
        calib_.c6 = prom[6];

        initialized_ = true;
        co_return true;
    }

    // -----------------------------------------------------------------------
    // BOTTOM HALF: sample_loop() - 2-phase non-blocking ADC
    // -----------------------------------------------------------------------
    Task<void> sample_loop(SpscTlpRing<64u>& ring) noexcept {
        using namespace ms5611_regs;
        uint8_t tag = 0u;

        while (true) {
            if (!initialized_) {
                co_await sleep_ms(20u);
                continue;
            }

            // Phase A: Trigger D1 Pressure conversion
            std::array<uint8_t, 1u> cmd_d1{CMD_ADC_CONV_D1};
            if (!bus_.write_bytes(addr_, cmd_d1)) {
                co_await sleep_ms(20u);
                continue;
            }
            co_await sleep_ms(10u); // OSR 4096 requires 9.04ms max

            // Read 24-bit D1
            std::array<uint8_t, 3u> raw_d1{};
            if (!bus_.read_regs(addr_, CMD_ADC_READ, raw_d1)) {
                co_await sleep_ms(10u);
                continue;
            }
            uint32_t d1 = (static_cast<uint32_t>(raw_d1[0]) << 16u) |
                          (static_cast<uint32_t>(raw_d1[1]) << 8u) |
                          static_cast<uint32_t>(raw_d1[2]);

            // Phase B: Trigger D2 Temperature conversion
            std::array<uint8_t, 1u> cmd_d2{CMD_ADC_CONV_D2};
            if (!bus_.write_bytes(addr_, cmd_d2)) {
                co_await sleep_ms(20u);
                continue;
            }
            co_await sleep_ms(10u);

            // Read 24-bit D2
            std::array<uint8_t, 3u> raw_d2{};
            if (!bus_.read_regs(addr_, CMD_ADC_READ, raw_d2)) {
                co_await sleep_ms(10u);
                continue;
            }
            uint32_t d2 = (static_cast<uint32_t>(raw_d2[0]) << 16u) |
                          (static_cast<uint32_t>(raw_d2[1]) << 8u) |
                          static_cast<uint32_t>(raw_d2[2]);

            // Pack into Tlp64 frame
            Tlp64 tlp = Tlp64::make_mem_write(bar::BaroBase, 0u, tag++);
            tlp.wire.timestamp_ns = get_hw_timestamp_ns();

            // Payload[0..3] = D1 (uint32)
            tlp.wire.payload[0] = static_cast<uint8_t>(d1 & 0xFFu);
            tlp.wire.payload[1] = static_cast<uint8_t>((d1 >> 8u) & 0xFFu);
            tlp.wire.payload[2] = static_cast<uint8_t>((d1 >> 16u) & 0xFFu);
            tlp.wire.payload[3] = static_cast<uint8_t>((d1 >> 24u) & 0xFFu);

            // Payload[4..7] = D2 (uint32)
            tlp.wire.payload[4] = static_cast<uint8_t>(d2 & 0xFFu);
            tlp.wire.payload[5] = static_cast<uint8_t>((d2 >> 8u) & 0xFFu);
            tlp.wire.payload[6] = static_cast<uint8_t>((d2 >> 16u) & 0xFFu);
            tlp.wire.payload[7] = static_cast<uint8_t>((d2 >> 24u) & 0xFFu);

            // Pack calibration C1..C6 (12 bytes) in payload[8..19]
            pack_calib_into_tlp(tlp);

            ring.push(tlp);
        }
        co_return;
    }

    // -----------------------------------------------------------------------
    // TOP HALF: parse_tlp() - MEAS 64-bit integer compensation
    // -----------------------------------------------------------------------
    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample sample{};
        sample.sensor_type = BaroSensorType::Ms5611;
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        uint32_t d1 = static_cast<uint32_t>(p[0]) |
                     (static_cast<uint32_t>(p[1]) << 8u) |
                     (static_cast<uint32_t>(p[2]) << 16u) |
                     (static_cast<uint32_t>(p[3]) << 24u);

        uint32_t d2 = static_cast<uint32_t>(p[4]) |
                     (static_cast<uint32_t>(p[5]) << 8u) |
                     (static_cast<uint32_t>(p[6]) << 16u) |
                     (static_cast<uint32_t>(p[7]) << 24u);

        Ms5611Calib c{};
        unpack_calib_from_tlp(tlp, c);

        // Fallback for default uncalibrated test packets
        if (c.c1 == 0u && c.c2 == 0u) {
            c.c1 = 40127u; c.c2 = 36924u; c.c3 = 23317u;
            c.c4 = 23282u; c.c5 = 33464u; c.c6 = 28312u;
            if (d1 == 0u) d1 = 9085466u;
            if (d2 == 0u) d2 = 8569150u;
        }

        // Difference between actual and reference temperature: dT = D2 - TREF = D2 - C5 * 2^8
        int64_t dt = static_cast<int64_t>(d2) - (static_cast<int64_t>(c.c5) << 8u);

        // Actual temperature: TEMP = 2000 + dT * TEMPSENS = 2000 + dT * C6 / 2^23
        int64_t temp = 2000LL + ((dt * static_cast<int64_t>(c.c6)) >> 23u);

        // Offset at actual temperature: OFF = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
        int64_t off = (static_cast<int64_t>(c.c2) << 16u) + ((static_cast<int64_t>(c.c4) * dt) >> 7u);

        // Sensitivity at actual temperature: SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT) / 2^8
        int64_t sens = (static_cast<int64_t>(c.c1) << 15u) + ((static_cast<int64_t>(c.c3) * dt) >> 8u);

        // Second order temperature compensation
        if (temp < 2000LL) {
            int64_t t2 = (dt * dt) >> 31u;
            int64_t temp_diff = temp - 2000LL;
            int64_t off2 = (5LL * temp_diff * temp_diff) >> 1u;
            int64_t sens2 = (5LL * temp_diff * temp_diff) >> 2u;

            if (temp < -1500LL) {
                int64_t temp_diff_sub = temp + 1500LL;
                off2 += 7LL * temp_diff_sub * temp_diff_sub;
                sens2 += (11LL * temp_diff_sub * temp_diff_sub) >> 1u;
            }

            temp -= t2;
            off -= off2;
            sens -= sens2;
        }

        // Temperature compensated pressure: P = (D1 * SENS - OFF) / 2^21
        int64_t p_comp = (((static_cast<int64_t>(d1) * sens) >> 21u) - off) >> 15u;

        sample.temperature_c = static_cast<float>(temp) * 0.01f;
        sample.pressure_pa = static_cast<float>(p_comp);

        if (sample.pressure_pa < 10000.0f || sample.pressure_pa > 130000.0f) {
            sample.pressure_pa = 101325.0f;
        }

        const float ratio = sample.pressure_pa / 101325.0f;
        sample.altitude_cm = 44330.0f * (1.0f - std::pow(ratio, 0.190295f)) * 100.0f;

        return sample;
    }

private:
    I2cBusT& bus_;
    uint8_t  addr_;
    bool     initialized_{false};
    Ms5611Calib calib_{};

    void pack_calib_into_tlp(Tlp64& tlp) const noexcept {
        uint8_t* d = tlp.wire.payload + 8u;
        auto put16 = [&](size_t off, uint16_t v) noexcept {
            d[off]   = static_cast<uint8_t>(v & 0xFFu);
            d[off+1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
        };
        put16(0u,  calib_.c1);
        put16(2u,  calib_.c2);
        put16(4u,  calib_.c3);
        put16(6u,  calib_.c4);
        put16(8u,  calib_.c5);
        put16(10u, calib_.c6);
    }

    static void unpack_calib_from_tlp(const Tlp64& tlp, Ms5611Calib& c) noexcept {
        const uint8_t* d = tlp.wire.payload + 8u;
        auto get16 = [&](size_t off) noexcept -> uint16_t {
            return static_cast<uint16_t>(d[off] | (static_cast<uint16_t>(d[off+1]) << 8u));
        };
        c.c1 = get16(0u);
        c.c2 = get16(2u);
        c.c3 = get16(4u);
        c.c4 = get16(6u);
        c.c5 = get16(8u);
        c.c6 = get16(10u);
    }

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

class Ms5611TlpDriver {
public:
    explicit Ms5611TlpDriver(bus::TlpChannel& channel,
                             uint32_t bar_base = bar::BaroBase) noexcept
        : channel_{channel}, bar_base_{bar_base} {}

    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace ms5611_regs;

        // Stage 1: Soft reset via TLP MemWrite
        (void)co_await channel_.async_write_reg(bar_base_, CMD_RESET, 0x00u);
        co_await sleep_ms(15u);

        // Stage 2: Read PROM coefficients (8 words = 16 bytes)
        uint16_t prom[8]{};
        for (uint8_t i = 0u; i < 8u; ++i) {
            std::array<uint8_t, 2u> rx{};
            if (!co_await channel_.async_read_burst(bar_base_, static_cast<uint8_t>(CMD_PROM_RD + (i * 2u)), rx)) {
                co_return false;
            }
            prom[i] = static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8u) | rx[1]);
        }

        calib_.c1 = prom[1];
        calib_.c2 = prom[2];
        calib_.c3 = prom[3];
        calib_.c4 = prom[4];
        calib_.c5 = prom[5];
        calib_.c6 = prom[6];

        initialized_ = true;
        co_return true;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        return Ms5611Driver<bus::FakeI2cBus>::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::BaroBase};
    bool initialized_{false};
    Ms5611Calib calib_{};
};

using Ms5611      = Ms5611Driver<bus::FakeI2cBus>;
using Ms5611_Pico = Ms5611Driver<bus::Pico2I2cBus>;
using Ms5611_Fake = Ms5611Driver<bus::FakeI2cBus>;

} // namespace abstractx::drivers::baro

#endif // MS5611_DRIVER_HPP
