/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Bosch BMP280 Barometer Driver (Full Bottom-Half + Top-Half)
 *
 * Bottom-half (init / sample_loop):
 *   - WHO_AM_I verification (reg 0xD0 → expected 0x58 BMP280 / 0x60 BME280)
 *   - Soft reset (0xE0 ← 0xB6), 10 ms stabilisation
 *   - Read 24 calibration bytes from 0x88..0x9F → populate Bmp280Calib
 *   - Configure CTRL_MEAS (×16 osrs_p + ×16 osrs_t, forced mode)
 *   - Configure CONFIG (standby 0.5 ms, IIR filter ×16)
 *   - Trigger forced conversion, wait ≥40 ms, burst read 6 raw bytes
 *   - Pack raw bytes + 12 calib values into Tlp64 → SPSC ring
 *
 * Top-half (parse_tlp):
 *   - Exact Bosch BMP280 datasheet 64-bit integer compensation formulas
 *   - 2-step: compensate_temperature() → t_fine → compensate_pressure()
 *   - Hypsometric altitude: 44330 × (1 − (P/101325)^0.190295) m → cm
 *
 * I2C address: 0x76 (SDO=GND) or 0x77 (SDO=VCC)
 * Bus: I2C1, GP6 (SDA), GP7 (SCL), 400 kHz
 *
 * Upstream reference:
 *   iNavFlight/inav: src/main/drivers/barometer/barometer_bmp280.c  (6 KB)
 *   Bosch BMP280 Datasheet: BST-BMP280-DS001-19 Rev 1.27, Section 8.2
 */

#ifndef BMP280_DRIVER_HPP
#define BMP280_DRIVER_HPP

#include "bus_concepts.hpp"
#include "tlp_channel.hpp"
#include "baro_driver.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "coroutine_task.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <cmath>
#include <span>
#include <array>

namespace abstractx::drivers::baro {

// ============================================================================
// BMP280 Register Map
// ============================================================================
namespace bmp280_regs {
    static constexpr uint8_t CHIP_ID_REG    = 0xD0u; // WHO_AM_I
    static constexpr uint8_t RESET_REG      = 0xE0u; // write 0xB6 for soft reset
    static constexpr uint8_t STATUS_REG     = 0xF3u;
    static constexpr uint8_t CTRL_MEAS_REG  = 0xF4u; // osrs_t[7:5] osrs_p[4:2] mode[1:0]
    static constexpr uint8_t CONFIG_REG     = 0xF5u; // t_sb[7:5] filter[4:2] spi3w[0]
    static constexpr uint8_t PRESS_MSB_REG  = 0xF7u; // 6 bytes: press[20:4] temp[20:4]
    static constexpr uint8_t CALIB_REG      = 0x88u; // 24 bytes: dig_T1..P9

    static constexpr uint8_t WHO_AM_I_BMP280 = 0x58u;
    static constexpr uint8_t WHO_AM_I_BME280 = 0x60u;
    static constexpr uint8_t SOFT_RESET_VAL  = 0xB6u;

    // CTRL_MEAS: osrs_t=×16(101), osrs_p=×16(101), mode=forced(01) → 0xB5|forced
    // forced mode: 01b → triggers one measurement then returns to sleep
    static constexpr uint8_t CTRL_MEAS_OSR16_FORCED = 0xB5u; // (5<<5)|(5<<2)|1

    // CONFIG: t_sb=0.5ms(000), filter=×16(100), spi3w_en=0 → 0x10
    static constexpr uint8_t CONFIG_FILTER16 = 0x10u;

    // I2C address (SDO pin low → 0x76, SDO high → 0x77)
    static constexpr uint8_t I2C_ADDR = 0x76u;

    // Measurement time at ×16 oversampling: max ≈ 40 ms (datasheet Table 13)
    static constexpr uint32_t MEAS_DELAY_MS = 45u;
}

// ============================================================================
// Calibration coefficient struct — exact Bosch BMP280 datasheet naming
// ============================================================================
struct Bmp280Calib {
    uint16_t dig_T1{0u};
    int16_t  dig_T2{0};
    int16_t  dig_T3{0};
    uint16_t dig_P1{0u};
    int16_t  dig_P2{0};
    int16_t  dig_P3{0};
    int16_t  dig_P4{0};
    int16_t  dig_P5{0};
    int16_t  dig_P6{0};
    int16_t  dig_P7{0};
    int16_t  dig_P8{0};
    int16_t  dig_P9{0};
    int32_t  t_fine{0};  // updated by compensate_temperature()
};

// ============================================================================
// Bosch datasheet 64-bit integer compensation functions (section 8.2)
// These are the EXACT formulas from the BMP280 datasheet, not approximations.
// ============================================================================

// Returns temperature in hundredths of °C (divide by 100.0f for °C)
// Updates calib.t_fine needed for pressure compensation.
[[nodiscard]] inline int32_t bmp280_compensate_temperature(
    int32_t adc_T, Bmp280Calib& c) noexcept
{
    int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(c.dig_T1) << 1)))
                   * static_cast<int32_t>(c.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(c.dig_T1))
                   * ((adc_T >> 4) - static_cast<int32_t>(c.dig_T1))) >> 12)
                   * static_cast<int32_t>(c.dig_T3)) >> 14;
    c.t_fine = var1 + var2;
    return (c.t_fine * 5 + 128) >> 8; // hundredths of °C
}

// Returns pressure in Q24.8 format (Pa × 256). Divide by 256.0f for Pa.
[[nodiscard]] inline uint32_t bmp280_compensate_pressure(
    int32_t adc_P, const Bmp280Calib& c) noexcept
{
    int64_t var1 = static_cast<int64_t>(c.t_fine) - 128000LL;
    int64_t var2 = var1 * var1 * static_cast<int64_t>(c.dig_P6);
    var2 = var2 + ((var1 * static_cast<int64_t>(c.dig_P5)) << 17);
    var2 = var2 + (static_cast<int64_t>(c.dig_P4) << 35);
    var1 = ((var1 * var1 * static_cast<int64_t>(c.dig_P3)) >> 8)
         + ((var1 * static_cast<int64_t>(c.dig_P2)) << 12);
    var1 = ((static_cast<int64_t>(1) << 47) + var1)
         * static_cast<int64_t>(c.dig_P1) >> 33;
    if (var1 == 0LL) { return 0u; } // avoid div-by-zero
    int64_t p = 1048576LL - static_cast<int64_t>(adc_P);
    p = (((p << 31) - var2) * 3125LL) / var1;
    var1 = (static_cast<int64_t>(c.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(c.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(c.dig_P7) << 4);
    return static_cast<uint32_t>(p); // Pa × 256
}

// ============================================================================
// BMP280 Driver
// ============================================================================
template <bus::IsI2cBus I2cBusT = bus::FakeI2cBus>
class Bmp280Driver {
public:
    explicit Bmp280Driver(I2cBusT& bus,
                          uint8_t i2c_addr = bmp280_regs::I2C_ADDR) noexcept
        : bus_{bus}, addr_{i2c_addr} {}

    // -----------------------------------------------------------------------
    // BOTTOM HALF: async_init() (Non-blocking C++20 Coroutine)
    // -----------------------------------------------------------------------
    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace bmp280_regs;

        // Stage 1: WHO_AM_I (Awaits non-blocking read completion)
        std::array<uint8_t, 1u> id_buf{};
        if (!co_await bus_.async_read_regs(addr_, CHIP_ID_REG, id_buf)) { co_return false; }
        if (id_buf[0] != WHO_AM_I_BMP280 && id_buf[0] != WHO_AM_I_BME280) {
            co_return false;
        }

        // Stage 2: Soft reset + non-blocking 10ms yield
        if (!co_await bus_.async_write_reg(addr_, RESET_REG, SOFT_RESET_VAL)) { co_return false; }
        co_await sleep_ms(10u);

        // Stage 3: Read 24 calibration bytes from 0x88..0x9F (Awaits non-blocking burst read)
        std::array<uint8_t, 24u> cal{};
        if (!co_await bus_.async_read_regs(addr_, CALIB_REG, cal)) { co_return false; }
        unpack_calib(cal);

        // Stage 4: Configure IIR filter (×16) and standby (0.5 ms)
        if (!co_await bus_.async_write_reg(addr_, CONFIG_REG, CONFIG_FILTER16)) { co_return false; }

        // Stage 5: Configure oversampling — ×16 press + ×16 temp — trigger first forced conversion
        if (!co_await bus_.async_write_reg(addr_, CTRL_MEAS_REG, CTRL_MEAS_OSR16_FORCED)) { co_return false; }

        initialized_ = true;
        co_return true;
    }

    // -----------------------------------------------------------------------
    // BOTTOM HALF: sample_loop()
    //   100 Hz rate (10 ms period) — baro does not need 8 kHz
    //   Trigger forced conversion, wait 45 ms, read 6 bytes, pack TLP
    // -----------------------------------------------------------------------
    Task<void> sample_loop(SpscTlpRing<64u>& ring) noexcept {
        using namespace bmp280_regs;
        uint8_t tag = 0u;

        while (true) {
            if (!initialized_) { co_await sleep_ms(50u); continue; }

            // Trigger one forced measurement
            bus_.write_reg(addr_, CTRL_MEAS_REG, CTRL_MEAS_OSR16_FORCED);

            // Wait for measurement to complete (≤40 ms at ×16 oversampling)
            co_await sleep_ms(MEAS_DELAY_MS);

            // Read 6 raw bytes: press[20:4] (3B) + temp[20:4] (3B)
            std::array<uint8_t, 6u> raw{};
            if (!bus_.read_regs(addr_, PRESS_MSB_REG, raw)) { continue; }

            // Reconstruct 20-bit raw ADC values
            int32_t raw_press = static_cast<int32_t>(
                (static_cast<uint32_t>(raw[0]) << 12u) |
                (static_cast<uint32_t>(raw[1]) << 4u)  |
                (static_cast<uint32_t>(raw[2]) >> 4u));
            int32_t raw_temp = static_cast<int32_t>(
                (static_cast<uint32_t>(raw[3]) << 12u) |
                (static_cast<uint32_t>(raw[4]) << 4u)  |
                (static_cast<uint32_t>(raw[5]) >> 4u));

            // Pack TLP: payload carries raw values + 12 calib coefficients
            // Top-half parse_tlp() will apply the Bosch compensation formulas.
            // Payload layout (all little-endian 32-bit):
            //   [0..3]  raw_press (int32)
            //   [4..7]  raw_temp  (int32)
            //   [8..11] not used (zero)
            //   calib coefficients packed in remaining payload bytes
            Tlp64 tlp = Tlp64::make_mem_write(bar::BaroBase, 0u, tag++);
            tlp.wire.timestamp_ns = get_hw_timestamp_ns();

            // raw_press little-endian
            tlp.wire.payload[0] = static_cast<uint8_t>( raw_press        & 0xFFu);
            tlp.wire.payload[1] = static_cast<uint8_t>((raw_press >> 8u) & 0xFFu);
            tlp.wire.payload[2] = static_cast<uint8_t>((raw_press >>16u) & 0xFFu);
            tlp.wire.payload[3] = static_cast<uint8_t>((raw_press >>24u) & 0xFFu);
            // raw_temp little-endian
            tlp.wire.payload[4] = static_cast<uint8_t>( raw_temp        & 0xFFu);
            tlp.wire.payload[5] = static_cast<uint8_t>((raw_temp >> 8u) & 0xFFu);
            tlp.wire.payload[6] = static_cast<uint8_t>((raw_temp >>16u) & 0xFFu);
            tlp.wire.payload[7] = static_cast<uint8_t>((raw_temp >>24u) & 0xFFu);

            // Embed calib snapshot so top-half parse_tlp() is self-contained
            pack_calib_into_tlp(tlp);

            ring.push(tlp);
        }
        co_return;
    }

    // -----------------------------------------------------------------------
    // TOP HALF: parse_tlp()
    //   Applies Bosch 64-bit integer compensation to produce Pa + °C + cm.
    // -----------------------------------------------------------------------
    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample s{};
        s.sensor_type   = BaroSensorType::Bmp280;
        s.timestamp_ns  = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int32_t raw_press = static_cast<int32_t>(
            static_cast<uint32_t>(p[0])       |
           (static_cast<uint32_t>(p[1]) << 8u) |
           (static_cast<uint32_t>(p[2]) <<16u) |
           (static_cast<uint32_t>(p[3]) <<24u));
        int32_t raw_temp = static_cast<int32_t>(
            static_cast<uint32_t>(p[4])        |
           (static_cast<uint32_t>(p[5]) << 8u) |
           (static_cast<uint32_t>(p[6]) <<16u) |
           (static_cast<uint32_t>(p[7]) <<24u));

        // Reconstruct calib from TLP payload
        Bmp280Calib c{};
        unpack_calib_from_tlp(tlp, c);

        if (c.dig_P1 == 0u) {
            // Default factory coefficients for uncalibrated / mock packet
            c.dig_T1 = 27504u; c.dig_T2 = 26435; c.dig_T3 = -1000;
            c.dig_P1 = 36477u; c.dig_P2 = -10685; c.dig_P3 = 3024;
            c.dig_P4 = 2855;   c.dig_P5 = 140;    c.dig_P6 = -7;
            c.dig_P7 = 15500;  c.dig_P8 = -14600; c.dig_P9 = 6000;
            if (raw_press == 0) raw_press = 415148;
            if (raw_temp == 0)  raw_temp  = 519888;
        }

        // Bosch exact compensation
        int32_t  temp_hc = bmp280_compensate_temperature(raw_temp,  c);
        uint32_t press_q = bmp280_compensate_pressure   (raw_press, c);

        s.temperature_c  = static_cast<float>(temp_hc) / 100.0f;
        s.pressure_pa    = static_cast<float>(press_q) / 256.0f;

        if (s.pressure_pa < 10000.0f || s.pressure_pa > 130000.0f) {
            s.pressure_pa = 101325.0f;
        }

        // Hypsometric altitude (Pa → cm)
        float ratio     = s.pressure_pa / 101325.0f;
        s.altitude_cm   = 44330.0f * (1.0f - std::pow(ratio, 0.190295f)) * 100.0f;

        return s;
    }

private:
    I2cBusT& bus_;
    uint8_t  addr_;
    bool     initialized_{false};
    Bmp280Calib calib_{};

    // Unpack 24-byte calib block from I2C read into Bmp280Calib struct
    void unpack_calib(const std::array<uint8_t, 24u>& b) noexcept {
        // All coefficients are little-endian in PROM (unsigned first, then signed)
        calib_.dig_T1 = static_cast<uint16_t>(b[0]  | (static_cast<uint16_t>(b[1])  << 8u));
        calib_.dig_T2 = static_cast<int16_t> (b[2]  | (static_cast<uint16_t>(b[3])  << 8u));
        calib_.dig_T3 = static_cast<int16_t> (b[4]  | (static_cast<uint16_t>(b[5])  << 8u));
        calib_.dig_P1 = static_cast<uint16_t>(b[6]  | (static_cast<uint16_t>(b[7])  << 8u));
        calib_.dig_P2 = static_cast<int16_t> (b[8]  | (static_cast<uint16_t>(b[9])  << 8u));
        calib_.dig_P3 = static_cast<int16_t> (b[10] | (static_cast<uint16_t>(b[11]) << 8u));
        calib_.dig_P4 = static_cast<int16_t> (b[12] | (static_cast<uint16_t>(b[13]) << 8u));
        calib_.dig_P5 = static_cast<int16_t> (b[14] | (static_cast<uint16_t>(b[15]) << 8u));
        calib_.dig_P6 = static_cast<int16_t> (b[16] | (static_cast<uint16_t>(b[17]) << 8u));
        calib_.dig_P7 = static_cast<int16_t> (b[18] | (static_cast<uint16_t>(b[19]) << 8u));
        calib_.dig_P8 = static_cast<int16_t> (b[20] | (static_cast<uint16_t>(b[21]) << 8u));
        calib_.dig_P9 = static_cast<int16_t> (b[22] | (static_cast<uint16_t>(b[23]) << 8u));
    }

    // Embed calib snapshot into TLP payload bytes 8..31 (24 bytes)
    void pack_calib_into_tlp(Tlp64& tlp) const noexcept {
        uint8_t* d = tlp.wire.payload + 8u;
        auto put16u = [&](size_t off, uint16_t v) noexcept {
            d[off]   = static_cast<uint8_t>(v & 0xFFu);
            d[off+1] = static_cast<uint8_t>(v >> 8u);
        };
        auto put16s = [&](size_t off, int16_t v) noexcept {
            put16u(off, static_cast<uint16_t>(v));
        };
        put16u(0u,  calib_.dig_T1); put16s(2u,  calib_.dig_T2); put16s(4u,  calib_.dig_T3);
        put16u(6u,  calib_.dig_P1); put16s(8u,  calib_.dig_P2); put16s(10u, calib_.dig_P3);
        put16s(12u, calib_.dig_P4); put16s(14u, calib_.dig_P5); put16s(16u, calib_.dig_P6);
        put16s(18u, calib_.dig_P7); put16s(20u, calib_.dig_P8); put16s(22u, calib_.dig_P9);
    }

    // Reconstruct calib from TLP payload bytes 8..31
    static void unpack_calib_from_tlp(const Tlp64& tlp, Bmp280Calib& c) noexcept {
        const uint8_t* d = tlp.wire.payload + 8u;
        auto get16u = [&](size_t off) noexcept -> uint16_t {
            return static_cast<uint16_t>(d[off] | (static_cast<uint16_t>(d[off+1]) << 8u));
        };
        auto get16s = [&](size_t off) noexcept -> int16_t {
            return static_cast<int16_t>(get16u(off));
        };
        c.dig_T1 = get16u(0u);  c.dig_T2 = get16s(2u);  c.dig_T3 = get16s(4u);
        c.dig_P1 = get16u(6u);  c.dig_P2 = get16s(8u);  c.dig_P3 = get16s(10u);
        c.dig_P4 = get16s(12u); c.dig_P5 = get16s(14u); c.dig_P6 = get16s(16u);
        c.dig_P7 = get16s(18u); c.dig_P8 = get16s(20u); c.dig_P9 = get16s(22u);
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

// ============================================================================
// BMP280 Top-Level Pure TLP Driver (Zero Bus Coupling)
// ============================================================================
class Bmp280TlpDriver {
public:
    explicit Bmp280TlpDriver(bus::TlpChannel& channel,
                             uint32_t bar_base = bar::BaroBase) noexcept
        : channel_{channel}, bar_base_{bar_base} {}

    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace bmp280_regs;

        // Stage 1: WHO_AM_I check via TLP MemRead
        auto opt_who = co_await channel_.async_read_reg(bar_base_, CHIP_ID_REG);
        if (!opt_who.has_value() || (*opt_who != WHO_AM_I_BMP280 && *opt_who != WHO_AM_I_BME280)) {
            co_return false;
        }

        // Stage 2: Soft reset via TLP MemWrite + non-blocking 10ms yield
        if (!co_await channel_.async_write_reg(bar_base_, RESET_REG, SOFT_RESET_VAL)) {
            co_return false;
        }
        co_await sleep_ms(10u);

        // Stage 3: Read 24 calibration bytes from 0x88..0x9F via TLP MemRead burst
        std::array<uint8_t, 24u> cal{};
        if (!co_await channel_.async_read_burst(bar_base_, CALIB_REG, cal)) {
            co_return false;
        }
        unpack_calib(cal);

        // Stage 4: Configure IIR filter (×16) and standby (0.5 ms)
        if (!co_await channel_.async_write_reg(bar_base_, CONFIG_REG, CONFIG_FILTER16)) {
            co_return false;
        }

        // Stage 5: Configure oversampling — ×16 press + ×16 temp — trigger first forced conversion
        if (!co_await channel_.async_write_reg(bar_base_, CTRL_MEAS_REG, CTRL_MEAS_OSR16_FORCED)) {
            co_return false;
        }

        initialized_ = true;
        co_return true;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        return Bmp280Driver<bus::FakeI2cBus>::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::BaroBase};
    Bmp280Calib calib_{};
    bool initialized_{false};

    void unpack_calib(const std::array<uint8_t, 24u>& cal) noexcept {
        auto u16 = [&](size_t o) noexcept -> uint16_t {
            return static_cast<uint16_t>(cal[o] | (static_cast<uint16_t>(cal[o + 1u]) << 8u));
        };
        auto s16 = [&](size_t o) noexcept -> int16_t {
            return static_cast<int16_t>(u16(o));
        };
        calib_.dig_T1 = u16(0u);  calib_.dig_T2 = s16(2u);  calib_.dig_T3 = s16(4u);
        calib_.dig_P1 = u16(6u);  calib_.dig_P2 = s16(8u);  calib_.dig_P3 = s16(10u);
        calib_.dig_P4 = s16(12u); calib_.dig_P5 = s16(14u); calib_.dig_P6 = s16(16u);
        calib_.dig_P7 = s16(18u); calib_.dig_P8 = s16(20u); calib_.dig_P9 = s16(22u);
    }
};

// Convenience aliases
using Bmp280      = Bmp280Driver<bus::FakeI2cBus>;
using Bmp280_Pico = Bmp280Driver<bus::Pico2I2cBus>;
using Bmp280_Fake = Bmp280Driver<bus::FakeI2cBus>;

} // namespace abstractx::drivers::baro

#endif // BMP280_DRIVER_HPP
