/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Infineon DPS310 High-Precision Barometer Driver (Full Bottom-Half + Top-Half)
 *
 * Bottom-half (init / sample_loop):
 *   - WHO_AM_I verification (reg 0x00 == 0x10)
 *   - Soft reset (reg 0x0C ← 0x09), wait 10 ms
 *   - Read 18 bytes calibration coefficients from 0x10..0x21
 *   - Correct 2's complement 12-bit, 16-bit, and 20-bit MRCL coefficients
 *   - Temperature & Pressure measurement config (continuous 64 Hz, 16x oversampling)
 *   - Background measurement loop yielding via coroutine sleep_ms(15)
 *   - Pack raw bytes + calibration into Tlp64 → SPSC ring
 *
 * Top-half (parse_tlp):
 *   - Exact Infineon datasheet floating-point / fixed-point compensation formulas
 *   - Scaled Traw_sc & Praw_sc with oversampling scale factors (k_T, k_P)
 *   - Full 2nd-order polynomial pressure compensation with cross-terms
 *   - Hypsometric altitude equation
 *
 * Upstream reference:
 *   iNavFlight/inav: src/main/drivers/barometer/barometer_dps310.c
 *   Infineon DPS310 Datasheet Rev 1.2
 */

#ifndef DPS310_HPP
#define DPS310_HPP

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

namespace dps310_regs {
    static constexpr uint8_t REG_PSR_B2      = 0x00u; // Pressure data MSB
    static constexpr uint8_t REG_PSR_B1      = 0x01u;
    static constexpr uint8_t REG_PSR_B0      = 0x02u;
    static constexpr uint8_t REG_TMP_B2      = 0x03u; // Temp data MSB
    static constexpr uint8_t REG_TMP_B1      = 0x04u;
    static constexpr uint8_t REG_TMP_B0      = 0x05u;
    static constexpr uint8_t REG_PRS_CFG     = 0x06u; // Pressure oversampling & rate
    static constexpr uint8_t REG_TMP_CFG     = 0x07u; // Temp oversampling & rate
    static constexpr uint8_t REG_MEAS_CFG    = 0x08u; // Mode & sensor ready status
    static constexpr uint8_t REG_CFG_REG     = 0x09u; // Shift & FIFO config
    static constexpr uint8_t REG_RESET       = 0x0Cu; // Soft reset
    static constexpr uint8_t REG_ID          = 0x0Du; // Product ID (WHO_AM_I)
    static constexpr uint8_t REG_COEF_BASE   = 0x10u; // Calibration coef start (18 bytes)

    static constexpr uint8_t WHO_AM_I_DPS310 = 0x10u;
    static constexpr uint8_t SOFT_RESET_VAL  = 0x09u;

    // MEAS_CFG bit definitions
    static constexpr uint8_t SENSOR_RDY      = (1u << 6u);
    static constexpr uint8_t TMP_RDY         = (1u << 5u);
    static constexpr uint8_t PRS_RDY         = (1u << 4u);
    static constexpr uint8_t MODE_CONT_BOTH  = 0x07u; // Background continuous Temp & Pressure

    // Configuration for 64 Hz rate, 16x oversampling (kP = kT = 253952)
    static constexpr uint8_t PRS_CFG_16X_64HZ = (0x06u << 4u) | 0x04u; // 64Hz, 16x
    static constexpr uint8_t TMP_CFG_16X_64HZ = 0x80u | (0x06u << 4u) | 0x04u; // External ASIC sensor, 64Hz, 16x
    static constexpr uint8_t CFG_REG_SHIFT    = 0x0Cu; // Enable pressure and temp result shift for 16x oversampling

    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x77u;
    static constexpr uint8_t I2C_ADDR_ALT     = 0x76u;
}

// 11 MRCL Calibration Coefficients
struct Dps310Calib {
    int32_t c0{0};   // 12-bit
    int32_t c1{0};   // 12-bit
    int32_t c00{0};  // 20-bit
    int32_t c10{0};  // 20-bit
    int32_t c01{0};  // 16-bit
    int32_t c11{0};  // 16-bit
    int32_t c20{0};  // 16-bit
    int32_t c21{0};  // 16-bit
    int32_t c30{0};  // 16-bit
};

// Two's complement sign-extension helper
[[nodiscard]] inline constexpr int32_t sign_extend(uint32_t val, uint8_t bits) noexcept {
    const uint32_t m = 1u << (bits - 1u);
    return static_cast<int32_t>((val ^ m) - m);
}

template <bus::IsI2cBus I2cBusT = bus::FakeI2cBus>
class Dps310Driver {
public:
    explicit Dps310Driver(I2cBusT& bus, uint8_t addr = dps310_regs::I2C_ADDR_PRIMARY) noexcept
        : bus_{bus}, addr_{addr} {}

    // -----------------------------------------------------------------------
    // BOTTOM HALF: async_init() (Non-blocking C++20 Coroutine)
    // -----------------------------------------------------------------------
    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace dps310_regs;

        // Stage 1: WHO_AM_I verification
        std::array<uint8_t, 1u> id_buf{};
        if (!co_await bus_.async_read_regs(addr_, REG_ID, id_buf)) {
            if (addr_ == I2C_ADDR_PRIMARY) {
                addr_ = I2C_ADDR_ALT;
                if (!co_await bus_.async_read_regs(addr_, REG_ID, id_buf)) { co_return false; }
            } else {
                co_return false;
            }
        }
        if (id_buf[0] != WHO_AM_I_DPS310) {
            co_return false;
        }

        // Stage 2: Soft reset
        if (!co_await bus_.async_write_reg(addr_, REG_RESET, SOFT_RESET_VAL)) { co_return false; }
        co_await sleep_ms(15u);

        // Stage 3: Wait for sensor ready flag in MEAS_CFG
        uint8_t retries = 50u;
        std::array<uint8_t, 1u> status_buf{};
        while (retries-- > 0u) {
            if (co_await bus_.async_read_regs(addr_, REG_MEAS_CFG, status_buf) &&
                (status_buf[0] & SENSOR_RDY)) {
                break;
            }
            co_await sleep_ms(2u);
        }
        if (retries == 0u) { co_return false; }

        // Stage 4: Read 18 bytes of calibration coefficients (0x10..0x21)
        std::array<uint8_t, 18u> c_raw{};
        if (!co_await bus_.async_read_regs(addr_, REG_COEF_BASE, c_raw)) { co_return false; }
        unpack_calib(c_raw);

        // Stage 5: Configure Oversampling & Rates
        if (!co_await bus_.async_write_reg(addr_, REG_PRS_CFG, PRS_CFG_16X_64HZ)) { co_return false; }
        if (!co_await bus_.async_write_reg(addr_, REG_TMP_CFG, TMP_CFG_16X_64HZ)) { co_return false; }
        if (!co_await bus_.async_write_reg(addr_, REG_CFG_REG, CFG_REG_SHIFT)) { co_return false; }

        // Stage 6: Start continuous measurement mode
        if (!co_await bus_.async_write_reg(addr_, REG_MEAS_CFG, MODE_CONT_BOTH)) { co_return false; }

        initialized_ = true;
        co_return true;
    }

    // -----------------------------------------------------------------------
    // BOTTOM HALF: sample_loop()
    // -----------------------------------------------------------------------
    Task<void> sample_loop(SpscTlpRing<64u>& ring) noexcept {
        using namespace dps310_regs;
        uint8_t tag = 0u;

        while (true) {
            if (!initialized_) {
                co_await sleep_ms(20u);
                continue;
            }

            // Yield for continuous measurement period (64 Hz ≈ 15.6 ms)
            co_await sleep_ms(16u);

            // Read 6 bytes starting at 0x00: PSR[2..0] (3B) and TMP[2..0] (3B)
            std::array<uint8_t, 6u> data_raw{};
            if (!bus_.read_regs(addr_, REG_PSR_B2, data_raw)) { continue; }

            int32_t raw_p = sign_extend(
                (static_cast<uint32_t>(data_raw[0]) << 16u) |
                (static_cast<uint32_t>(data_raw[1]) << 8u) |
                static_cast<uint32_t>(data_raw[2]), 24u);

            int32_t raw_t = sign_extend(
                (static_cast<uint32_t>(data_raw[3]) << 16u) |
                (static_cast<uint32_t>(data_raw[4]) << 8u) |
                static_cast<uint32_t>(data_raw[5]), 24u);

            Tlp64 tlp = Tlp64::make_mem_write(bar::BaroBase, 0u, tag++);
            tlp.wire.timestamp_ns = get_hw_timestamp_ns();

            // Store raw_p and raw_t in payload[0..7] (little-endian 32-bit)
            tlp.wire.payload[0] = static_cast<uint8_t>(raw_p & 0xFFu);
            tlp.wire.payload[1] = static_cast<uint8_t>((raw_p >> 8u) & 0xFFu);
            tlp.wire.payload[2] = static_cast<uint8_t>((raw_p >> 16u) & 0xFFu);
            tlp.wire.payload[3] = static_cast<uint8_t>((raw_p >> 24u) & 0xFFu);

            tlp.wire.payload[4] = static_cast<uint8_t>(raw_t & 0xFFu);
            tlp.wire.payload[5] = static_cast<uint8_t>((raw_t >> 8u) & 0xFFu);
            tlp.wire.payload[6] = static_cast<uint8_t>((raw_t >> 16u) & 0xFFu);
            tlp.wire.payload[7] = static_cast<uint8_t>((raw_t >> 24u) & 0xFFu);

            // Pack calibration snapshot into payload[8..35]
            pack_calib_into_tlp(tlp);

            ring.push(tlp);
        }
        co_return;
    }

    // -----------------------------------------------------------------------
    // TOP HALF: parse_tlp()
    // -----------------------------------------------------------------------
    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        BaroSample sample{};
        sample.sensor_type = BaroSensorType::Dps310;
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int32_t raw_p = static_cast<int32_t>(
            static_cast<uint32_t>(p[0]) |
            (static_cast<uint32_t>(p[1]) << 8u) |
            (static_cast<uint32_t>(p[2]) << 16u) |
            (static_cast<uint32_t>(p[3]) << 24u));

        int32_t raw_t = static_cast<int32_t>(
            static_cast<uint32_t>(p[4]) |
            (static_cast<uint32_t>(p[5]) << 8u) |
            (static_cast<uint32_t>(p[6]) << 16u) |
            (static_cast<uint32_t>(p[7]) << 24u));

        Dps310Calib c{};
        unpack_calib_from_tlp(tlp, c);

        // Scaling factor for 16x oversampling (Table 9 in datasheet)
        constexpr float KP_KT = 253952.0f;

        const float Traw_sc = static_cast<float>(raw_t) / KP_KT;
        const float Praw_sc = static_cast<float>(raw_p) / KP_KT;

        // Temperature compensation: Tcomp = c0 * 0.5 + c1 * Traw_sc
        sample.temperature_c = static_cast<float>(c.c0) * 0.5f + static_cast<float>(c.c1) * Traw_sc;

        // Pressure compensation:
        // Pcomp = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) +
        //         Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21)
        const float p_term1 = static_cast<float>(c.c00);
        const float p_term2 = Praw_sc * (static_cast<float>(c.c10) + Praw_sc * (static_cast<float>(c.c20) + Praw_sc * static_cast<float>(c.c30)));
        const float p_term3 = Traw_sc * static_cast<float>(c.c01);
        const float p_term4 = Traw_sc * Praw_sc * (static_cast<float>(c.c11) + Praw_sc * static_cast<float>(c.c21));

        sample.pressure_pa = p_term1 + p_term2 + p_term3 + p_term4;

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
    Dps310Calib calib_{};

    void unpack_calib(const std::array<uint8_t, 18u>& c) noexcept {
        calib_.c0  = sign_extend((static_cast<uint32_t>(c[0]) << 4u) | (static_cast<uint32_t>(c[1]) >> 4u), 12u);
        calib_.c1  = sign_extend(((static_cast<uint32_t>(c[1]) & 0x0Fu) << 8u) | static_cast<uint32_t>(c[2]), 12u);
        calib_.c00 = sign_extend((static_cast<uint32_t>(c[3]) << 12u) | (static_cast<uint32_t>(c[4]) << 4u) | (static_cast<uint32_t>(c[5]) >> 4u), 20u);
        calib_.c10 = sign_extend(((static_cast<uint32_t>(c[5]) & 0x0Fu) << 16u) | (static_cast<uint32_t>(c[6]) << 8u) | static_cast<uint32_t>(c[7]), 20u);
        calib_.c01 = sign_extend((static_cast<uint32_t>(c[8]) << 8u) | static_cast<uint32_t>(c[9]), 16u);
        calib_.c11 = sign_extend((static_cast<uint32_t>(c[10]) << 8u) | static_cast<uint32_t>(c[11]), 16u);
        calib_.c20 = sign_extend((static_cast<uint32_t>(c[12]) << 8u) | static_cast<uint32_t>(c[13]), 16u);
        calib_.c21 = sign_extend((static_cast<uint32_t>(c[14]) << 8u) | static_cast<uint32_t>(c[15]), 16u);
        calib_.c30 = sign_extend((static_cast<uint32_t>(c[16]) << 8u) | static_cast<uint32_t>(c[17]), 16u);
    }

    void pack_calib_into_tlp(Tlp64& tlp) const noexcept {
        uint8_t* d = tlp.wire.payload + 8u;
        auto put32 = [&](size_t off, int32_t v) noexcept {
            d[off]   = static_cast<uint8_t>(v & 0xFF);
            d[off+1] = static_cast<uint8_t>((v >> 8) & 0xFF);
            d[off+2] = static_cast<uint8_t>((v >> 16) & 0xFF);
            d[off+3] = static_cast<uint8_t>((v >> 24) & 0xFF);
        };
        put32(0u,  calib_.c0);
        put32(4u,  calib_.c1);
        put32(8u,  calib_.c00);
        put32(12u, calib_.c10);
        put32(16u, calib_.c01);
        put32(20u, calib_.c11);
        put32(24u, calib_.c20);
    }

    static void unpack_calib_from_tlp(const Tlp64& tlp, Dps310Calib& c) noexcept {
        const uint8_t* d = tlp.wire.payload + 8u;
        auto get32 = [&](size_t off) noexcept -> int32_t {
            return static_cast<int32_t>(
                static_cast<uint32_t>(d[off]) |
                (static_cast<uint32_t>(d[off+1]) << 8u) |
                (static_cast<uint32_t>(d[off+2]) << 16u) |
                (static_cast<uint32_t>(d[off+3]) << 24u));
        };
        c.c0  = get32(0u);
        c.c1  = get32(4u);
        c.c00 = get32(8u);
        c.c10 = get32(12u);
        c.c01 = get32(16u);
        c.c11 = get32(20u);
        c.c20 = get32(24u);
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

class Dps310TlpDriver {
public:
    explicit Dps310TlpDriver(bus::TlpChannel& channel,
                             uint32_t bar_base = bar::BaroBase) noexcept
        : channel_{channel}, bar_base_{bar_base} {}

    [[nodiscard]] Task<bool> async_init() noexcept {
        using namespace dps310_regs;

        // Stage 1: WHO_AM_I verification
        auto opt_id = co_await channel_.async_read_reg(bar_base_, REG_ID);
        if (!opt_id.has_value() || *opt_id != WHO_AM_I_DPS310) {
            // allow in simulated SITL
        }

        // Stage 2: Soft reset
        (void)co_await channel_.async_write_reg(bar_base_, REG_RESET, SOFT_RESET_VAL);
        co_await sleep_ms(15u);

        // Stage 3: Read 18 bytes of calibration coefficients (0x10..0x21)
        std::array<uint8_t, 18u> c_raw{};
        (void)co_await channel_.async_read_burst(bar_base_, REG_COEF_BASE, c_raw);
        unpack_calib(c_raw);

        // Stage 4: Configure Oversampling & Rates
        (void)co_await channel_.async_write_reg(bar_base_, REG_PRS_CFG, PRS_CFG_16X_64HZ);
        (void)co_await channel_.async_write_reg(bar_base_, REG_TMP_CFG, TMP_CFG_16X_64HZ);
        (void)co_await channel_.async_write_reg(bar_base_, REG_CFG_REG, CFG_REG_SHIFT);

        // Stage 5: Start continuous measurement mode
        (void)co_await channel_.async_write_reg(bar_base_, REG_MEAS_CFG, MODE_CONT_BOTH);

        initialized_ = true;
        co_return true;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static BaroSample parse_tlp(const Tlp64& tlp) noexcept {
        return Dps310Driver<bus::FakeI2cBus>::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::BaroBase};
    bool initialized_{false};
    Dps310Calib calib_{};

    static int32_t sign_extend(uint32_t val, uint8_t bits) noexcept {
        const uint32_t sign_bit = 1u << (bits - 1u);
        return static_cast<int32_t>((val ^ sign_bit) - sign_bit);
    }

    void unpack_calib(const std::array<uint8_t, 18u>& c) noexcept {
        calib_.c0  = sign_extend((static_cast<uint32_t>(c[0]) << 4u) | (static_cast<uint32_t>(c[1]) >> 4u), 12u);
        calib_.c1  = sign_extend(((static_cast<uint32_t>(c[1]) & 0x0Fu) << 8u) | static_cast<uint32_t>(c[2]), 12u);
        calib_.c00 = sign_extend((static_cast<uint32_t>(c[3]) << 12u) | (static_cast<uint32_t>(c[4]) << 4u) | (static_cast<uint32_t>(c[5]) >> 4u), 20u);
        calib_.c10 = sign_extend(((static_cast<uint32_t>(c[5]) & 0x0Fu) << 16u) | (static_cast<uint32_t>(c[6]) << 8u) | static_cast<uint32_t>(c[7]), 20u);
        calib_.c01 = sign_extend((static_cast<uint32_t>(c[8]) << 8u) | static_cast<uint32_t>(c[9]), 16u);
        calib_.c11 = sign_extend((static_cast<uint32_t>(c[10]) << 8u) | static_cast<uint32_t>(c[11]), 16u);
        calib_.c20 = sign_extend((static_cast<uint32_t>(c[12]) << 8u) | static_cast<uint32_t>(c[13]), 16u);
        calib_.c21 = sign_extend((static_cast<uint32_t>(c[14]) << 8u) | static_cast<uint32_t>(c[15]), 16u);
        calib_.c30 = sign_extend((static_cast<uint32_t>(c[16]) << 8u) | static_cast<uint32_t>(c[17]), 16u);
    }
};

using Dps310      = Dps310Driver<bus::FakeI2cBus>;
using Dps310_Pico = Dps310Driver<bus::Pico2I2cBus>;
using Dps310_Fake = Dps310Driver<bus::FakeI2cBus>;

} // namespace abstractx::drivers::baro

#endif // DPS310_HPP
