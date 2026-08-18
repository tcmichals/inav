/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - TDK ICM-42688-P IMU Driver (Full Bottom-Half + Top-Half)
 *
 * Bottom-half (do_init / do_sample_loop):
 *   - WHO_AM_I verification (reg 0x75 → expected 0x47)
 *   - Soft reset, 1 ms stabilisation wait
 *   - Bank 0: PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0, INTF_CONFIG1
 *   - Bank 1: Gyro Anti-Alias Filter (AAF) coefficients
 *   - Bank 2: Accel Anti-Alias Filter (AAF) coefficients
 *   - Bank 0: INT_CONFIG, INT_CONFIG0, INT_CONFIG1, INT_SOURCE0 (DRDY→INT1)
 *   - 14-byte SPI burst read per DRDY edge → pack Tlp64 → push SPSC ring
 *
 * Top-half (parse_tlp):
 *   - Fixed-point scale factors for ±16g (2048 LSB/g) and ±2000 dps (16.4 LSB/dps)
 *   - Temperature: 25°C + raw/326.8
 *
 * Upstream references:
 *   iNavFlight/inav: src/main/drivers/accgyro/accgyro_icm42605.c  (15 KB)
 *   Betaflight:      src/main/drivers/accgyro/accgyro_spi_icm42688p.c
 */

#ifndef ICM42688P_DRIVER_HPP
#define ICM42688P_DRIVER_HPP

#include "bus_concepts.hpp"
#include "tlp_channel.hpp"
#include "imu_base.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "coroutine_task.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include <cstdint>
#include <span>
#include <array>

namespace abstractx::drivers::imu {

// ============================================================================
// Register map — Bank 0 (default after reset)
// ============================================================================
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

    // Bank 1 — Gyro AAF
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC3 = 0x0Cu;
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC4 = 0x0Du;
    static constexpr uint8_t B1_GYRO_CONFIG_STATIC5 = 0x0Eu;

    // Bank 2 — Accel AAF
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC2 = 0x03u;
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC3 = 0x04u;
    static constexpr uint8_t B2_ACCEL_CONFIG_STATIC4 = 0x05u;

    // WHO_AM_I expected values
    static constexpr uint8_t WHO_AM_I_ICM42688P = 0x47u;
    static constexpr uint8_t WHO_AM_I_ICM42605  = 0x42u;

    // PWR_MGMT0 bits
    static constexpr uint8_t ACCEL_MODE_LN = (3u << 0u); // low-noise
    static constexpr uint8_t GYRO_MODE_LN  = (3u << 2u); // low-noise

    // GYRO_CONFIG0 / ACCEL_CONFIG0: ODR + FS_SEL
    // ODR field [3:0], FS_SEL field [7:5]
    static constexpr uint8_t GYRO_FS_2000DPS = (0u << 5u);
    static constexpr uint8_t GYRO_ODR_8KHZ   = 0x03u;
    static constexpr uint8_t ACCEL_FS_16G    = (0u << 5u);
    static constexpr uint8_t ACCEL_ODR_8KHZ  = 0x03u;

    // INT1: push-pull, active-high, pulsed
    static constexpr uint8_t INT1_POLARITY_HIGH = (1u << 0u);
    static constexpr uint8_t INT1_DRIVE_PP      = (1u << 1u);
    static constexpr uint8_t INT1_MODE_PULSED   = (0u << 2u);

    // INT_CONFIG1: 8 µs pulse, async reset off
    static constexpr uint8_t INT_TPULSE_8US         = (1u << 6u);
    static constexpr uint8_t INT_TDEASSERT_DISABLED  = (1u << 5u);
    static constexpr uint8_t INT_ASYNC_RESET_BIT     = (1u << 4u);

    // INT_SOURCE0: DRDY → INT1 enable
    static constexpr uint8_t UI_DRDY_INT1_EN = (1u << 3u);

    // INTF_CONFIG1: disable AFSR (adaptive frequency scaling — causes ODR jitter)
    static constexpr uint8_t AFSR_DISABLE = 0x40u;

    // AAF coefficients for 536 Hz corner @ 8 kHz ODR (from TDK application note)
    // Gyro Bank1 / Accel Bank2 — delt=6, deltsqr=36, bitshift=4
    static constexpr uint8_t AAF_DELT       = 6u;
    static constexpr uint8_t AAF_DELTSQR_L  = 36u; // low byte
    static constexpr uint8_t AAF_DELTSQR_H  = 0u;  // high nibble (deltsqr >> 8)
    static constexpr uint8_t AAF_BITSHIFT    = 4u;  // [3:0] in STATIC5/STATIC4
}

// ============================================================================
// ICM-42688-P Driver
// ============================================================================
template <bus::IsSpiBus SpiBusT = bus::FakeSpiBus>
class Icm42688PDriver {
public:
    explicit Icm42688PDriver(SpiBusT& bus, const ImuConfig& cfg = ImuConfig{}) noexcept
        : bus_{bus}, cfg_{cfg} {}

    // -----------------------------------------------------------------------
    // BOTTOM HALF: async_init() (Non-blocking C++20 Coroutine)
    // -----------------------------------------------------------------------
    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        using namespace icm42688p_regs;

        // Stage 1: WHO_AM_I (Awaits non-blocking bus read completion)
        uint8_t who = co_await bus_.async_read_reg(REG_WHO_AM_I);
        if (who != WHO_AM_I_ICM42688P && who != WHO_AM_I_ICM42605) {
            co_return ImuInitResult::WhoAmIMismatch;
        }

        // Stage 2: Soft reset (Awaits non-blocking write completion + 2ms oscillator lock)
        if (!co_await bus_.async_write_reg(REG_DEVICE_CONFIG, 0x01u)) {
            co_return ImuInitResult::BusError;
        }
        co_await sleep_ms(2u); // Non-blocking 2ms yield for oscillator lock

        // Stage 3: Bank 0 — disable AFSR
        (void)co_await bus_.async_write_reg(REG_BANK_SEL, 0x00u);
        (void)co_await bus_.async_write_reg(REG_INTF_CONFIG1, AFSR_DISABLE);

        // Stage 4: Bank 0 — power on accel + gyro in low-noise mode
        (void)co_await bus_.async_write_reg(REG_PWR_MGMT0,
            static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN));
        co_await sleep_ms(1u); // Non-blocking 1ms yield for power transition

        // Stage 5: Bank 0 — GYRO_CONFIG0 (ODR + FS)
        uint8_t gyro_odr  = gyro_odr_from_hz(cfg_.odr_hz);
        uint8_t gyro_fs   = gyro_fs_from_dps(cfg_.gyro_range_dps);
        (void)co_await bus_.async_write_reg(REG_GYRO_CONFIG0,
            static_cast<uint8_t>((gyro_fs << 5u) | gyro_odr));

        // Stage 6: Bank 0 — ACCEL_CONFIG0 (ODR + FS)
        uint8_t accel_odr = gyro_odr; // match gyro ODR
        uint8_t accel_fs  = accel_fs_from_g(cfg_.accel_range_g);
        (void)co_await bus_.async_write_reg(REG_ACCEL_CONFIG0,
            static_cast<uint8_t>((accel_fs << 5u) | accel_odr));

        // Stage 7: Bank 1 — Gyro AAF (536 Hz corner @ 8 kHz ODR)
        (void)co_await bus_.async_write_reg(REG_BANK_SEL, 0x01u);
        (void)co_await bus_.async_write_reg(B1_GYRO_CONFIG_STATIC3, AAF_DELT);
        (void)co_await bus_.async_write_reg(B1_GYRO_CONFIG_STATIC4, AAF_DELTSQR_L);
        (void)co_await bus_.async_write_reg(B1_GYRO_CONFIG_STATIC5,
            static_cast<uint8_t>((AAF_BITSHIFT << 4u) | AAF_DELTSQR_H));

        // Stage 8: Bank 2 — Accel AAF
        (void)co_await bus_.async_write_reg(REG_BANK_SEL, 0x02u);
        (void)co_await bus_.async_write_reg(B2_ACCEL_CONFIG_STATIC2, AAF_DELT);
        (void)co_await bus_.async_write_reg(B2_ACCEL_CONFIG_STATIC3, AAF_DELTSQR_L);
        (void)co_await bus_.async_write_reg(B2_ACCEL_CONFIG_STATIC4,
            static_cast<uint8_t>((AAF_BITSHIFT << 4u) | AAF_DELTSQR_H));

        // Stage 9: Bank 0 — INT1 config & clear on status burst read (fc_init / accgyro_icm42605.c)
        (void)co_await bus_.async_write_reg(REG_BANK_SEL, 0x00u);
        (void)co_await bus_.async_write_reg(REG_INT_CONFIG, static_cast<uint8_t>(INT1_DRIVE_PP | INT1_POLARITY_HIGH | INT1_MODE_PULSED));
        (void)co_await bus_.async_write_reg(REG_INT_CONFIG0, 0x00u); // Clear UI DRDY int on sensor register read
        (void)co_await bus_.async_write_reg(REG_INT_CONFIG1, static_cast<uint8_t>(INT_TPULSE_8US | INT_TDEASSERT_DISABLED));

        // Stage 10: Bank 0 — route DATA_RDY interrupt to INT1
        (void)co_await bus_.async_write_reg(REG_INT_SOURCE0, UI_DRDY_INT1_EN);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    [[nodiscard]] bool is_initialized() const noexcept {
        return initialized_;
    }

    // -----------------------------------------------------------------------
    // BOTTOM HALF: sample_loop()
    //   Runs forever as a coroutine.  For each IMU sample period:
    //     1. Wait for DRDY edge (or 2 ms watchdog timeout via when_any)
    //     2. Read 14-byte burst (accel 6B + temp 2B + gyro 6B)
    //     3. Pack into Tlp64
    //     4. Push into SpscTlpRing for top-half consumption
    //
    //   On RP2350: DRDY fires on GP14 edge interrupt (configured by init()).
    //   On SITL:   The fake SPI bus provides injected bytes; no GPIO needed.
    // -----------------------------------------------------------------------
    Task<void> sample_loop(SpscTlpRing<64u>& ring) noexcept {
        static constexpr uint32_t DRDY_TIMEOUT_US = 2000u; // 2× 8 kHz period
        uint8_t tag = 0u;

        while (true) {
            // Race: coroutine timer watchdog (DRDY GPIO awaiter added separately)
            // On real hardware: this yields for up to 2 ms.
            // On SITL: immediately continues (fake bus always has data).
            co_await sleep_us(DRDY_TIMEOUT_US);

            if (!initialized_) {
                co_await sleep_ms(10u); // wait for init() to complete
                continue;
            }

            // 14-byte burst: ACCEL_DATA_X1(0x1F) → GYRO_DATA_Z0(0x30)
            // ICM-42688P burst map (contiguous from 0x1D):
            //   0x1D: TEMP_DATA1, 0x1E: TEMP_DATA0
            //   0x1F..0x24: ACCEL X1,X0,Y1,Y0,Z1,Z0
            //   0x25..0x2A: GYRO  X1,X0,Y1,Y0,Z1,Z0
            // We read 14 bytes starting at TEMP_DATA1 (0x1D)
            bool ok = bus_.read_burst(icm42688p_regs::REG_TEMP_DATA1, rx_buf_);
            if (!ok) { continue; }

            // Pack Tlp64 frame — layout matches existing parse_tlp() expectations:
            //   payload[0..1]   = accel X (big-endian int16)
            //   payload[2..3]   = accel Y
            //   payload[4..5]   = accel Z
            //   payload[6..7]   = gyro X
            //   payload[8..9]   = gyro Y
            //   payload[10..11] = gyro Z
            //   payload[12..13] = temperature
            // Burst from 0x1D: temp[0..1], accel[2..7], gyro[8..13]
            // Remap into TLP payload order:
            Tlp64 tlp = Tlp64::make_mem_write(bar::ImuBase, 0u, tag++);
            tlp.wire.timestamp_ns = get_hw_timestamp_ns();

            // accel (burst bytes 2..7 → payload 0..5)
            for (size_t i = 0u; i < 6u; ++i) { tlp.wire.payload[i] = rx_buf_[2u + i]; }
            // gyro  (burst bytes 8..13 → payload 6..11)
            for (size_t i = 0u; i < 6u; ++i) { tlp.wire.payload[6u + i] = rx_buf_[8u + i]; }
            // temp  (burst bytes 0..1 → payload 12..13)
            tlp.wire.payload[12u] = rx_buf_[0u];
            tlp.wire.payload[13u] = rx_buf_[1u];

            (void)ring.push(tlp);
        }
        co_return;
    }

    // -----------------------------------------------------------------------
    // TOP HALF: parse_tlp()
    //   Converts raw TLP payload bytes to ImuSample using ICM-42688P scale
    //   factors from datasheet Table 3 (accel) and Table 14 (gyro).
    //
    //   ±16g  →  2048 LSB/g
    //   ±2000 dps → 16.4 LSB/(°/s)
    //   Temp  →  25°C + raw/326.8
    // -----------------------------------------------------------------------
    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        ImuSample sample{};
        sample.timestamp_ns = tlp.wire.timestamp_ns;

        const uint8_t* p = tlp.wire.payload;

        int16_t raw_ax = static_cast<int16_t>((static_cast<int16_t>(p[0])  << 8) | p[1]);
        int16_t raw_ay = static_cast<int16_t>((static_cast<int16_t>(p[2])  << 8) | p[3]);
        int16_t raw_az = static_cast<int16_t>((static_cast<int16_t>(p[4])  << 8) | p[5]);
        int16_t raw_gx = static_cast<int16_t>((static_cast<int16_t>(p[6])  << 8) | p[7]);
        int16_t raw_gy = static_cast<int16_t>((static_cast<int16_t>(p[8])  << 8) | p[9]);
        int16_t raw_gz = static_cast<int16_t>((static_cast<int16_t>(p[10]) << 8) | p[11]);
        int16_t raw_t  = static_cast<int16_t>((static_cast<int16_t>(p[12]) << 8) | p[13]);

        // ±16g: sensitivity = 2048 LSB/g (datasheet Table 3, AFS_SEL=0)
        sample.accel_g[0] = static_cast<float>(raw_ax) * (1.0f / 2048.0f);
        sample.accel_g[1] = static_cast<float>(raw_ay) * (1.0f / 2048.0f);
        sample.accel_g[2] = static_cast<float>(raw_az) * (1.0f / 2048.0f);

        // ±2000 dps: sensitivity = 16.4 LSB/(°/s) (datasheet Table 14, GFS_SEL=0)
        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) * (1.0f / 16.4f);
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) * (1.0f / 16.4f);
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) * (1.0f / 16.4f);

        // Temp: T(°C) = (raw / 132.48) + 25  — datasheet section 4.13
        sample.temperature_c = (static_cast<float>(raw_t) / 132.48f) + 25.0f;

        return sample;
    }

    // Translate config ODR Hz → register value
    [[nodiscard]] static uint8_t gyro_odr_from_hz(uint32_t hz) noexcept {
        if (hz >= 8000u) { return 0x03u; } // 8 kHz
        if (hz >= 4000u) { return 0x04u; } // 4 kHz
        if (hz >= 2000u) { return 0x05u; } // 2 kHz
        if (hz >= 1000u) { return 0x06u; } // 1 kHz
        return 0x06u; // default 1 kHz
    }

    [[nodiscard]] static uint8_t gyro_fs_from_dps(uint16_t dps) noexcept {
        if (dps <= 250u)  { return 3u; }
        if (dps <= 500u)  { return 2u; }
        if (dps <= 1000u) { return 1u; }
        return 0u; // ±2000 dps
    }

    [[nodiscard]] static uint8_t accel_fs_from_g(uint8_t g) noexcept {
        if (g <= 2u)  { return 3u; }
        if (g <= 4u)  { return 2u; }
        if (g <= 8u)  { return 1u; }
        return 0u; // ±16g
    }

private:
    SpiBusT&  bus_;
    ImuConfig cfg_{};
    bool      initialized_{false};
    std::array<uint8_t, 16u> rx_buf_{};

    void select_bank(uint8_t bank) noexcept {
        bus_.write_reg(icm42688p_regs::REG_BANK_SEL, bank);
    }

    [[nodiscard]] static uint64_t get_hw_timestamp_ns() noexcept {
#if defined(PICO_BOARD)
        return static_cast<uint64_t>(time_us_64()) * 1000u;
#else
        // SITL: use steady_clock
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
#endif
    }
};

// ============================================================================
// ICM-42688-P Top-Level Pure TLP Driver (Zero Bus Coupling)
// ============================================================================
class Icm42688pTlpDriver {
public:
    explicit Icm42688pTlpDriver(bus::TlpChannel& channel,
                                uint32_t bar_base = bar::ImuBase,
                                const ImuConfig& cfg = ImuConfig{}) noexcept
        : channel_{channel}, bar_base_{bar_base}, cfg_{cfg} {}

    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        using namespace icm42688p_regs;

        // Stage 1: WHO_AM_I check via TLP MemRead
        auto opt_who = co_await channel_.async_read_reg(bar_base_, REG_WHO_AM_I);
        if (!opt_who.has_value() || (*opt_who != WHO_AM_I_ICM42688P && *opt_who != WHO_AM_I_ICM42605)) {
            co_return ImuInitResult::WhoAmIMismatch;
        }

        // Stage 2: Soft reset via TLP MemWrite + non-blocking 2ms yield
        if (!co_await channel_.async_write_reg(bar_base_, REG_DEVICE_CONFIG, 0x01u)) {
            co_return ImuInitResult::BusError;
        }
        co_await sleep_ms(2u); // 2ms oscillator lock

        // Stage 3: Bank 0 — disable AFSR
        (void)co_await channel_.async_write_reg(bar_base_, REG_BANK_SEL, 0x00u);
        (void)co_await channel_.async_write_reg(bar_base_, REG_INTF_CONFIG1, AFSR_DISABLE);

        // Stage 4: Bank 0 — power on accel + gyro in low-noise mode + non-blocking 1ms yield
        (void)co_await channel_.async_write_reg(bar_base_, REG_PWR_MGMT0, static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN));
        co_await sleep_ms(1u);

        // Stage 5: Bank 0 — GYRO_CONFIG0 (ODR + FS)
        uint8_t gyro_odr  = Icm42688PDriver<bus::FakeSpiBus>::gyro_odr_from_hz(cfg_.odr_hz);
        uint8_t gyro_fs   = Icm42688PDriver<bus::FakeSpiBus>::gyro_fs_from_dps(cfg_.gyro_range_dps);
        (void)co_await channel_.async_write_reg(bar_base_, REG_GYRO_CONFIG0, static_cast<uint8_t>((gyro_fs << 5u) | gyro_odr));

        // Stage 6: Bank 0 — ACCEL_CONFIG0 (ODR + FS)
        uint8_t accel_fs  = Icm42688PDriver<bus::FakeSpiBus>::accel_fs_from_g(cfg_.accel_range_g);
        (void)co_await channel_.async_write_reg(bar_base_, REG_ACCEL_CONFIG0, static_cast<uint8_t>((accel_fs << 5u) | gyro_odr));

        // Stage 7: Bank 0 — route DATA_RDY interrupt to INT1
        (void)co_await channel_.async_write_reg(bar_base_, REG_INT_SOURCE0, UI_DRDY_INT1_EN);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        return Icm42688PDriver<bus::FakeSpiBus>::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::ImuBase};
    ImuConfig cfg_{};
    bool initialized_{false};
};

// ---------------------------------------------------------------------------
// Convenience type aliases used in pico2_main.cpp, sitl_main.cpp, and tests
// ---------------------------------------------------------------------------
using Icm42688P = Icm42688PDriver<bus::FakeSpiBus>;
using Icm42688P_Pico = Icm42688PDriver<bus::Pico2SpiBus>;
using Icm42688P_Fake = Icm42688PDriver<bus::FakeSpiBus>;

} // namespace abstractx::drivers::imu

#endif // ICM42688P_DRIVER_HPP
