/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - TDK ICM-42688-P on AbstractX's TLP bus: the one driver for every board and SITL.
 */

#ifndef ICM42688P_DRIVER_HPP
#define ICM42688P_DRIVER_HPP

#include "icm42688p_regs.hpp"
#include "imu_base.hpp"
#include "bus_concepts.hpp"
#include "tlp_channel.hpp"
#include "pcie_bar_map.hpp"
#include "coroutine_task.hpp"
#include "tlp_ring.hpp"
#include "config_registry.hpp"
#include "asp_coro.hpp"
#include "asp_tlp64.hpp"
#include "abstractx/hal/platform.hpp"
#include <array>
#include <cstdint>
#include <span>

namespace abstractx::drivers::imu {

// Where the chip is on the board (run-time configuration, never the preprocessor).
struct SpiDevice {
    uint8_t bus_id{0};
    uint8_t cs{0};
};

struct DrdyStream {
    uint8_t auto_channel{0};   // the board's auto channel for this IMU (IoProcessorSetup::channels)
    uint8_t trigger_pin{0};    // INT1 GPIO
    uint8_t source_tag{1};     // tag carried by every sample TLP
};

// @impl [SPEC-DRV-01] docs/HARDWARE_DRIVERS.md
// Register access is SPI bridge TLPs (ASP_ADDR_SPI_BASE), which every target's IoEngine (Linux,
// Pico 2 W, E906) and the SITL plugin answer; samples are an auto-DMA stream armed with DMA_CFG,
// one DMA_STREAM TLP per DRDY edge.
class Icm42688p {
public:
    static constexpr uint8_t kSpiRead = 0x80u;

    Icm42688p(coro::CoroutineIoEngine& io, SpiDevice dev, const ImuConfig& cfg = ImuConfig{}) noexcept
        : io_{io}, dev_{dev}, cfg_{cfg} {}

    // The writes after the soft reset, in order, for this configuration (bank selects included).
    [[nodiscard]] static constexpr std::array<RegWrite, 18> init_writes(const ImuConfig& cfg) noexcept {
        using namespace icm42688p_regs;
        const uint8_t odr = icm42688p::odr_from_hz(cfg.odr_hz);
        const uint8_t gfs = icm42688p::gyro_fs_from_dps(cfg.gyro_range_dps);
        const uint8_t afs = icm42688p::accel_fs_from_g(cfg.accel_range_g);
        const auto aaf5 = static_cast<uint8_t>((AAF_BITSHIFT << 4u) | AAF_DELTSQR_H);
        return {{
            {REG_BANK_SEL, 0x00u},
            {REG_INTF_CONFIG1, AFSR_DISABLE},
            {REG_PWR_MGMT0, static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN)},
            {REG_GYRO_CONFIG0, static_cast<uint8_t>((gfs << 5u) | odr)},
            {REG_ACCEL_CONFIG0, static_cast<uint8_t>((afs << 5u) | odr)},
            {REG_BANK_SEL, 0x01u},
            {B1_GYRO_CONFIG_STATIC3, AAF_DELT},
            {B1_GYRO_CONFIG_STATIC4, AAF_DELTSQR_L},
            {B1_GYRO_CONFIG_STATIC5, aaf5},
            {REG_BANK_SEL, 0x02u},
            {B2_ACCEL_CONFIG_STATIC2, AAF_DELT},
            {B2_ACCEL_CONFIG_STATIC3, AAF_DELTSQR_L},
            {B2_ACCEL_CONFIG_STATIC4, aaf5},
            {REG_BANK_SEL, 0x00u},
            {REG_INT_CONFIG, static_cast<uint8_t>(INT1_DRIVE_PP | INT1_POLARITY_HIGH | INT1_MODE_PULSED)},
            {REG_INT_CONFIG0, 0x00u},
            {REG_INT_CONFIG1, static_cast<uint8_t>(INT_TPULSE_8US | INT_TDEASSERT_DISABLED)},
            {REG_INT_SOURCE0, UI_DRDY_INT1_EN},
        }};
    }

    // A lost completion never arrives: the caller bounds this with coro::with_timeout.
    [[nodiscard]] coro::Task<ImuInitResult> init_async() noexcept {
        using namespace icm42688p_regs;
        initialized_ = false;
        const Tlp64 who = co_await io_.async_transact(read_req(REG_WHO_AM_I));
        if (!ok(who)) co_return ImuInitResult::BusError;
        const uint8_t id = data(who, 1);
        if (id != WHO_AM_I_ICM42688P && id != WHO_AM_I_ICM42605) co_return ImuInitResult::WhoAmIMismatch;

        if (!ok(co_await io_.async_transact(write_req(REG_DEVICE_CONFIG, 0x01u)))) co_return ImuInitResult::BusError;
        (void)co_await hal::get_timer_driver().sleep_ms_async(2u);   // oscillator lock after reset

        for (const RegWrite& w : init_writes(cfg_)) {
            if (!ok(co_await io_.async_transact(write_req(w.reg_addr, w.value)))) co_return ImuInitResult::BusError;
            if (w.reg_addr == REG_PWR_MGMT0) (void)co_await hal::get_timer_driver().sleep_ms_async(1u);
        }
        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    // Arms the board's auto channel: on each INT1 edge it reads the 14-byte burst and sends it as one
    // DMA_STREAM TLP on the Telemetry channel, read with io.async_await_stream().
    [[nodiscard]] coro::Task<bool> arm_drdy_stream(DrdyStream s) noexcept {
        if (!ok(co_await io_.async_transact(Tlp64::make_hw_fusion_setup(auto_dma_cfg(dev_, s))))) co_return false;
        co_return ok(co_await io_.async_transact(Tlp64::make_hw_fusion_control(s.auto_channel, true)));
    }

    [[nodiscard]] static constexpr asp_tlp_auto_dma_cfg_t auto_dma_cfg(SpiDevice dev, DrdyStream s) noexcept {
        asp_tlp_auto_dma_cfg_t c{};
        c.cmd = ASP_DMA_CMD_SETUP;
        c.channel_id = s.auto_channel;
        c.trigger_pin = s.trigger_pin;
        c.trigger_edge = ASP_GPIO_EDGE_RISING;   // INT1 push-pull, active high
        c.bus_type = 0;                          // SPI
        c.bus_id = dev.bus_id;
        c.tx_len = 1;
        c.rx_len = icm42688p::kBurstLen;
        c.tx_cmd[0] = static_cast<uint8_t>(icm42688p::kBurstReg | kSpiRead);
        c.tlp_channel = ASP_CHANNEL_TELEMETRY;
        c.tlp_tag = s.source_tag;
        return c;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    // A DMA_STREAM sample TLP from the armed auto channel.
    [[nodiscard]] static ImuSample parse_sample(const Tlp64& tlp) noexcept {
        return icm42688p::parse_burst(std::span<const uint8_t>(tlp.wire.payload, icm42688p::kBurstLen),
                                      tlp.wire.timestamp_ns);
    }

    [[nodiscard]] static constexpr ImuAutoDmaProfile auto_dma_profile() noexcept {
        return ImuAutoDmaProfile{
            .burst_reg = icm42688p_regs::REG_TEMP_DATA1,
            .burst_len = icm42688p::kBurstLen,
            .trigger_edge_rising = true,
        };
    }

    [[nodiscard]] static ImuSample parse_raw_burst(std::span<const uint8_t> raw, uint64_t timestamp_ns) noexcept {
        return icm42688p::parse_burst(raw, timestamp_ns);
    }

    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        if (tlp.wire.type == ASP_TLP_TYPE_DMA_STREAM) {
            return parse_raw_burst(tlp.payload(), tlp.wire.timestamp_ns);
        }

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

        sample.accel_g[0] = static_cast<float>(raw_ax) * (1.0f / 2048.0f);
        sample.accel_g[1] = static_cast<float>(raw_ay) * (1.0f / 2048.0f);
        sample.accel_g[2] = static_cast<float>(raw_az) * (1.0f / 2048.0f);

        sample.gyro_deg_s[0] = static_cast<float>(raw_gx) * (1.0f / 16.4f);
        sample.gyro_deg_s[1] = static_cast<float>(raw_gy) * (1.0f / 16.4f);
        sample.gyro_deg_s[2] = static_cast<float>(raw_gz) * (1.0f / 16.4f);

        sample.temperature_c = (static_cast<float>(raw_t) / 132.48f) + 25.0f;

        return sample;
    }

    [[nodiscard]] Tlp64 read_req(uint8_t reg) const noexcept {
        const uint8_t tx[1] = {static_cast<uint8_t>(reg | kSpiRead)};
        return Tlp64::make_spi_transfer(dev_.bus_id, dev_.cs, tx, 2u);   // byte 1 clocks the value in
    }

    [[nodiscard]] Tlp64 write_req(uint8_t reg, uint8_t value) const noexcept {
        const uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7Fu), value};
        return Tlp64::make_spi_transfer(dev_.bus_id, dev_.cs, tx, 0u);
    }

    // A bridge or DMA_CFG completion succeeded (a refused request comes back with flags 1).
    [[nodiscard]] static bool ok(const Tlp64& cpl) noexcept {
        return cpl.wire.flags == 0u && cpl.wire.payload[0] == ASP_STATUS_OK;
    }

    // Byte i clocked in during an SPI bridge transfer (after the 4-byte completion header).
    [[nodiscard]] static uint8_t data(const Tlp64& cpl, size_t i) noexcept {
        return cpl.wire.payload[sizeof(asp_tlp_spi_cpl_header_t) + i];
    }

private:
    coro::CoroutineIoEngine& io_;
    SpiDevice dev_{};
    ImuConfig cfg_{};
    bool initialized_{false};
};

// ============================================================================
// Direct SPI Bus Driver
// ============================================================================
template <bus::IsSpiBus SpiBusT = bus::FakeSpiBus>
class Icm42688PDriver {
public:
    explicit Icm42688PDriver(SpiBusT& bus, const ImuConfig& cfg = ImuConfig{}) noexcept
        : bus_{bus}, cfg_{cfg} {}

    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        using namespace icm42688p_regs;

        uint8_t who = bus_.read_reg(REG_WHO_AM_I);
        if (who != WHO_AM_I_ICM42688P && who != WHO_AM_I_ICM42605) {
            co_return ImuInitResult::WhoAmIMismatch;
        }

        bus_.write_reg(REG_DEVICE_CONFIG, 0x01u);
        co_await sleep_ms(2u);

        select_bank(0x00u);
        bus_.write_reg(REG_INTF_CONFIG1, AFSR_DISABLE);

        bus_.write_reg(REG_PWR_MGMT0, static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN));
        co_await sleep_ms(1u);

        uint8_t gyro_odr  = icm42688p::odr_from_hz(cfg_.odr_hz);
        uint8_t gyro_fs   = icm42688p::gyro_fs_from_dps(cfg_.gyro_range_dps);
        uint8_t accel_fs  = icm42688p::accel_fs_from_g(cfg_.accel_range_g);

        bus_.write_reg(REG_GYRO_CONFIG0,  static_cast<uint8_t>((gyro_fs  << 5u) | gyro_odr));
        bus_.write_reg(REG_ACCEL_CONFIG0, static_cast<uint8_t>((accel_fs << 5u) | gyro_odr));

        select_bank(0x01u);
        bus_.write_reg(B1_GYRO_CONFIG_STATIC3, AAF_DELT);
        bus_.write_reg(B1_GYRO_CONFIG_STATIC4, AAF_DELTSQR_L);
        bus_.write_reg(B1_GYRO_CONFIG_STATIC5, static_cast<uint8_t>((AAF_BITSHIFT << 4u) | AAF_DELTSQR_H));

        select_bank(0x02u);
        bus_.write_reg(B2_ACCEL_CONFIG_STATIC2, AAF_DELT);
        bus_.write_reg(B2_ACCEL_CONFIG_STATIC3, AAF_DELTSQR_L);
        bus_.write_reg(B2_ACCEL_CONFIG_STATIC4, static_cast<uint8_t>((AAF_BITSHIFT << 4u) | AAF_DELTSQR_H));

        select_bank(0x00u);
        bus_.write_reg(REG_INT_CONFIG, static_cast<uint8_t>(INT1_DRIVE_PP | INT1_POLARITY_HIGH | INT1_MODE_PULSED));
        bus_.write_reg(REG_INT_CONFIG0, 0x00u);
        bus_.write_reg(REG_INT_CONFIG1, static_cast<uint8_t>(INT_TPULSE_8US | INT_TDEASSERT_DISABLED));
        bus_.write_reg(REG_INT_SOURCE0, UI_DRDY_INT1_EN);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static constexpr ImuAutoDmaProfile auto_dma_profile() noexcept {
        return ImuAutoDmaProfile{
            .burst_reg = icm42688p_regs::REG_TEMP_DATA1,
            .burst_len = 14u,
            .trigger_edge_rising = true,
        };
    }

    [[nodiscard]] static ImuSample parse_raw_burst(std::span<const uint8_t> raw, uint64_t timestamp_ns) noexcept {
        return icm42688p::parse_burst(raw, timestamp_ns);
    }

    template <size_t N>
    [[nodiscard]] Task<void> sample_loop(SpscTlpRing<N>& ring) noexcept {
        while (true) {
            uint64_t ts = get_hw_timestamp_ns();
            bus_.burst_read(icm42688p_regs::REG_TEMP_DATA1, rx_buf_.data(), 14u);
            Tlp64 tlp{};
            tlp.wire.type = ASP_TLP_TYPE_DMA_STREAM;
            tlp.wire.timestamp_ns = ts;
            std::memcpy(tlp.wire.payload, rx_buf_.data(), 14u);
            (void)ring.push(tlp);
            co_await YieldTick{};
        }
    }

    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        if (tlp.wire.type == ASP_TLP_TYPE_DMA_STREAM) {
            return parse_raw_burst(tlp.payload(), tlp.wire.timestamp_ns);
        }
        return icm42688p::parse_burst(std::span<const uint8_t>(tlp.wire.payload, 14u), tlp.wire.timestamp_ns);
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

        auto opt_who = co_await channel_.async_read_reg(bar_base_, REG_WHO_AM_I);
        if (!opt_who.has_value() || (*opt_who != WHO_AM_I_ICM42688P && *opt_who != WHO_AM_I_ICM42605)) {
            co_return ImuInitResult::WhoAmIMismatch;
        }

        if (!co_await channel_.async_write_reg(bar_base_, REG_DEVICE_CONFIG, 0x01u)) {
            co_return ImuInitResult::BusError;
        }
        co_await sleep_ms(2u);

        (void)co_await channel_.async_write_reg(bar_base_, REG_BANK_SEL, 0x00u);
        (void)co_await channel_.async_write_reg(bar_base_, REG_INTF_CONFIG1, AFSR_DISABLE);

        (void)co_await channel_.async_write_reg(bar_base_, REG_PWR_MGMT0, static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN));
        co_await sleep_ms(1u);

        uint8_t gyro_odr  = icm42688p::odr_from_hz(cfg_.odr_hz);
        uint8_t gyro_fs   = icm42688p::gyro_fs_from_dps(cfg_.gyro_range_dps);
        (void)co_await channel_.async_write_reg(bar_base_, REG_GYRO_CONFIG0, static_cast<uint8_t>((gyro_fs << 5u) | gyro_odr));

        uint8_t accel_fs  = icm42688p::accel_fs_from_g(cfg_.accel_range_g);
        (void)co_await channel_.async_write_reg(bar_base_, REG_ACCEL_CONFIG0, static_cast<uint8_t>((accel_fs << 5u) | gyro_odr));

        (void)co_await channel_.async_write_reg(bar_base_, REG_INT_SOURCE0, UI_DRDY_INT1_EN);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

    [[nodiscard]] static constexpr ImuAutoDmaProfile auto_dma_profile() noexcept {
        return Icm42688p::auto_dma_profile();
    }

    [[nodiscard]] static ImuSample parse_raw_burst(std::span<const uint8_t> raw, uint64_t timestamp_ns) noexcept {
        return icm42688p::parse_burst(raw, timestamp_ns);
    }

    [[nodiscard]] static ImuSample parse_tlp(const Tlp64& tlp) noexcept {
        return Icm42688p::parse_tlp(tlp);
    }

private:
    bus::TlpChannel& channel_;
    uint32_t bar_base_{bar::ImuBase};
    ImuConfig cfg_{};
    bool initialized_{false};
};

using Icm42688P = Icm42688p;
using Icm42688P_Pico = Icm42688PDriver<bus::Pico2SpiBus>;
using Icm42688P_Fake = Icm42688PDriver<bus::FakeSpiBus>;

} // namespace abstractx::drivers::imu

#endif // ICM42688P_DRIVER_HPP
