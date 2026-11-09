// @verifies [SPEC-DRV-01] docs/HARDWARE_DRIVERS.md
/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The ICM-42688-P driver on AbstractX's runtime and bus, driven as production drives it: a task is
 * started once, then step() and poll_completions() are pumped; nothing calls resume() on a waiting
 * task. The chip is modelled at the SPI-byte level behind the SPI bridge (what a board's IoEngine
 * or the SITL plugin answers):
 *  1. init: WHO_AM_I, soft reset, then every configuration write lands in its register bank
 *  2. a wrong WHO_AM_I is reported, and nothing is written
 *  3. arming DRDY sends DMA_CFG SETUP (burst 0x9D, 14 bytes, Telemetry) then ENABLE
 *  4. a DMA_STREAM sample parses to g, deg/s and degC
 *  5. a silent bus does not hang: with_timeout returns the timeout
 *  6. 1000 inits: the coroutine pool and the engine stay flat
 */
#undef NDEBUG
#include "icm42688p.hpp"

#include "abstractx/abstractx.hpp"

#include <array>
#include <cassert>
#include <cstdio>
#include <variant>

using namespace abstractx;
using abstractx::drivers::imu::DrdyStream;
using abstractx::drivers::imu::Icm42688p;
using abstractx::drivers::imu::ImuConfig;
using abstractx::drivers::imu::ImuInitResult;
namespace regs = abstractx::drivers::imu::icm42688p_regs;

namespace {

SpscTlpRing<> g_tx, g_rx;
coro::CoroutineIoEngine g_io{g_tx, g_rx};

// The chip, behind the SPI bridge: 4 banks of 128 registers, BANK_SEL in every bank.
struct ChipModel {
    uint8_t bus{0}, cs{0};
    uint8_t who_am_i{regs::WHO_AM_I_ICM42688P};
    bool silent{false};
    std::array<std::array<uint8_t, 128>, 4> bank{};
    uint8_t sel{0};
    int writes{0};
    int wrong_device{0};
    std::array<asp_tlp_auto_dma_cfg_t, 2> dma{};
    int dma_n{0};

    void service() {
        Tlp64 req{};
        while (g_tx.pop(req)) {
            if (silent) continue;
            if (req.wire.type == ASP_TLP_TYPE_DMA_CFG) {
                asp_tlp_auto_dma_cfg_t c{};
                std::memcpy(&c, req.wire.payload, sizeof(c));
                if (dma_n < 2) dma[static_cast<size_t>(dma_n)] = c;
                ++dma_n;
                assert(g_rx.push(Tlp64::make_hw_fusion_cpl(req.wire.tag, ASP_STATUS_OK, c.cmd, c.channel_id)));
                continue;
            }
            assert(req.wire.target_address == (ASP_ADDR_SPI_BASE | ASP_REG_XFER) && "SPI bridge requests only");
            asp_tlp_spi_req_header_t h{};
            std::memcpy(&h, req.wire.payload, sizeof(h));
            if (h.bus_id != bus || h.cs_pin != cs) { ++wrong_device; continue; }
            const uint8_t* tx = req.wire.payload + sizeof(h);
            const size_t n = h.tx_len > h.rx_len ? h.tx_len : h.rx_len;
            std::array<uint8_t, 36> rx{};
            const uint8_t reg = tx[0] & 0x7Fu;
            if (tx[0] & 0x80u) {   // read: register auto-increment after the address byte
                for (size_t i = 1; i < n; ++i) {
                    const auto r = static_cast<uint8_t>(reg + i - 1);
                    rx[i] = (sel == 0 && r == regs::REG_WHO_AM_I) ? who_am_i : bank[sel][r];
                }
            } else if (h.tx_len >= 2) {
                if (reg == regs::REG_BANK_SEL) sel = tx[1] & 0x03u;
                else bank[sel][reg] = tx[1];
                ++writes;
            }
            assert(g_rx.push(Tlp64::make_spi_cpl(req.wire.tag, ASP_STATUS_OK, std::span<const uint8_t>(rx.data(), n))));
        }
    }
};

ChipModel g_chip;

template <typename Done>
void pump(Done done, int max_iters = 2'000'000) {
    for (int i = 0; i < max_iters && !done(); ++i) {
        g_chip.service();
        g_io.poll_completions();
        (void)abstractx::step();
    }
}

ImuInitResult g_init{ImuInitResult::BusError};
bool g_init_done = false;
coro::Task<void> run_init(Icm42688p& imu) {
    g_init = co_await imu.init_async();
    g_init_done = true;
}

bool g_arm_ok = false, g_arm_done = false;
coro::Task<void> run_arm(Icm42688p& imu, DrdyStream s) {
    g_arm_ok = co_await imu.arm_drdy_stream(s);
    g_arm_done = true;
}

int g_timeout_index = -1;
coro::Task<void> run_init_bounded(Icm42688p& imu) {
    auto r = co_await coro::with_timeout(imu.init_async(), hal::get_timer_driver(), 20'000u);
    g_timeout_index = static_cast<int>(r.index());
}

ImuInitResult init_once(Icm42688p& imu) {
    g_init_done = false;
    auto t = run_init(imu);
    t.resume();  // resume: starts the never-run task once; step() does the rest
    pump([&] { return g_init_done; });
    assert(g_init_done && t.done() && "init finishes");
    return g_init;
}

} // namespace

int main() {
    g_chip.bus = 1;
    g_chip.cs = 0;
    ImuConfig cfg{};
    cfg.odr_hz = 1000u;
    cfg.gyro_range_dps = 2000u;
    cfg.accel_range_g = 16u;
    Icm42688p imu{g_io, {1, 0}, cfg};

    {   // 1
        assert(init_once(imu) == ImuInitResult::Ok && imu.is_initialized());
        assert(g_chip.wrong_device == 0 && "requests carry the board's bus and chip select");
        assert(g_chip.bank[0][regs::REG_DEVICE_CONFIG] == 0x01u && "soft reset written");
        assert(g_chip.bank[0][regs::REG_PWR_MGMT0] == 0x0Fu && "accel and gyro low-noise");
        assert(g_chip.bank[0][regs::REG_GYRO_CONFIG0] == 0x06u && "2000 dps, 1 kHz");
        assert(g_chip.bank[0][regs::REG_ACCEL_CONFIG0] == 0x06u && "16 g, 1 kHz");
        assert(g_chip.bank[1][regs::B1_GYRO_CONFIG_STATIC3] == regs::AAF_DELT && "gyro AAF in bank 1");
        assert(g_chip.bank[2][regs::B2_ACCEL_CONFIG_STATIC2] == regs::AAF_DELT && "accel AAF in bank 2");
        assert(g_chip.bank[0][regs::REG_INT_SOURCE0] == regs::UI_DRDY_INT1_EN && "DRDY routed to INT1");
        assert(g_chip.sel == 0 && "left in bank 0");
        std::printf("[+] 1 init Ok: %d register writes, all in the right bank\n", g_chip.writes);
    }
    {   // 2
        ChipModel saved = g_chip;
        g_chip.bank = {};
        g_chip.writes = 0;
        g_chip.who_am_i = 0x12u;
        assert(init_once(imu) == ImuInitResult::WhoAmIMismatch && !imu.is_initialized());
        assert(g_chip.writes == 0 && "nothing is written to an unknown chip");
        g_chip = saved;
        std::puts("[+] 2 wrong WHO_AM_I reported, nothing written");
    }
    {   // 3
        g_arm_done = false;
        auto t = run_arm(imu, DrdyStream{2, 14, 1});
        t.resume();  // resume: starts the never-run task once
        pump([] { return g_arm_done; });
        assert(g_arm_done && g_arm_ok && g_chip.dma_n == 2);
        const auto& s = g_chip.dma[0];
        assert(s.cmd == ASP_DMA_CMD_SETUP && s.channel_id == 2 && s.trigger_pin == 14 && s.bus_id == 1);
        assert(s.tx_len == 1 && s.tx_cmd[0] == 0x9Du && s.rx_len == 14 && "burst from TEMP_DATA1, read flag");
        assert(s.tlp_channel == ASP_CHANNEL_TELEMETRY && s.tlp_tag == 1);
        assert(g_chip.dma[1].cmd == ASP_DMA_CMD_ENABLE && g_chip.dma[1].channel_id == 2);
        std::puts("[+] 3 DRDY stream armed: SETUP 0x9D x14 on Telemetry, then ENABLE");
    }
    {   // 4
        // temp +132 (26 degC), accel (0, 0, +2048 = 1 g), gyro (+164 = 10 dps, 0, -164)
        const uint8_t burst[14] = {0x00, 0x84, 0, 0, 0, 0, 0x08, 0x00, 0x00, 0xA4, 0, 0, 0xFF, 0x5C};
        Tlp64 t = Tlp64::make_stream_tx(Channel::Telemetry, 0, burst, 1);
        t.wire.timestamp_ns = 123;
        const auto s = Icm42688p::parse_sample(t);
        assert(s.accel_g[2] == 1.0f && s.accel_g[0] == 0.0f);
        assert(s.gyro_deg_s[0] > 9.99f && s.gyro_deg_s[0] < 10.01f && s.gyro_deg_s[2] < -9.99f);
        assert(s.temperature_c > 25.99f && s.temperature_c < 26.01f && s.timestamp_ns == 123);
        std::puts("[+] 4 sample: 1 g, +-10 dps, 26 degC");
    }
    {   // 5
        g_chip.silent = true;
        g_timeout_index = -1;
        auto t = run_init_bounded(imu);
        t.resume();  // resume: starts the never-run task once
        pump([] { return g_timeout_index != -1; });
        assert(g_timeout_index == 1 && "the timeout wins on a silent bus");
        g_chip.silent = false;
        std::puts("[+] 5 silent bus: init bounded by with_timeout, no hang");
    }
    {   // 6
        assert(init_once(imu) == ImuInitResult::Ok);
        const size_t hw = coro::coro_pool_high_water();
        const size_t used = coro::coro_pool_used();
        for (int i = 0; i < 1000; ++i) assert(init_once(imu) == ImuInitResult::Ok);
        assert(coro::coro_pool_high_water() == hw && coro::coro_pool_used() == used && "no frame leak");
        assert(g_tx.empty() && g_rx.empty() && Dispatcher::dropped_resumes() == 0);
        std::printf("[+] 6 1000 inits: pool high water %zu B, used %zu B, unchanged\n", hw, used);
    }
    std::puts("[SUCCESS] ICM-42688-P on AbstractX's runtime and SPI bridge [SPEC-DRV-01]");
    return 0;
}
