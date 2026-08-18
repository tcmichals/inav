/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - RP2350 Pico 2 W Bare-Metal Entry Point
 *
 * Core 0: Wi-Fi / lwIP MSP Server (TCP 5760) + QSPI flash blackbox logging
 * Core 1: Isolated RT flight loop — IMU bottom-half + top-half + EKF3 + PID
 *
 * Boot sequence (Core 0, before multicore launch):
 *   1. stdio + CYW43439 Wi-Fi init
 *   2. Load MasterConfig (incl. SensorConfig) from flash sector 0x1F0000
 *   3. Init SPI1 + I2C1 + UART0 peripheral buses from SensorConfig
 *   4. ICM-42688P init: WHO_AM_I → reset → banks → AAF → INT1 config
 *   5. BMP280 init: WHO_AM_I → reset → 24-byte calib → CTRL_MEAS → CONFIG
 *   6. QMC5883L init: WHO_AM_I → mode config (future)
 *   7. Launch Core 1 flight loop
 *   8. Core 0 background: lwIP poll + MSP server
 */

#if defined(PICO_BOARD)

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"

#include "coroutine_task.hpp"
#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include "bus_concepts.hpp"
#include "pico2_target.hpp"
#include "imu_base.hpp"
#include "icm42688p.hpp"
#include "bmp280.hpp"
#include "gps_driver.hpp"
#include "msp_protocol.hpp"
#include "msp_server.hpp"
#include "lwip_msp_transport.hpp"
#include "flight_engine_template.hpp"

using namespace abstractx;

// ---------------------------------------------------------------------------
// Global SPSC rings: bottom-half sensor loops push, top-half engine pops.
// Sized for RP2350 520 KB SRAM (g_logging_ring retains 64 frames = 4 KB
// of boot diagnostics and calibration before TCP client attaches).
// ---------------------------------------------------------------------------
static SpscTlpRing<64u> g_telemetry_ring;
static SpscTlpRing<64u> g_logging_ring;

// ---------------------------------------------------------------------------
// Singleton driver instances (constructed from SensorConfig)
// ---------------------------------------------------------------------------
static drivers::gps::GpsDriver g_gps_driver;
static msp::MspServer<msp::LwipMspTransport> g_msp_server{5760u};

// IMU and baro drivers are constructed in core1_flight_loop_entry() so they
// share Core 1's stack context and can safely call co_await on it.

// ---------------------------------------------------------------------------
// Core 1: Sensor bottom-halves + flight engine (all on one isolated core)
// ---------------------------------------------------------------------------
void core1_flight_loop_entry() {
    // Retrieve bus singletons initialised by Core 0
    auto& imu_bus  = target::pico2::Pico2Target::get_imu_spi_bus();
    auto& baro_bus = target::pico2::Pico2Target::get_baro_mag_i2c_bus();

    const auto& sc = ConfigRegistry::get().sensor;

    // -----------------------------------------------------------------------
    // Build driver instances from config
    // -----------------------------------------------------------------------
    drivers::imu::ImuConfig imu_cfg{};
    imu_cfg.odr_hz          = sc.imu_odr_hz;
    imu_cfg.accel_range_g   = sc.imu_accel_range_g;
    imu_cfg.gyro_range_dps  = sc.imu_gyro_range_dps;
    imu_cfg.enable_drdy_int = sc.imu_enable_drdy;

    drivers::imu::Icm42688PDriver<drivers::bus::Pico2SpiBus> imu_driver{imu_bus, imu_cfg};
    drivers::baro::Bmp280Driver<drivers::bus::Pico2I2cBus>   baro_driver{baro_bus, sc.baro_i2c_addr};

    // -----------------------------------------------------------------------
    // Stage 1: Init sensors — non-blocking coroutine async_init
    // -----------------------------------------------------------------------
    {
        auto imu_init_task = imu_driver.async_init();
        imu_init_task.resume();
        auto baro_init_task = baro_driver.async_init();
        baro_init_task.resume();
    }

    // -----------------------------------------------------------------------
    // Stage 2: Launch concurrent sensor sample loops
    //   - IMU: fires at 8 kHz triggered by DRDY (GP14) edge
    //   - Baro: fires at 100 Hz (10 ms period, 45 ms ADC wait)
    // -----------------------------------------------------------------------
    auto imu_loop_task  = imu_driver.sample_loop(g_telemetry_ring);
    auto baro_loop_task = baro_driver.sample_loop(g_telemetry_ring);

    // -----------------------------------------------------------------------
    // Stage 3: Launch top-half flight engine
    // -----------------------------------------------------------------------
    target::pico2::Pico2Target pico_target{};
    target::TargetAdapter<target::pico2::Pico2Target> platform_adapter{pico_target};

    flight::FlightEngine<
        target::TargetAdapter<target::pico2::Pico2Target>,
        drivers::imu::Icm42688PDriver<drivers::bus::Pico2SpiBus>,
        drivers::baro::Bmp280Driver<drivers::bus::Pico2I2cBus>,
        drivers::gps::GpsDriver,
        4u
    > pico_engine{platform_adapter, g_gps_driver};

    auto engine_task = pico_engine.run_loop(g_telemetry_ring, g_logging_ring);

    // -----------------------------------------------------------------------
    // Stage 4: Cooperative coroutine scheduler — 8 kHz tick
    //   All tasks share one cooperative executor on Core 1.
    //   No preemption, no OS — deterministic WCET.
    // -----------------------------------------------------------------------
    while (true) {
        imu_loop_task.resume();
        baro_loop_task.resume();
        engine_task.resume();
        ::sleep_us(125u); // 8 kHz = 125 µs per tick
    }
}

// ---------------------------------------------------------------------------
// Core 0: Peripherals, Wi-Fi, lwIP MSP TCP Server + Flash Storage
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();

    // 1. Init CYW43439 Wi-Fi chip + lwIP stack
    if (cyw43_arch_init() != 0) { return 1; }
    cyw43_arch_enable_sta_mode();

    // 2. Load MasterConfig (SensorConfig + PID + Nav) from flash
    storage::FlashStorageAdapter flash{storage::FlashMediumType::Pico2OnChip};
    ConfigRegistry::load_from_storage(flash);

    // 3. Init all physical peripheral buses from SensorConfig
    //    Must happen on Core 0 before Core 1 touches the bus singletons
    target::pico2::Pico2Target::init_core0_peripherals();

    // 4. Start lwIP MSP TCP Server on port 5760
    g_msp_server.start();

    // 5. Launch Core 1 — dedicated real-time flight engine
    multicore_launch_core1(core1_flight_loop_entry);

    // 6. Core 0 background: Wi-Fi + MSP polling + pre-connect backlog offload
    while (true) {
        cyw43_arch_poll();
        g_msp_server.poll();

        // Offload retained boot & runtime blackbox TLPs if client is attached
        if (g_msp_server.is_running()) {
            uint32_t drained = 0u;
            while (!g_logging_ring.empty() && drained < 16u) {
                auto opt_tlp = g_logging_ring.pop();
                if (!opt_tlp.has_value()) { break; }
                const auto& tlp = opt_tlp.value();
                g_msp_server.transport().send(
                    std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(&tlp.wire), sizeof(TlpWire64)));
                drained++;
            }
        }

        ::sleep_ms(1u);
    }

    return 0;
}

#endif // PICO_BOARD
