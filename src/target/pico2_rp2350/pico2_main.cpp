/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Pico 2 W (RP2350) Bare-Metal Entry Point
 *
 * Core 0: CYW43439 Wi-Fi / lwIP MSP Server (Port 5760) / Flash Sector 0x1F0000 Logging.
 * Core 1: Isolated 1kHz RT Flight Loop (multicore_launch_core1()). Zero OS, zero preemption.
 */


#if defined(PICO_BOARD)

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "flight_engine_template.hpp"
#include "pico2_target.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include "msp_protocol.hpp"
#include "msp_server.hpp"
#include "lwip_msp_transport.hpp"
#include "icm42688p.hpp"
#include "dps310.hpp"
#include "gps_driver.hpp"

using namespace abstractx;

static SpscTlpRing<64> g_telemetry_ring;
static SpscTlpRing<64> g_logging_ring;

static drivers::gps::GpsDriver g_gps_driver;
static msp::MspServer<msp::LwipMspTransport> g_msp_server{5760};

// ---------------------------------------------------------------------------
// Core 1 Execution Engine — 100% Dedicated 1kHz Real-Time Flight Loop
// ---------------------------------------------------------------------------
void core1_flight_loop_entry() {
    target::pico2::Pico2Target pico_target{};
    target::TargetAdapter<target::pico2::Pico2Target> platform_adapter{pico_target};

    flight::FlightEngine<
        target::TargetAdapter<target::pico2::Pico2Target>,
        drivers::imu::Icm42688P,
        drivers::baro::Dps310,
        drivers::gps::GpsDriver,
        4
    > pico_engine{platform_adapter, g_gps_driver};

    auto engine_task = pico_engine.run_loop(g_telemetry_ring, g_logging_ring);

    while (true) {
        engine_task.resume();
        sleep_us(1000); // 1kHz loop tick
    }
}

// ---------------------------------------------------------------------------
// Core 0 Entry Point — Peripherals, Wi-Fi, lwIP TCP Server & Flash Storage
// ---------------------------------------------------------------------------
int main() {
    stdio_init_all();

    // 1. Initialize CYW43439 Wi-Fi chip & lwIP stack
    if (cyw43_arch_init()) {
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    // 2. Load Configuration from Flash Sector 0x1F0000
    storage::FlashStorageAdapter flash{storage::FlashMediumType::Pico2OnChip};

    ConfigRegistry::load_from_storage(flash);

    // 3. Start lwIP raw API MSP TCP Server on port 5760
    g_msp_server.start();

    // 4. Launch Core 1 Dedicated Real-Time Flight Engine
    multicore_launch_core1(core1_flight_loop_entry);

    // 5. Core 0 Background Loop (Polling lwIP Wi-Fi timers)
    while (true) {
        cyw43_arch_poll();
        g_msp_server.poll();
        sleep_ms(1);
    }

    return 0;
}

#endif // PICO_BOARD
