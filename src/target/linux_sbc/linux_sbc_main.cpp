/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Linux Quad-Core SBC Hardware Entry Point
 *
 * Targeted at Linux Single-Board Computers (e.g. Raspberry Pi 5, Jetson Orin) with PCIe BAR hardware bridge.
 * Pins real-time flight thread to Core 3 (SCHED_FIFO 99) and I/O reactor to Core 2.
 */

#include "flight_engine_template.hpp"
#include "config_registry.hpp"
#include "msp_server.hpp"
#include "posix_tcp_transport.hpp"
#include "blackbox_logger.hpp"
#include "linux_rt_hardener.hpp"
#include "icm42688p.hpp"
#include "dps310.hpp"
#include "gps_driver.hpp"


#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

using namespace abstractx;

static std::atomic<bool> g_system_running{true};
static SpscTlpRing<64> g_telemetry_ring;
static SpscTlpRing<64> g_logging_ring;

static drivers::gps::GpsDriver g_gps_driver;
static msp::MspServer<msp::PosixTcpTransport> g_msp_server{5760};
static logging::BlackboxLogger g_blackbox_logger;

// Minimal Dummy SBC Target for FlightEngine concept
struct LinuxSbcTarget {
    std::string_view name() const noexcept { return "LINUX_SBC"; }
    bool init() noexcept { return true; }
};

static void signal_handler(int /*sig*/) {
    g_system_running.store(false, std::memory_order_relaxed);
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Harden Real-Time Flight Loop on Core 3 (Priority 99, SCHED_FIFO, mlockall)
    target::sitl::LinuxRtHardener::harden_single_realtime_core(3);
    target::sitl::LinuxRtHardener::lock_process_memory();

    // 1. Config Registry Load

    ConfigRegistry::load_from_file("config.bin");

    // 2. Start MSP TCP Server (TCP 5760)
    g_msp_server.start();

    // 3. Start Blackbox Logger Thread (Core 0)
    g_blackbox_logger.start(g_logging_ring, "blackbox");

    std::cout << "Linux SBC Flight Engine Running (PCIe BAR + SCHED_FIFO Core 3).\n";

    LinuxSbcTarget sbc_target{};
    flight::FlightEngine<
        LinuxSbcTarget,
        drivers::imu::Icm42688P,
        drivers::baro::Dps310,
        drivers::gps::GpsDriver,
        4
    > sbc_engine{sbc_target, g_gps_driver};

    auto engine_task = sbc_engine.run_loop(g_telemetry_ring, g_logging_ring);
    
    while (g_system_running.load(std::memory_order_relaxed)) {
        engine_task.resume();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sbc_engine.stop();
    g_msp_server.stop();
    g_blackbox_logger.stop();

    std::cout << "Linux SBC Flight Engine Stopped Cleanly.\n";
    return 0;
}
