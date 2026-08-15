/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Linux SITL Desktop Entry Point
 *
 * Runs the hardware simulator, Boost.Asio MSP TCP server (port 5760),
 * EKF3 fusion, PID controller, 3D RTH navigation, and background blackbox logger.
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "posix_tcp_transport.hpp"
#include "coroutine_task.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include "msp_protocol.hpp"
#include "msp_server.hpp"
#include "msp_transport.hpp"
#include "blackbox_logger.hpp"
#include "target_interface.hpp"
#include "pid.hpp"
#include "attitude.hpp"
#include "navigation.hpp"
#include "mixer.hpp"
#include "ekf3.hpp"
#include "failsafe.hpp"
#include "arming_checker.hpp"
#include "imu_pcie_driver.hpp"
#include "esc_dshot_driver.hpp"
#include "dps310.hpp"
#include "gps_driver.hpp"
#include "hardware_simulator.hpp"
#include "linux_rt_hardener.hpp"

using namespace abstractx;

// Global System State & SPSC Rings
static std::atomic<bool> g_system_running{true};
static SpscTlpRing<64> g_telemetry_ring;
static SpscTlpRing<64> g_logging_ring;

// SITL Peripherals & GPS Driver
static sitl::HardwareSimulator g_hw_sim;
static drivers::gps::GpsDriver g_gps_driver;

// Flight Control Engine Modules
static flight::Ekf3Filter g_ekf3;
static flight::PidController g_pid_controller;
static flight::NavigationEngine g_nav_engine;
static flight::Mixer<4> g_quad_mixer(flight::presets::QuadX);
static flight::FailsafeEngine g_failsafe_engine;

// POSIX TCP MSP Server & Blackbox Logger
static msp::MspServer<msp::PosixTcpTransport> g_msp_server{5760};
static logging::BlackboxLogger g_blackbox_logger;
static msp::MspLiveState g_msp_live_state;

static void signal_handler(int /*sig*/) {
    g_system_running.store(false, std::memory_order_relaxed);
}

static void update_msp_live_state() {
    const auto& ekf_state = g_ekf3.state();
    g_msp_live_state.roll_decideg  = static_cast<int16_t>(ekf_state.attitude.roll_deg * 10.0f);
    g_msp_live_state.pitch_decideg = static_cast<int16_t>(ekf_state.attitude.pitch_deg * 10.0f);
    g_msp_live_state.yaw_deg       = static_cast<int16_t>(ekf_state.attitude.yaw_deg);
    g_msp_live_state.altitude_cm   = static_cast<int32_t>(ekf_state.pos_ned_cm[2]);
    g_msp_live_state.vario_cms     = static_cast<int16_t>(ekf_state.vel_ned_cms[2]);
    g_msp_live_state.sensor_flags  = 0x31; // ACC | BARO | GPS
    g_msp_live_state.cycle_time_us = 1000;
}

// ---------------------------------------------------------------------------
// Concurrent Sensor Coroutine Tasks
// ---------------------------------------------------------------------------

// Concurrent Coroutine Task 1: Barometer Sensor Processing
static Task<void> baro_sensor_task(const Tlp64& tlp) {
    drivers::BaroSample baro = co_await BaroSampleAwaiter<drivers::baro::Dps310, Tlp64>{tlp, true};
    g_ekf3.correct_baro(baro.altitude_cm, baro.timestamp_ns);
    co_return;
}

// Concurrent Coroutine Task 2: GPS Sensor Processing
static Task<void> gps_sensor_task() {
    drivers::gps::GpsSample gps_sample = co_await GpsSampleAwaiter<drivers::gps::GpsDriver>{g_gps_driver};
    float lat_deg = static_cast<float>(gps_sample.latitude_1e7) / 1e7f;
    float lon_deg = static_cast<float>(gps_sample.longitude_1e7) / 1e7f;
    float alt_cm  = static_cast<float>(gps_sample.altitude_cm);
    std::array<float, 3> vel_ned_cms{
        static_cast<float>(gps_sample.vel_n_cms),
        static_cast<float>(gps_sample.vel_e_cms),
        static_cast<float>(gps_sample.vel_d_cms)
    };
    g_ekf3.correct_gps(lat_deg, lon_deg, alt_cm * 0.01f, vel_ned_cms[0] * 0.01f, vel_ned_cms[1] * 0.01f, static_cast<float>(gps_sample.hdop_centi) * 0.01f, gps_sample.num_sats);
    co_return;

}

// Main Flight Engine Coroutine: Orchestrates concurrent sensor tasks
static Task<void> run_flight_loop() {
    g_ekf3.reset();
    g_nav_engine.set_home(37.7749f, -122.4194f, 1000.0f);
    g_nav_engine.set_mode(flight::NavMode::ReturnToHome);

    uint64_t step_count = 0;
    while (g_system_running.load(std::memory_order_relaxed)) {
        step_count++;

        // 1. Step Hardware Simulator
        g_hw_sim.step(1000, g_telemetry_ring);

        // 2. Event-driven coroutine execution awaiting TLP telemetry stream
        while (!g_telemetry_ring.empty()) {
            Tlp64 tlp = co_await ImuSampleAwaiter<SpscTlpRing<64>>{g_telemetry_ring};

            drivers::ImuSample imu = drivers::ImuPcieDriver::parse_tlp(tlp);
            g_ekf3.predict_imu(imu);

            // 3. Dispatch & resume concurrent sensor coroutines asynchronously
            if (step_count % 5 == 0) {
                auto b_task = baro_sensor_task(tlp);
                b_task.resume();

                auto g_task = gps_sensor_task();
                g_task.resume();
            }

            // 4. Run Navigation Engine on EKF3 estimated position
            const auto& ekf_state = g_ekf3.state();
            flight::NavState nav_state{};
            nav_state.pos_x_m = ekf_state.pos_ned_cm[0] / 100.0f;
            nav_state.pos_y_m = ekf_state.pos_ned_cm[1] / 100.0f;
            nav_state.pos_z_m = ekf_state.pos_ned_cm[2] / 100.0f;
            nav_state.mode = flight::NavMode::ReturnToHome;
            nav_state.home_set = true;

            flight::NavCommand nav_cmd = g_nav_engine.update(nav_state, 0.001f);

            // 5. Feed Navigation target rates into PID Controller
            flight::Axis3f target_rates{
                nav_cmd.target_roll_deg * 2.0f,
                nav_cmd.target_pitch_deg * 2.0f,
                nav_cmd.target_yaw_rate_deg_s
            };
            flight::Axis3f gyro_rates{
                imu.gyro_deg_s[0],
                imu.gyro_deg_s[1],
                imu.gyro_deg_s[2]
            };
            flight::PidState pid_out = g_pid_controller.update(target_rates, gyro_rates, 0.5f, 0.001f);


            auto motors = g_quad_mixer.mix(0.5f, pid_out);
            for (size_t i = 0; i < 4; ++i) {
                g_hw_sim.set_motor(i, motors[i]);
            }


            update_msp_live_state();
            g_msp_server.update_live_state(g_msp_live_state);

            (void)g_logging_ring.push(tlp);
        }

        co_await YieldTick{};
    }

    co_return;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Harden Linux Real-Time Environment (Isolated Core 3, SCHED_FIFO 99)
    target::sitl::LinuxRtHardener::harden_single_realtime_core(3);

    // 1. Config Registry Init
    bool config_loaded = ConfigRegistry::load_from_file("config.bin");
    if (!config_loaded) {
        std::cerr << "Warning: Created default config.bin\n";
    }

    // 2. Start MSP TCP Server (TCP 5760)
    if (!g_msp_server.start()) {
        std::cerr << "Error: Failed to start MSP server on TCP 5760\n";
        return 1;
    }

    // 3. Start Blackbox Disk Logger Thread (Core 0)
    g_blackbox_logger.start(g_logging_ring, "blackbox");

    // 4. Initialize Hardware Simulator & Enable IMU
    g_hw_sim.handle_mem_write(bar::ImuBase + reg::imu::Control, 0x01);

    std::cout << "SITL Flight Engine Running (Linux Desktop) — TCP 5760 — Press Ctrl+C to Stop.\n";

    // 5. Run Flight Loop Coroutine
    Task<void> flight_task = run_flight_loop();
    while (g_system_running.load(std::memory_order_relaxed) && !flight_task.done()) {
        flight_task.resume();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 6. Clean Shutdown
    g_msp_server.stop();
    g_blackbox_logger.stop();

    std::cout << "SITL Flight Engine Stopped Cleanly. Logged TLPs: "
              << g_blackbox_logger.total_tlps_written() << "\n";
    return 0;
}
