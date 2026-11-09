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
#include "tlp_ring.hpp"
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

static void update_msp_live_state(bool is_armed, const std::array<uint16_t, 4>& motors, const std::array<uint16_t, 16>& rc) {
    const auto& ekf_state = g_ekf3.state();
    g_msp_live_state.roll_decideg  = static_cast<int16_t>(ekf_state.attitude.roll_deg * 10.0f);
    g_msp_live_state.pitch_decideg = static_cast<int16_t>(ekf_state.attitude.pitch_deg * 10.0f);
    g_msp_live_state.yaw_deg       = static_cast<int16_t>(ekf_state.attitude.yaw_deg);
    g_msp_live_state.altitude_cm   = static_cast<int32_t>(g_hw_sim.altitude_m() * 100.0f);
    g_msp_live_state.vario_cms     = static_cast<int16_t>(g_hw_sim.vario_m_s() * 100.0f);
    g_msp_live_state.sensor_flags  = 0x31; // ACC | BARO | GPS
    g_msp_live_state.cycle_time_us = 1000;
    g_msp_live_state.arming_flags  = is_armed ? 1 : 0;
    for (size_t i = 0; i < 4; ++i) {
        g_msp_live_state.motor_us[i] = motors[i];
    }
    g_msp_live_state.rc_channels = rc;
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

            // 4. Poll MSP Server for live RC stick inputs
            const auto rc = g_msp_server.rc_channels();
            const bool is_armed = (rc[4] > 1700);

            const auto& ekf_state = g_ekf3.state();
            std::array<uint16_t, 4> motors{1000, 1000, 1000, 1000};

            if (is_armed) {
                // Throttle: 1000..2000 µs mapped to 0.0..1.0f
                float throttle = 0.0f;
                const float raw_th = static_cast<float>(rc[2]);
                if (raw_th > 1050.0f) {
                    throttle = std::clamp((raw_th - 1000.0f) / 1000.0f, 0.0f, 1.0f);
                }

                flight::Axis3f target_rates{0.0f, 0.0f, 0.0f};
                const uint16_t mode_ch = rc[5];

                if (mode_ch > 1700) {
                    // Flight Mode: Autonomous Return-to-Home (RTH) / Waypoint Navigation
                    flight::NavState nav_state{};
                    nav_state.pos_x_m = ekf_state.pos_ned_cm[0] / 100.0f;
                    nav_state.pos_y_m = ekf_state.pos_ned_cm[1] / 100.0f;
                    nav_state.pos_z_m = ekf_state.pos_ned_cm[2] / 100.0f;
                    nav_state.mode = flight::NavMode::ReturnToHome;
                    nav_state.home_set = true;

                    flight::NavCommand nav_cmd = g_nav_engine.update(nav_state, 0.001f);
                    target_rates = flight::Axis3f{
                        nav_cmd.target_roll_deg * 2.0f,
                        nav_cmd.target_pitch_deg * 2.0f,
                        nav_cmd.target_yaw_rate_deg_s
                    };
                    if (throttle == 0.0f) {
                        throttle = 0.5f; // Auto-hover throttle in RTH
                    }
                } else if (mode_ch >= 1300) {
                    // Flight Mode: Angle Mode (Self-Leveling / Stabilized)
                    // Stick mapping: 1000..2000 µs (center 1500) -> ±45° target roll/pitch, ±200°/s yaw
                    const float target_roll_deg  = (static_cast<float>(rc[0]) - 1500.0f) * (45.0f / 500.0f);
                    const float target_pitch_deg = (static_cast<float>(rc[1]) - 1500.0f) * (45.0f / 500.0f);
                    const float target_yaw_dps   = (static_cast<float>(rc[3]) - 1500.0f) * (200.0f / 500.0f);

                    flight::Axis3f target_angles{target_roll_deg, target_pitch_deg, 0.0f};
                    flight::Axis3f current_angles{
                        ekf_state.attitude.roll_deg,
                        ekf_state.attitude.pitch_deg,
                        ekf_state.attitude.yaw_deg
                    };
                    target_rates = g_pid_controller.calculate_angle_mode_rates(
                        target_angles, current_angles, target_yaw_dps
                    );
                } else {
                    // Flight Mode: Acro / Rate Mode
                    target_rates = flight::Axis3f{
                        (static_cast<float>(rc[0]) - 1500.0f) * (400.0f / 500.0f),
                        (static_cast<float>(rc[1]) - 1500.0f) * (400.0f / 500.0f),
                        (static_cast<float>(rc[3]) - 1500.0f) * (300.0f / 500.0f)
                    };
                }

                // 5. Rate PID Controller
                flight::Axis3f gyro_rates{
                    imu.gyro_deg_s[0],
                    imu.gyro_deg_s[1],
                    imu.gyro_deg_s[2]
                };
                flight::PidState pid_out = g_pid_controller.update(target_rates, gyro_rates, throttle, 0.001f);

                // 6. QuadX Motor Mixing & Actuator Outputs
                motors = g_quad_mixer.mix(throttle, pid_out);
                for (size_t i = 0; i < 4; ++i) {
                    g_hw_sim.set_motor(i, motors[i]);
                }
            } else {
                // Disarmed: Safety cutoff and PID integral reset
                g_pid_controller.reset();
                for (size_t i = 0; i < 4; ++i) {
                    g_hw_sim.set_motor(i, 1000);
                }
            }

            update_msp_live_state(is_armed, motors, rc);
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
