/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - `pcie-clean` Flight Engine, Config Persistence & MSP TCP Server
 */

#include "coroutine_task.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include "msp_protocol.hpp"
#include "msp_server.hpp"
#include "blackbox_logger.hpp"
#include "target_interface.hpp"
#include "pid.hpp"
#include "attitude.hpp"
#include "navigation.hpp"
#include "mixer.hpp"
#include "ekf3.hpp"
#include "imu_pcie_driver.hpp"
#include "esc_dshot_driver.hpp"
#include "hardware_simulator.hpp"
#include "linux_rt_hardener.hpp"
#include <iostream>
#include <cassert>

using namespace abstractx;

// Global SPSC Telemetry & CTF Logging Rings
SpscTlpRing<64> g_telemetry_ring;
SpscTlpRing<64> g_logging_ring;

// Hardware Simulator Instance
sitl::HardwareSimulator g_hw_sim;

// Flight Control & EKF3 State Estimator Modules
flight::Ekf3Filter g_ekf3;
flight::PidController g_pid_controller;
flight::NavigationEngine g_nav_engine;
flight::Mixer<4> g_quad_mixer(flight::presets::QuadX);

// Configurable MSP TCP Server (Listens on TCP 5760 for iNav Configurator)
msp::MspServer g_msp_server{msp::MspServerConfig{msp::TransportMode::Tcp, 5760, 115200}};

// Coroutine running the full flight loop with EKF3 & Zero-Alloc BareCTF Binary Logging
Task<void> run_flight_loop() {
    logging::BlackboxLogger::set_level(logging::LogLevel::FlightData);
    logging::BlackboxLogger::log_info(g_logging_ring, "Starting EKF3 Multi-Sensor Flight Loop", 1000000000ULL);

    g_ekf3.reset();
    g_nav_engine.set_home(37.7749f, -122.4194f, 1000.0f);
    g_nav_engine.set_mode(flight::NavMode::ReturnToHome);

    for (uint32_t step = 0; step < 10; ++step) {
        // Step Hardware Simulator (1 ms timestep)
        g_hw_sim.step(1000, g_telemetry_ring);

        // Poll Configurable MSP TCP Server
        g_msp_server.poll();

        // Consume TLP Telemetry Stream
        while (!g_telemetry_ring.empty()) {
            auto opt_tlp = g_telemetry_ring.pop();
            if (opt_tlp.has_value()) {
                const auto& tlp = opt_tlp.value();

                // Unpack IMU burst from 64B TLP
                drivers::ImuSample imu = drivers::ImuPcieDriver::parse_tlp(tlp);

                // Run EKF3 Time Predict Step (64-bit nanosecond hardware timestamps)
                g_ekf3.predict_imu(imu);

                // Simulate periodic Baro & GPS corrections
                if (step % 5 == 0) {
                    g_ekf3.correct_baro(1000.0f, imu.timestamp_ns);
                    g_ekf3.correct_gps(37.7740f, -122.4180f, 1000.0f, {0.0f, 0.0f, 0.0f}, imu.timestamp_ns);
                }

                // Get EKF3 State Estimation
                const auto& ekf_state = g_ekf3.state();

                // Run Navigation Controller
                std::array<float, 3> nav_vel = g_nav_engine.update(37.7740f, -122.4180f, 500.0f);
                (void)nav_vel;

                // Run PID Controller
                std::array<float, 3> target_rates{0.0f, 0.0f, 0.0f};
                flight::PidState pid_out = g_pid_controller.update(target_rates, imu.gyro_deg_s, 0.001f);

                // Mix Quadcopter Motors using C++20 QuadX Mixer
                auto motors = g_quad_mixer.mix(0.5f, pid_out);
                Tlp64 dshot_tlp = drivers::EscDshotDriver::make_motor_write_tlp(0, motors[0], static_cast<uint8_t>(step));
                g_hw_sim.handle_mem_write(dshot_tlp.target_address(), motors[0]);

                // Log BareCTF Binary Trace Packet
                logging::CtfFlightEvent ctf_evt{};
                ctf_evt.roll_deg_x10 = static_cast<int16_t>(ekf_state.attitude.roll_deg * 10.0f);
                ctf_evt.pitch_deg_x10 = static_cast<int16_t>(ekf_state.attitude.pitch_deg * 10.0f);
                ctf_evt.yaw_deg = static_cast<uint16_t>(ekf_state.attitude.yaw_deg);
                ctf_evt.motor1 = motors[0];

                logging::BlackboxLogger::log_ctf_event(g_logging_ring, imu.timestamp_ns, ctf_evt);
            }
        }

        co_await std::suspend_always{};
    }

    co_return;
}

int main() {
#if defined(__linux__)
    // 0. Harden Linux Real-Time Environment (Pin to Isolated CPU 3, SCHED_FIFO Priority 99, mlockall RAM)
    target::linux_rt::LinuxRtHardener::harden_realtime_thread(3, 99);
#endif

    // 1. Initialize Zero-Linker Configuration Registry from config.bin (or auto-create config.bin with defaults)
    bool config_loaded = ConfigRegistry::load_from_file("config.bin");
    assert(config_loaded && "Failed to load/create config.bin");
    assert(ConfigRegistry::verify_magic() && "Config magic check failed");

    // 2. Start Configurable MSP TCP Server for iNav Configurator (TCP 5760)
    assert(g_msp_server.start() && "MSP server failed to start");

    // 3. Initialize Hardware Simulator & Enable IMU
    g_hw_sim.handle_mem_write(bar::ImuBase + reg::imu::Control, 0x01);

    // 4. Run Flight Loop Coroutine
    Task<void> flight_task = run_flight_loop();
    while (!flight_task.done()) {
        flight_task.resume();
    }

    // 5. Verify CTF Binary Trace Log Packets in SPSC Ring
    size_t log_count = 0;
    while (!g_logging_ring.empty()) {
        auto opt_log = g_logging_ring.pop();
        if (opt_log.has_value()) {
            log_count++;
        }
    }
    assert(log_count > 0 && "Zero CTF log records generated");

    g_msp_server.stop();
    return 0;
}
