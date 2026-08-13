/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - `pcie-clean` Complete Flight Engine & Hardware Simulator
 */

#include "coroutine_task.hpp"
#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "config_registry.hpp"
#include "msp_protocol.hpp"
#include "pid.hpp"
#include "attitude.hpp"
#include "navigation.hpp"
#include "imu_pcie_driver.hpp"
#include "esc_dshot_driver.hpp"
#include "hardware_simulator.hpp"
#include <iostream>
#include <cassert>

using namespace abstractx;

// Global SPSC Telemetry Ring
SpscTlpRing<64> g_telemetry_ring;

// Hardware Simulator Instance
sitl::HardwareSimulator g_hw_sim;

// Flight Control Modules
flight::AttitudeFilter g_attitude_filter;
flight::PidController g_pid_controller;
flight::NavigationEngine g_nav_engine;

// Coroutine running the full flight loop
Task<void> run_flight_loop() {
    std::cout << "\n[Flight Loop] Starting Zero-Alloc C++20 Flight Control Loop..." << std::endl;

    // Set RTH Home Position
    g_nav_engine.set_home(37.7749f, -122.4194f, 1000.0f);
    g_nav_engine.set_mode(flight::NavMode::ReturnToHome);

    for (uint32_t step = 0; step < 10; ++step) {
        // Step Linux Hardware Simulator (1 ms / 1000 us timestep)
        g_hw_sim.step(1000, g_telemetry_ring);

        // Consume TLP Telemetry Stream
        while (!g_telemetry_ring.empty()) {
            auto opt_tlp = g_telemetry_ring.pop();
            if (opt_tlp.has_value()) {
                const auto& tlp = opt_tlp.value();

                // Driver parses 14B IMU burst from 64B TLP
                drivers::ImuSample imu = drivers::ImuPcieDriver::parse_tlp(tlp);

                // Run Attitude Sensor Fusion
                flight::AttitudeAngles angles = g_attitude_filter.update(imu.accel_g, imu.gyro_deg_s, 0.001f);

                // Run Navigation RTH Controller
                std::array<float, 3> nav_target_vel = g_nav_engine.update(37.7740f, -122.4180f, 500.0f);

                // Run PID Controller
                std::array<float, 3> target_rates{0.0f, 0.0f, 0.0f};
                flight::PidState pid_out = g_pid_controller.update(target_rates, imu.gyro_deg_s, 0.001f);

                // Dispatch ESC DShot Command over PCIe BAR
                uint16_t motor1_cmd = static_cast<uint16_t>(1000.0f + std::abs(pid_out.total_out[0]) * 100.0f);
                Tlp64 dshot_tlp = drivers::EscDshotDriver::make_motor_write_tlp(0, motor1_cmd, static_cast<uint8_t>(step));
                g_hw_sim.handle_mem_write(dshot_tlp.target_address(), motor1_cmd);

                std::cout << "  [Step " << step << "] HW Time=" << imu.timestamp_ns << " ns"
                          << " | Roll=" << angles.roll_deg << " deg"
                          << " | Nav RTH Target Vel X=" << nav_target_vel[0] << " cm/s"
                          << " -> Motor1=" << motor1_cmd << std::endl;
            }
        }

        co_await std::suspend_always{};
    }

    co_return;
}

int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  tcmichals/inav (pcie-clean): The Fusion of iNav Nav & Betaflight" << std::endl;
    std::cout << "=================================================================" << std::endl;

    // 1. Initialize Zero-Linker Configuration Registry
    ConfigRegistry::reset_defaults();
    assert(ConfigRegistry::verify_magic() && "Config magic verification failed");
    std::cout << "[PASS] C++20 Zero-Linker Configuration Registry active (Zero .ld hacks)" << std::endl;

    // 2. Initialize Hardware Simulator
    g_hw_sim.handle_mem_write(bar::ImuBase + reg::imu::Control, 0x01); // Enable IMU Auto-DMA

    // 3. Execute Flight Loop Coroutine
    Task<void> flight_task = run_flight_loop();
    while (!flight_task.done()) {
        flight_task.resume();
    }

    std::cout << "\n=================================================================" << std::endl;
    std::cout << " [SUCCESS] iNav Navigation & Betaflight Dynamics Flight Engine Live!" << std::endl;
    std::cout << "=================================================================" << std::endl;

    return 0;
}
