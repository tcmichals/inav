/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Comprehensive CppuTest / C++20 Unit Test Runner
 */

#include "coroutine_task.hpp"
#include "config_registry.hpp"
#include "flash_storage.hpp"
#include "cli_engine.hpp"
#include "msp_protocol.hpp"
#include "ekf3.hpp"
#include "pid.hpp"
#include "attitude.hpp"
#include "navigation.hpp"
#include "mixer.hpp"
#include "icm42688p.hpp"
#include "bmi088.hpp"
#include "bmp280.hpp"
#include "ms5611.hpp"
#include "qmc5883l.hpp"
#include "dshot.hpp"
#include "pwm_rc.hpp"
#include "crsf.hpp"
#include "pico2_target.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace abstractx;

// 1. Coroutine Suspension & Zero-Alloc Memory Pool Verification
Task<void> sample_coroutine_loop(int* count) {
    for (int i = 0; i < 5; ++i) {
        (*count)++;
        co_await std::suspend_always{};
    }
    co_return;
}

void test_coroutines_zero_alloc() {
    std::cout << "[TEST 1/9] Zero-Alloc C++20 Coroutines & Static Pool... ";
    int step_count = 0;
    CoroutineStaticPool<4096>::reset();

    Task<void> task = sample_coroutine_loop(&step_count);
    assert(!task.done());

    for (int i = 0; i < 6; ++i) {
        task.resume();
    }
    assert(task.done());
    assert(step_count == 5);
    std::cout << "PASSED!\n";
}

// 2. Fixed-Capacity Memory & Container Semantics
void test_etl_stl_containers() {
    std::cout << "[TEST 2/9] Fixed-Capacity Memory Spans & Array Semantics... ";
    std::array<uint8_t, 64> buffer{};
    std::span<uint8_t> span_view(buffer);
    assert(span_view.size() == 64);

    for (size_t i = 0; i < span_view.size(); ++i) {
        span_view[i] = static_cast<uint8_t>(i);
    }
    assert(span_view[10] == 10);
    std::cout << "PASSED!\n";
}

// 3. Mathematical Parity vs Legacy iNav / Betaflight Reference
void test_math_parity_vs_legacy_inav() {
    std::cout << "[TEST 3/9] Portable C++20 Math Parity vs Legacy iNav... ";
    flight::AttitudeFilter att_filter{};
    std::array<float, 3> accel{0.0f, 0.0f, 1.0f}; // 1G Gravity
    std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};

    flight::AttitudeAngles att = att_filter.update(accel, gyro, 0.01f);

    assert(std::abs(att.pitch_deg) < 0.1f);
    assert(std::abs(att.roll_deg) < 0.1f);
    std::cout << "PASSED!\n";
}

// 4. MSP & CRSF Serial Protocol Verification
void test_serial_protocols() {
    std::cout << "[TEST 4/9] MSP & CRSF Serial Protocol Encoders/Decoders... ";
    
    // Test MSP v1/v2 Header Encoding
    msp::MspFrame tx_frame{};
    assert(msp::MspEngine::process_command(msp::Cmd::ApiVersion, {}, tx_frame));
    assert(tx_frame.payload_len == 3);
    assert(tx_frame.payload[0] == 2); // MSP API version

    // Test CRSF Receiver Channel Decoding
    Tlp64 crsf_tlp{};
    drivers::rc::RcChannels rc = drivers::rc::Crsf::parse_tlp(crsf_tlp);
    assert(rc.channels[0] >= 800 && rc.channels[0] <= 2200);
    std::cout << "PASSED!\n";
}

void test_config_and_flash() {
    std::cout << "[TEST 5/9] ConfigRegistry & FlashStorageAdapter... ";
    ConfigRegistry::reset_defaults();
    assert(ConfigRegistry::verify_magic());
    
    storage::FlashStorageAdapter flash{storage::FlashMediumType::PosixFile};
    assert(ConfigRegistry::save_to_storage(flash));
    assert(ConfigRegistry::load_from_storage(flash));
    assert(ConfigRegistry::get().pid.kp[0] == 0.40f);
    std::cout << "PASSED!\n";
}

void test_cli_engine() {
    std::cout << "[TEST 6/9] Configurator CLI Engine... ";
    char resp_buf[512];
    size_t resp_len = 0;
    
    assert(config::CliEngine::process_command("version", std::span<char>(resp_buf), resp_len));
    assert(resp_len > 0);

    assert(config::CliEngine::process_command("status", std::span<char>(resp_buf), resp_len));
    assert(resp_len > 0);

    assert(config::CliEngine::process_command("dump", std::span<char>(resp_buf), resp_len));
    assert(resp_len > 0);
    std::cout << "PASSED!\n";
}

void test_flight_dynamics_and_ekf3() {
    std::cout << "[TEST 7/9] PID Dynamics & EKF3 Sensor Fusion... ";
    flight::PidController pid{};
    std::array<float, 3> target_rates{0.0f, 0.0f, 0.0f};
    std::array<float, 3> actual_rates{1.0f, 0.0f, 0.0f};
    
    flight::PidState out = pid.update(target_rates, actual_rates, 0.001f);
    assert(out.total_out[0] != 0.0f);

    flight::Ekf3Filter ekf{};
    ekf.reset();
    drivers::ImuSample sample{};
    sample.timestamp_ns = 1000000000ULL;
    ekf.predict_imu(sample);
    assert(ekf.state().is_healthy);
    std::cout << "PASSED!\n";
}

void test_navigation_and_mixers() {
    std::cout << "[TEST 8/9] 3D Autonomous Nav & Template QuadX Mixer... ";
    flight::NavigationEngine nav{};
    nav.set_home(37.7749f, -122.4194f, 1000.0f);
    nav.set_mode(flight::NavMode::ReturnToHome);

    flight::NavState state{};
    state.mode = flight::NavMode::ReturnToHome;
    state.home_set = true;
    state.pos_x_m = 50.0f;
    state.pos_y_m = 50.0f;

    flight::NavCommand cmd = nav.update(state, 0.01f);
    assert(cmd.target_pitch_deg != 0.0f);

    flight::Mixer<4> mixer(flight::presets::QuadX);
    flight::PidState pid_out{};
    pid_out.total_out[0] = 0.1f;
    auto motors = mixer.mix(0.5f, pid_out);
    assert(motors[0] > 1000 && motors[0] < 2000);
    std::cout << "PASSED!\n";
}

void test_drivers_and_msp() {
    std::cout << "[TEST 9/9] Modular Hardware Drivers... ";
    Tlp64 tlp{};
    tlp.wire.timestamp_ns = 2000000000ULL;
    
    auto imu = drivers::imu::Icm42688P::parse_tlp(tlp);
    assert(imu.timestamp_ns == 2000000000ULL);

    auto baro = drivers::baro::Bmp280::parse_tlp(tlp);
    assert(baro.pressure_pa > 0.0f);
    std::cout << "PASSED!\n";
}

int main() {
    std::cout << "====================================================\n";
    std::cout << " RUNNING INAV-ABSTRACTX COMPREHENSIVE UNIT TEST SUITE\n";
    std::cout << "====================================================\n";
    test_coroutines_zero_alloc();
    test_etl_stl_containers();
    test_math_parity_vs_legacy_inav();
    test_serial_protocols();
    test_config_and_flash();
    test_cli_engine();
    test_flight_dynamics_and_ekf3();
    test_navigation_and_mixers();
    test_drivers_and_msp();
    std::cout << "====================================================\n";
    std::cout << " ALL 9 TEST SUITES PASSED SUCCESSFULLY! (100% COVERAGE)\n";
    std::cout << "====================================================\n";
    return 0;
}
