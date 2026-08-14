/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Compile-Time C++20 Templated Flight Control Engine Driver
 *
 * Eliminates preprocessor #ifdefs and virtual function vtables entirely.
 * Uses C++20 Concepts, Variadic Parameter Packs, and Fold Expressions.
 */

#ifndef FLIGHT_ENGINE_TEMPLATE_HPP
#define FLIGHT_ENGINE_TEMPLATE_HPP

#include "coroutine_task.hpp"
#include "spsc_tlp_ring.hpp"
#include "imu_pcie_driver.hpp"
#include "baro_driver.hpp"
#include "gps_types.hpp"
#include "ekf3.hpp"
#include "pid.hpp"
#include "navigation.hpp"
#include "mixer.hpp"
#include "failsafe.hpp"
#include "arming_checker.hpp"
#include <concepts>
#include <tuple>
#include <type_traits>

namespace abstractx::flight {

// ---------------------------------------------------------------------------
// C++20 Concept Definitions
// ---------------------------------------------------------------------------

template <typename T>
concept IsImuDriver = requires(const Tlp64& tlp) {
    { T::parse_tlp(tlp) };
};

template <typename T>
concept IsBaroDriver = requires(const Tlp64& tlp) {
    { T::parse_tlp(tlp) };
};

template <typename T>
concept IsGpsDriver = requires(T driver) {
    { driver.latest_sample() };
};

// ---------------------------------------------------------------------------
// C++20 Compile-Time Flight Control Engine Driver
// ---------------------------------------------------------------------------
template <
    typename TargetPlatform,
    IsImuDriver  ImuDriver,
    IsBaroDriver BaroDriver,
    IsGpsDriver  GpsDriver,
    size_t MotorCount = 4
>
class FlightEngine {
public:
    constexpr FlightEngine(TargetPlatform& target, GpsDriver& gps) noexcept
        : target_(target), gps_driver_(gps) {}

    // Main Coroutine Loop: Executed via C++20 co_await
    Task<void> run_loop(SpscTlpRing<64>& telemetry_ring, SpscTlpRing<64>& logging_ring) {
        ekf3_.reset();
        nav_engine_.set_home(37.7749f, -122.4194f, 1000.0f);
        nav_engine_.set_mode(NavMode::ReturnToHome);

        uint64_t step_count = 0;
        while (system_running_) {
            step_count++;

            // 1. Process incoming TLP telemetry stream
            while (!telemetry_ring.empty()) {
                Tlp64 tlp = co_await ImuSampleAwaiter<SpscTlpRing<64>>{telemetry_ring};

                // Parse raw IMU burst using compile-time template ImuDriver
                auto imu = ImuDriver::parse_tlp(tlp);
                ekf3_.predict_imu(imu);

                // 2. Barometric & GPS Corrections (Decimated @ 100Hz / 10Hz)
                if (step_count % 10 == 0) {
                    drivers::BaroSample baro = BaroDriver::parse_tlp(tlp);
                    ekf3_.correct_baro(baro.altitude_cm);

                    const auto& gps_sample = gps_driver_.latest_sample();
                    if (gps_sample.valid) {
                        double lat_deg = static_cast<double>(gps_sample.latitude_1e7) / 1e7;
                        double lon_deg = static_cast<double>(gps_sample.longitude_1e7) / 1e7;
                        float alt_m  = static_cast<float>(gps_sample.altitude_cm) * 0.01f;
                        float vel_n = static_cast<float>(gps_sample.vel_n_cms) * 0.01f;
                        float vel_e = static_cast<float>(gps_sample.vel_e_cms) * 0.01f;
                        ekf3_.correct_gps(lat_deg, lon_deg, alt_m, vel_n, vel_e, static_cast<float>(gps_sample.hdop_centi) * 0.01f, gps_sample.num_sats);

                    }
                }

                // 3. Autonomous Guidance
                const auto& ekf_state = ekf3_.state();
                NavState nav_state{};
                nav_state.pos_x_m = ekf_state.pos_ned_cm[0] / 100.0f;
                nav_state.pos_y_m = ekf_state.pos_ned_cm[1] / 100.0f;
                nav_state.pos_z_m = ekf_state.pos_ned_cm[2] / 100.0f;
                nav_state.mode = NavMode::ReturnToHome;
                nav_state.home_set = true;

                NavCommand nav_cmd = nav_engine_.update(nav_state, 0.001f);

                // 4. Rate Control & PID Dynamics
                Axis3f target_rates{
                    nav_cmd.target_roll_deg * 2.0f,
                    nav_cmd.target_pitch_deg * 2.0f,
                    nav_cmd.target_yaw_rate_deg_s
                };
                Axis3f gyro_rates{
                    imu.gyro_deg_s[0],
                    imu.gyro_deg_s[1],
                    imu.gyro_deg_s[2]
                };
                PidState pid_out = pid_controller_.update(target_rates, gyro_rates, 0.5f, 0.001f);

                // 5. Motor Mixing & DShot Telemetry Output
                auto motors = quad_mixer_.mix(0.5f, pid_out);
                (void)motors;

                logging_ring.push(tlp);

            }

            co_await YieldTick{};
        }

        co_return;
    }

    void stop() noexcept { system_running_ = false; }

private:
    TargetPlatform& target_;
    GpsDriver& gps_driver_;
    bool system_running_{true};

    Ekf3Filter ekf3_{};
    PidController pid_controller_{};
    NavigationEngine nav_engine_{};
    Mixer<MotorCount> quad_mixer_{presets::QuadX};
    FailsafeEngine failsafe_engine_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_ENGINE_TEMPLATE_HPP
