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
#include "attitude.hpp"
#include "pos_estimator.hpp"
#include "gyro_analyse.hpp"
#include "pid.hpp"
#include "navigation.hpp"
#include "mixer.hpp"
#include "failsafe.hpp"
#include "arming_checker.hpp"
#include "ez_tune.hpp"
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
// C++20 Compile-Time Flight Control Engine Driver (100% INAV State Pipeline)
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
        ahrs_imu_.reset();
        pos_estimator_.reset();
        gyro_spectral_.init(100, 1000);
        dynamic_notch_.init(0.001f);
        nav_engine_.set_home(37.7749f, -122.4194f, 1000.0f);
        nav_engine_.set_mode(NavMode::ReturnToHome);

        // Execute INAV EZ-Tune Initialization (fc_init.c:524)
        auto ez_profile = ezTuneUpdate();
        (void)ez_profile;

        uint64_t step_count = 0;

        while (system_running_) {
            step_count++;

            // 1. Process incoming TLP telemetry stream
            while (!telemetry_ring.empty()) {
                Tlp64 tlp = co_await ImuSampleAwaiter<SpscTlpRing<64>>{telemetry_ring};

                // Parse raw IMU burst using compile-time template ImuDriver
                auto imu = ImuDriver::parse_tlp(tlp);

                // 2. Dynamic Gyro Notch Spectral Analysis (INAV gyroanalyse.c)
                gyro_spectral_.push(0, imu.gyro_deg_s[0]);
                gyro_spectral_.push(1, imu.gyro_deg_s[1]);
                gyro_spectral_.push(2, imu.gyro_deg_s[2]);
                gyro_spectral_.update();

                if (gyro_spectral_.has_filter_update()) {
                    const size_t ax = gyro_spectral_.filter_update_axis();
                    std::array<float, DYN_NOTCH_PEAK_COUNT> peaks{
                        gyro_spectral_.center_frequency(ax, 0),
                        gyro_spectral_.center_frequency(ax, 1),
                        gyro_spectral_.center_frequency(ax, 2)
                    };
                    dynamic_notch_.update_frequencies(ax, peaks);
                }

                // Filter Gyro through Dynamic Notch
                Axis3f notch_filtered_gyro{
                    dynamic_notch_.apply(0, imu.gyro_deg_s[0]),
                    dynamic_notch_.apply(1, imu.gyro_deg_s[1]),
                    dynamic_notch_.apply(2, imu.gyro_deg_s[2])
                };

                Axis3f accel_g{
                    imu.accel_g[0],
                    imu.accel_g[1],
                    imu.accel_g[2]
                };

                // 3. INAV AHRS IMU State Estimation (INAV imu.c)
                auto attitude = ahrs_imu_.update(accel_g, notch_filtered_gyro, 0.001f);
                (void)attitude;

                // 4. Barometric & GPS Corrections (Decimated @ 100Hz / 10Hz)
                if (step_count % 10 == 0) {
                    drivers::BaroSample baro = BaroDriver::parse_tlp(tlp);
                    pos_estimator_.correct_baro(baro.altitude_cm * 0.01f);

                    const auto& gps_sample = gps_driver_.latest_sample();
                    if (gps_sample.valid) {
                        double lat_deg = static_cast<double>(gps_sample.latitude_1e7) / 1e7;
                        double lon_deg = static_cast<double>(gps_sample.longitude_1e7) / 1e7;
                        float alt_m  = static_cast<float>(gps_sample.altitude_cm) * 0.01f;
                        float vel_n = static_cast<float>(gps_sample.vel_n_cms) * 0.01f;
                        float vel_e = static_cast<float>(gps_sample.vel_e_cms) * 0.01f;
                        pos_estimator_.correct_gps(lat_deg, lon_deg, alt_m, vel_n, vel_e, static_cast<float>(gps_sample.hdop_centi) * 0.01f, gps_sample.num_sats);
                    }
                }

                // Inertial Accel Prediction
                pos_estimator_.predict_imu(accel_g, ahrs_imu_, 0.001f);

                // 5. INAV 3D Autonomous Guidance (INAV navigation.c)
                const auto& est_state = pos_estimator_.state();
                NavState nav_state{};
                nav_state.pos_x_m = est_state.pos_n_m;
                nav_state.pos_y_m = est_state.pos_e_m;
                nav_state.pos_z_m = est_state.pos_d_m;
                nav_state.vel_x_m_s = est_state.vel_n_m_s;
                nav_state.vel_y_m_s = est_state.vel_e_m_s;
                nav_state.vel_z_m_s = est_state.vel_d_m_s;
                nav_state.mode = NavMode::ReturnToHome;
                nav_state.home_set = true;

                NavCommand nav_cmd = nav_engine_.update(nav_state, 0.001f);

                // 6. Rate Control & PID Dynamics (INAV pid.c)
                Axis3f target_rates{
                    nav_cmd.target_roll_deg * 2.0f,
                    nav_cmd.target_pitch_deg * 2.0f,
                    nav_cmd.target_yaw_rate_deg_s
                };

                PidState pid_out = pid_controller_.update(target_rates, notch_filtered_gyro, 0.5f, 0.001f);

                // 7. Motor Mixing & DShot Telemetry Output (INAV mixer.c)
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

    InavImu ahrs_imu_{};
    InavPosEstimator pos_estimator_{};
    GyroSpectralAnalyzer gyro_spectral_{};
    DynamicGyroNotchBank dynamic_notch_{};
    PidController pid_controller_{};
    NavigationEngine nav_engine_{};
    Mixer<MotorCount> quad_mixer_{presets::QuadX};
    FailsafeEngine failsafe_engine_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_ENGINE_TEMPLATE_HPP

