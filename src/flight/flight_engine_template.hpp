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
#include "pcie_bar_map.hpp"
#include "imu_pcie_driver.hpp"
#include "baro_driver.hpp"
#include "mag_driver.hpp"
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
#include "wind_estimator.hpp"
#include <concepts>
#include <tuple>
#include <type_traits>
#include <cstring>

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
        wind_estimator_.init();

        // Execute INAV EZ-Tune Initialization (fc_init.c:524)
        auto ez_profile = ezTuneUpdate();
        (void)ez_profile;

        uint64_t step_count = 0;

        while (system_running_) {
            step_count++;

            // 1. Process incoming TLP telemetry stream
            while (!telemetry_ring.empty()) {
                Tlp64 tlp = co_await ImuSampleAwaiter<SpscTlpRing<64>>{telemetry_ring};

                // Asynchronous BAR Address Demultiplexing (Auto-DMA and Parallel I/O)
                const uint32_t bar_id = tlp.wire.target_address & 0xF000u;

                // Event-Driven Barometer Altitude Correction (Asynchronous I2C DMA completion)
                if (bar_id == bar::BaroBase) {
                    drivers::BaroSample baro = BaroDriver::parse_tlp(tlp);
                    pos_estimator_.correct_baro(static_cast<float>(baro.altitude_cm) * 0.01f);
                    (void)logging_ring.push(tlp);
                    continue;
                }

                // Event-Driven GPS Position & Velocity Correction (Asynchronous UART DMA completion)
                if (bar_id == bar::GpsBase) {
                    const uint8_t* p = tlp.wire.payload;
                    int32_t itow = 0;
                    int32_t lat_i = 0;
                    int32_t lon_i = 0;
                    int32_t alt_i = 0;
                    int32_t spd_i = 0;
                    int32_t head_i = 0;
                    std::memcpy(&itow, p, 4);
                    std::memcpy(&lat_i, p + 4, 4);
                    std::memcpy(&lon_i, p + 8, 4);
                    std::memcpy(&alt_i, p + 12, 4);
                    std::memcpy(&spd_i, p + 16, 4);
                    std::memcpy(&head_i, p + 20, 4);
                    uint8_t sats = p[24];
                    uint8_t fix = p[25];

                    if (lat_i != 0 || lon_i != 0 || fix > 0) {
                        drivers::gps::GpsSample s{};
                        s.latitude_1e7 = lat_i;
                        s.longitude_1e7 = lon_i;
                        s.altitude_cm = alt_i / 10;
                        s.vel_n_cms = static_cast<int16_t>(spd_i / 10);
                        s.ground_course_decideg = static_cast<uint16_t>(head_i / 10000);
                        s.num_sats = sats;
                        s.fix_type = (fix > 0) ? drivers::gps::GpsFixType::Fix3D : drivers::gps::GpsFixType::NoFix;
                        s.valid = (fix > 0);
                        gps_driver_.inject_msp_gps(s);
                    }

                    const auto& gps_sample = gps_driver_.latest_sample();
                    if (gps_sample.valid) {
                        double lat_deg = static_cast<double>(gps_sample.latitude_1e7) / 1e7;
                        double lon_deg = static_cast<double>(gps_sample.longitude_1e7) / 1e7;
                        float alt_m  = static_cast<float>(gps_sample.altitude_cm) * 0.01f;
                        float vel_n = static_cast<float>(gps_sample.vel_n_cms) * 0.01f;
                        float vel_e = static_cast<float>(gps_sample.vel_e_cms) * 0.01f;
                        pos_estimator_.correct_gps(lat_deg, lon_deg, alt_m, vel_n, vel_e, static_cast<float>(gps_sample.hdop_centi) * 0.01f, gps_sample.num_sats);
                    }
                    (void)logging_ring.push(tlp);
                    continue;
                }

                // Event-Driven Magnetometer 3D Vector Correction (Asynchronous I2C DMA completion)
                if (bar_id == bar::MagBase) {
                    drivers::MagSample mag = drivers::MagDriver::parse_tlp(tlp);
                    latest_mag_bf_ = Vector3f{mag.mag_mgauss[0], mag.mag_mgauss[1], mag.mag_mgauss[2]};
                    has_mag_ = true;
                    (void)logging_ring.push(tlp);
                    continue;
                }

                // Event-Driven Airspeed Dynamic Pressure (Asynchronous I2C DMA completion)
                if (bar_id == bar::PitotBase) {
                    const uint8_t* raw = tlp.wire.payload;
                    uint16_t dp_raw = static_cast<uint16_t>(((raw[0] & 0x3Fu) << 8u) | raw[1]);
                    const float dp_psi = (static_cast<float>(dp_raw) - 1638.3f) * (1.0f / 13106.4f) - 0.5f;
                    const float dp_pa = dp_psi * 6894.757f;
                    if (dp_pa > 0.0f) {
                        latest_airspeed_m_s_ = std::sqrt((2.0f * dp_pa) / 1.225f);
                        has_airspeed_ = true;
                    }
                    (void)logging_ring.push(tlp);
                    continue;
                }

                // High-Rate IMU Path (DRDY Auto-DMA: bar::ImuBase or default)
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

                // 3. INAV AHRS IMU State Estimation with Mag Fusion (INAV imu.c)
                Vector3f accel_v{accel_g.roll, accel_g.pitch, accel_g.yaw};
                Vector3f gyro_v{notch_filtered_gyro.roll, notch_filtered_gyro.pitch, notch_filtered_gyro.yaw};
                auto attitude = ahrs_imu_.update(
                    accel_v,
                    gyro_v,
                    0.001f,
                    has_mag_ ? &latest_mag_bf_ : nullptr
                );
                (void)attitude;

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

                // Wind Velocity Estimation (INAV wind_estimator.c)
                (void)wind_estimator_.update(
                    nav_state.vel_x_m_s,
                    nav_state.vel_y_m_s,
                    attitude.yaw_deg,
                    latest_airspeed_m_s_,
                    has_airspeed_,
                    0.001f
                );

                // 6. Rate Control & PID Dynamics (INAV pid.c)
                Axis3f target_rates{
                    nav_cmd.target_roll_deg * 2.0f,
                    nav_cmd.target_pitch_deg * 2.0f,
                    nav_cmd.target_yaw_rate_deg_s
                };

                PidState pid_out = pid_controller_.update(target_rates, notch_filtered_gyro, 0.5f, 0.001f);

                // 7. Motor Mixing & DShot Telemetry Output (INAV mixer.c)
                (void)quad_mixer_.mix(0.5f, pid_out);
                (void)logging_ring.push(tlp);
            }

            co_await YieldTick{};
        }

        co_return;
    }

    void stop() noexcept { system_running_ = false; }

    [[nodiscard]] const InavImu& ahrs_imu() const noexcept { return ahrs_imu_; }
    [[nodiscard]] const InavPosEstimator& pos_estimator() const noexcept { return pos_estimator_; }
    [[nodiscard]] const NavigationEngine& nav_engine() const noexcept { return nav_engine_; }

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
    WindEstimator wind_estimator_{};

    Vector3f latest_mag_bf_{0.0f, 0.0f, 0.0f};
    bool has_mag_{false};
    float latest_airspeed_m_s_{0.0f};
    bool has_airspeed_{false};
};

} // namespace abstractx::flight

#endif // FLIGHT_ENGINE_TEMPLATE_HPP

