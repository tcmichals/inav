/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Unified Synthetic / Fake Sensor Test Harness
 *
 * Generates synthetic hardware sensor feeds for:
 *   1. Fake IMU (3-Axis Gyro & Accel with configurable motor vibration noise)
 *   2. Fake GPS (UBX / NMEA kinematic position, velocity & satellite health)
 *   3. Fake RC Sticks (16-Channel PWM pulses with deadbands & rates)
 *   4. Fake Distance Sensor (Lidar / ToF rangefinder with ground elevation & tilt)
 *   5. Fake Barometer (MS5611 / BMP280 atmospheric pressure & altitude)
 */

#ifndef DRIVERS_FAKE_SENSORS_HPP
#define DRIVERS_FAKE_SENSORS_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include "attitude.hpp"
#include "rangefinder_base.hpp"
#include "fake_rangefinder.hpp"

namespace abstractx::drivers {

struct FakeImuState {
    flight::Axis3f gyro_deg_s{0.0f, 0.0f, 0.0f};
    flight::Axis3f accel_g{0.0f, 0.0f, 1.0f};
};

struct FakeGpsState {
    double lat_deg{37.774900};
    double lon_deg{-122.419400};
    float alt_m{0.0f};
    float vel_n_m_s{0.0f};
    float vel_e_m_s{0.0f};
    float vel_d_m_s{0.0f};
    uint8_t satellites{12};
    float hdop{1.10f};
    bool fix_3d{true};
};

struct FakeRcState {
    std::array<uint16_t, 16> channels{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
                                       1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    bool armed{false};
    bool failsafe{false};
};

class FakeSensorHarness {
public:
    constexpr explicit FakeSensorHarness(double home_lat = 37.774900, double home_lon = -122.419400) noexcept
        : home_lat_(home_lat), home_lon_(home_lon) {}

    // 1. Generate Fake IMU Sample
    FakeImuState update_imu(float roll_rate, float pitch_rate, float yaw_rate,
                           float ax, float ay, float az,
                           float time_s, float motor_noise_freq_hz = 125.0f, float noise_amp = 3.0f) noexcept {
        FakeImuState s{};
        float noise = noise_amp * std::sin(2.0f * 3.14159265f * motor_noise_freq_hz * time_s);
        s.gyro_deg_s = {roll_rate + noise, pitch_rate + (noise * 0.5f), yaw_rate};
        s.accel_g = {ax, ay, az};
        return s;
    }

    // 2. Generate Fake GPS Sample
    FakeGpsState update_gps(float pos_n_m, float pos_e_m, float alt_m, float vn, float ve, float vd) noexcept {
        FakeGpsState s{};
        s.lat_deg = home_lat_ + (static_cast<double>(pos_n_m) / 111139.0);
        s.lon_deg = home_lon_ + (static_cast<double>(pos_e_m) / (111139.0 * std::cos(home_lat_ * 0.0174532925)));
        s.alt_m = alt_m;
        s.vel_n_m_s = vn;
        s.vel_e_m_s = ve;
        s.vel_d_m_s = vd;
        s.satellites = 14;
        s.hdop = 0.95f;
        s.fix_3d = true;
        return s;
    }

    // 3. Generate Fake RC Sticks
    FakeRcState update_rc(uint16_t roll_us, uint16_t pitch_us, uint16_t throttle_us, uint16_t yaw_us, bool armed, bool rth) noexcept {
        FakeRcState s{};
        s.channels[0] = roll_us;
        s.channels[1] = pitch_us;
        s.channels[2] = throttle_us;
        s.channels[3] = yaw_us;
        s.channels[4] = armed ? 1800 : 1000; // Aux 1: Arm
        s.channels[5] = rth ? 1800 : 1000;   // Aux 2: RTH Mode
        s.armed = armed;
        s.failsafe = false;
        return s;
    }

    // 4. Generate Fake Distance Sensor / Rangefinder
    RangefinderSample update_rangefinder(float true_alt_m, float roll_deg, float pitch_deg, uint64_t timestamp_us) noexcept {
        return rangefinder_.update_simulated(true_alt_m, roll_deg, pitch_deg, timestamp_us);
    }

    // 5. Generate Fake Barometer Altitude
    [[nodiscard]] float update_baro(float true_alt_m, float noise_amp = 0.05f, float time_s = 0.0f) const noexcept {
        float noise = noise_amp * std::sin(time_s * 10.0f);
        return true_alt_m + noise;
    }

private:
    double home_lat_{37.774900};
    double home_lon_{-122.419400};
    FakeRangefinder rangefinder_{};
};

} // namespace abstractx::drivers

#endif // DRIVERS_FAKE_SENSORS_HPP
