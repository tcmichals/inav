/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2015-2026 Betaflight Contributors (BorisB, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - State Estimator Wrapper (Mahony AHRS + INAV Position Estimator)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/imu.c, src/main/navigation/navigation_pos_estimator.c
 *   - Upstream Betaflight: src/main/flight/imu.c
 */

#ifndef FLIGHT_EKF3_HPP
#define FLIGHT_EKF3_HPP


#include "attitude.hpp"
#include "pos_estimator.hpp"
#include <cstdint>
#include <array>

namespace abstractx::flight {

struct EkfState {
    std::array<float, 3> pos_ned_cm{0.0f, 0.0f, 0.0f};  // North, East, Down (cm)
    std::array<float, 3> vel_ned_cms{0.0f, 0.0f, 0.0f}; // North, East, Down (cm/s)
    AttitudeAngles attitude{};                          // Roll, Pitch, Yaw (deg)
    std::array<float, 3> gyro_bias{0.0f, 0.0f, 0.0f};   // Estimated Gyro Biases
    uint64_t last_update_ns{0};
    bool is_healthy{true};
};

class Ekf3Filter {
public:
    constexpr Ekf3Filter() noexcept = default;

    void reset() noexcept {
        ahrs_.reset();
        pos_estimator_.reset();
        last_update_ns_ = 0;
        cached_state_ = EkfState{};
    }

    template <typename SampleType>
    void predict_imu(const SampleType& sample) noexcept {
        if (last_update_ns_ == 0) {
            last_update_ns_ = sample.timestamp_ns;
            return;
        }

        float dt_s = static_cast<float>(sample.timestamp_ns - last_update_ns_) * 1e-9f;
        if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.001f;
        last_update_ns_ = sample.timestamp_ns;

        Axis3f accel_g{sample.accel_g[0], sample.accel_g[1], sample.accel_g[2]};
        Axis3f gyro_dps{sample.gyro_deg_s[0], sample.gyro_deg_s[1], sample.gyro_deg_s[2]};

        // 1. Update Mahony AHRS Attitude
        cached_state_.attitude = ahrs_.update(accel_g, gyro_dps, dt_s);

        // 2. Update Inertial Position Estimator
        pos_estimator_.predict_imu(accel_g, ahrs_, dt_s);

        // Sync with legacy cache struct
        const auto& pos_st = pos_estimator_.state();
        cached_state_.pos_ned_cm = {pos_st.pos_n_m * 100.0f, pos_st.pos_e_m * 100.0f, pos_st.pos_d_m * 100.0f};
        cached_state_.vel_ned_cms = {pos_st.vel_n_m_s * 100.0f, pos_st.vel_e_m_s * 100.0f, pos_st.vel_d_m_s * 100.0f};
        cached_state_.is_healthy = pos_st.is_healthy;
        cached_state_.last_update_ns = sample.timestamp_ns;
    }

    void correct_baro(float alt_cm, uint64_t timestamp_ns = 0) noexcept {
        (void)timestamp_ns;
        pos_estimator_.correct_baro(alt_cm * 0.01f);
    }


    void correct_gps(double lat, double lon, float alt_m, float vel_n, float vel_e, float hdop, uint8_t sats) noexcept {
        pos_estimator_.correct_gps(lat, lon, alt_m, vel_n, vel_e, hdop, sats);
    }

    [[nodiscard]] const EkfState& state() const noexcept { return cached_state_; }
    [[nodiscard]] const MahonyAhrs& ahrs() const noexcept { return ahrs_; }
    [[nodiscard]] const InertialPosEstimator& pos_estimator() const noexcept { return pos_estimator_; }

private:
    MahonyAhrs ahrs_{};
    InertialPosEstimator pos_estimator_{};
    uint64_t last_update_ns_{0};
    EkfState cached_state_{};
};

} // namespace abstractx::flight

#endif // FLIGHT_EKF3_HPP
