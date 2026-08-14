/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 * Copyright (C) Robert Mahony et al. (Nonlinear Complementary Filters on SO(3))
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Mahony AHRS Quaternion Attitude Filter
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/imu.c
 *   - Upstream INAV: src/main/flight/ahrs.c
 *
 * Features:
 * 1. 3D Quaternion Attitude Integration ($q_0, q_1, q_2, q_3$) with zero gimbal lock.
 * 2. Centrifugal Acceleration Correction ($\vec{a}_{\text{cent}} = \vec{\omega} \times \vec{v}$) for coordinated turns.
 * 3. Dynamic Accelerometer G-Force Gating (Rejects high-vibration / non-1G transients).
 * 4. Integral Feedback Gyroscope Bias Estimation ($K_i$).
 * 5. Euler Angle Extraction (Roll, Pitch, Yaw) and Rotation Matrix transforms.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_ATTITUDE_HPP
#define FLIGHT_ATTITUDE_HPP


#include <cmath>
#include <array>
#include <algorithm>
#include "pid.hpp"

namespace abstractx::flight {

// Quaternion Representation
struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    void normalize() noexcept {
        const float norm_sq = w * w + x * x + y * y + z * z;
        if (norm_sq > 0.000001f) {
            const float inv_norm = 1.0f / std::sqrt(norm_sq);
            w *= inv_norm;
            x *= inv_norm;
            y *= inv_norm;
            z *= inv_norm;
        } else {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
    }
};

// Euler Angles in Degrees and Radians
struct AttitudeAngles {
    float roll_deg{0.0f};  // Roll (-180 to +180 deg)
    float pitch_deg{0.0f}; // Pitch (-90 to +90 deg)
    float yaw_deg{0.0f};   // Yaw (0 to 360 deg)

    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    float yaw_rad{0.0f};
};

struct MahonyConfig {
    float kp{0.50f};              // Proportional accelerometer feedback gain
    float ki{0.05f};              // Integral gyro bias feedback gain
    float accel_g_min{0.80f};     // Lower G-force gating bound (Gs)
    float accel_g_max{1.20f};     // Upper G-force gating bound (Gs)
    bool enable_centrifugal{true}; // Enable centrifugal acceleration compensation
};

class MahonyAhrs {
public:
    using Config = MahonyConfig;

    constexpr MahonyAhrs() noexcept = default;
    constexpr explicit MahonyAhrs(const Config& config) noexcept
        : config_(config) {}

    void set_config(const Config& config) noexcept {
        config_ = config;
    }

    [[nodiscard]] const Config& get_config() const noexcept {
        return config_;
    }


    void reset() noexcept {
        q_ = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
        gyro_bias_ = Axis3f{0.0f, 0.0f, 0.0f};
        angles_ = AttitudeAngles{};
    }

    // std::array overload for legacy / driver convenience
    [[nodiscard]] AttitudeAngles update(
        const std::array<float, 3>& accel_g,
        const std::array<float, 3>& gyro_dps,
        float dt_s) noexcept {
        return update(
            Axis3f{accel_g[0], accel_g[1], accel_g[2]},
            Axis3f{gyro_dps[0], gyro_dps[1], gyro_dps[2]},
            dt_s,
            Axis3f{}
        );
    }

    // Main AHRS Update Loop (1kHz standard rate)
    // accel in Gs, gyro in deg/s, velocity in m/s (NED frame)
    [[nodiscard]] AttitudeAngles update(
        const Axis3f& accel_g,
        const Axis3f& gyro_dps,
        float dt_s,
        const Axis3f& vel_ned_m_s = Axis3f{}) noexcept {


        if (dt_s <= 0.00001f || dt_s > 0.1f) {
            dt_s = 0.001f; // 1kHz default guard
        }

        constexpr float DEG_TO_RAD = 0.017453292519943295f;
        constexpr float RAD_TO_DEG = 57.29577951308232f;
        constexpr float GRAVITY_MSS = 9.80665f;

        // Convert Gyro to rad/s
        float gx = gyro_dps.roll * DEG_TO_RAD;
        float gy = gyro_dps.pitch * DEG_TO_RAD;
        float gz = gyro_dps.yaw * DEG_TO_RAD;

        // 1. Centrifugal Acceleration Correction (INAV imu.c)
        Axis3f a_corr = accel_g;
        if (config_.enable_centrifugal) {
            // Body horizontal speed approximation from NED velocity
            const float v_fwd = std::sqrt(vel_ned_m_s.roll * vel_ned_m_s.roll + vel_ned_m_s.pitch * vel_ned_m_s.pitch);
            if (v_fwd > 1.0f) {
                // Centrifugal acceleration = omega x v
                const float a_cent_y = (gz * v_fwd) / GRAVITY_MSS; // Centrifugal sideways G
                const float a_cent_z = (-gy * v_fwd) / GRAVITY_MSS; // Centrifugal vertical G
                a_corr.pitch -= a_cent_y;
                a_corr.yaw -= a_cent_z;
            }
        }

        // 2. Compute Accelerometer G-Force Norm
        const float accel_norm_sq = a_corr.roll * a_corr.roll + a_corr.pitch * a_corr.pitch + a_corr.yaw * a_corr.yaw;
        const float accel_norm = std::sqrt(accel_norm_sq);

        // 3. Accelerometer Error Feedback (Gated within trust bounds)
        Axis3f error{};
        if (accel_norm >= config_.accel_g_min && accel_norm <= config_.accel_g_max && accel_norm > 0.0001f) {
            const float inv_norm = 1.0f / accel_norm;
            const float ax = a_corr.roll * inv_norm;
            const float ay = a_corr.pitch * inv_norm;
            const float az = a_corr.yaw * inv_norm;

            // Estimated gravity direction from current quaternion attitude
            const float vx = 2.0f * (q_.x * q_.z - q_.w * q_.y);
            const float vy = 2.0f * (q_.w * q_.x + q_.y * q_.z);
            const float vz = q_.w * q_.w - q_.x * q_.x - q_.y * q_.y + q_.z * q_.z;

            // Error is cross product between measured gravity and estimated gravity
            error.roll  = (ay * vz - az * vy);
            error.pitch = (az * vx - ax * vz);
            error.yaw   = (ax * vy - ay * vx);

            // Accumulate integral gyro bias with error-gating & anti-windup (INAV imu.c)
            if (std::abs(error.roll) < 0.10f) {
                gyro_bias_.roll  += config_.ki * error.roll * dt_s;
                gyro_bias_.roll = std::clamp(gyro_bias_.roll, -0.05f, 0.05f);
            } else {
                gyro_bias_.roll *= 0.999f; // Decay during large transient steps
            }

            if (std::abs(error.pitch) < 0.10f) {
                gyro_bias_.pitch += config_.ki * error.pitch * dt_s;
                gyro_bias_.pitch = std::clamp(gyro_bias_.pitch, -0.05f, 0.05f);
            } else {
                gyro_bias_.pitch *= 0.999f;
            }

            if (std::abs(error.yaw) < 0.10f) {
                gyro_bias_.yaw   += config_.ki * error.yaw * dt_s;
                gyro_bias_.yaw = std::clamp(gyro_bias_.yaw, -0.05f, 0.05f);
            } else {
                gyro_bias_.yaw *= 0.999f;
            }
        }


        // 4. Apply Proportional & Integral Feedback to Gyro Rates
        gx += config_.kp * error.roll  + gyro_bias_.roll;
        gy += config_.kp * error.pitch + gyro_bias_.pitch;
        gz += config_.kp * error.yaw   + gyro_bias_.yaw;

        // 5. Integrate Quaternion Derivative (Runge-Kutta 1st order)
        const float half_dt = 0.5f * dt_s;
        const float qw = q_.w;
        const float qx = q_.x;
        const float qy = q_.y;
        const float qz = q_.z;

        q_.w += (-qx * gx - qy * gy - qz * gz) * half_dt;
        q_.x += ( qw * gx + qy * gz - qz * gy) * half_dt;
        q_.y += ( qw * gy - qx * gz + qz * gx) * half_dt;
        q_.z += ( qw * gz + qx * gy - qy * gx) * half_dt;

        q_.normalize();

        // 6. Extract Euler Angles
        extract_euler_angles(RAD_TO_DEG);

        return angles_;
    }

    // -------------------------------------------------------------------------
    // Transform a 3D vector from Body Frame to Earth (NED) Frame
    // -------------------------------------------------------------------------
    [[nodiscard]] Axis3f rotate_body_to_earth(const Axis3f& body_vec) const noexcept {
        const float qw = q_.w, qx = q_.x, qy = q_.y, qz = q_.z;

        // Rotation matrix elements derived from quaternion:
        const float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
        const float r12 = 2.0f * (qx * qy - qw * qz);
        const float r13 = 2.0f * (qx * qz + qw * qy);

        const float r21 = 2.0f * (qx * qy + qw * qz);
        const float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
        const float r23 = 2.0f * (qy * qz - qw * qx);

        const float r31 = 2.0f * (qx * qz - qw * qy);
        const float r32 = 2.0f * (qy * qz + qw * qx);
        const float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

        Axis3f earth{};
        earth.roll  = r11 * body_vec.roll + r12 * body_vec.pitch + r13 * body_vec.yaw; // North
        earth.pitch = r21 * body_vec.roll + r22 * body_vec.pitch + r23 * body_vec.yaw; // East
        earth.yaw   = r31 * body_vec.roll + r32 * body_vec.pitch + r33 * body_vec.yaw; // Down
        return earth;
    }

    [[nodiscard]] const Quaternion& quaternion() const noexcept { return q_; }
    [[nodiscard]] const AttitudeAngles& angles() const noexcept { return angles_; }

private:
    void extract_euler_angles(float rad_to_deg) noexcept {
        const float qw = q_.w, qx = q_.x, qy = q_.y, qz = q_.z;

        // Roll (x-axis rotation): -pi to +pi
        angles_.roll_rad = std::atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
        angles_.roll_deg = angles_.roll_rad * rad_to_deg;

        // Pitch (y-axis rotation): -pi/2 to +pi/2
        const float sin_pitch = 2.0f * (qw * qy - qz * qx);
        if (std::abs(sin_pitch) >= 1.0f) {
            angles_.pitch_rad = std::copysign(3.14159265f / 2.0f, sin_pitch); // Gimbal lock guard
        } else {
            angles_.pitch_rad = std::asin(sin_pitch);
        }
        angles_.pitch_deg = angles_.pitch_rad * rad_to_deg;

        // Yaw (z-axis rotation): 0 to 360 deg
        angles_.yaw_rad = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
        angles_.yaw_deg = angles_.yaw_rad * rad_to_deg;
        if (angles_.yaw_deg < 0.0f) angles_.yaw_deg += 360.0f;
    }

    Config config_{};
    Quaternion q_{1.0f, 0.0f, 0.0f, 0.0f};
    Axis3f gyro_bias_{0.0f, 0.0f, 0.0f};
    AttitudeAngles angles_{};
};

// Backwards compatibility alias
using AttitudeFilter = MahonyAhrs;

} // namespace abstractx::flight

#endif // FLIGHT_ATTITUDE_HPP
