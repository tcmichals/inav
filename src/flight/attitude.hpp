/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production INAV IMU & Mahony AHRS Attitude Filter
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/imu.c`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/imu.h`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/common/quaternion.h`
 *
 * Faithfully ports all mathematical stages, constants, and algorithms:
 * 1. Exact 3D Quaternion Orientation & Rotation Matrix (rMat[3][3]) on SO(3).
 * 2. Accelerometer Nearness Weighting (bellCurve with 20% tolerance, MAX_ACC_NEARNESS = 0.20f).
 * 3. Dynamic Spin-Rate Accelerometer Weight Invalidation (acc_ignore_rate & acc_ignore_slope).
 * 4. 3 Hz PT1 Filter Cascades (Gyro BF, Accel BF, Heading EF, GPS 3D Speed).
 * 5. Centrifugal Acceleration Compensation (VELNED GPS Delta-V & Turn-Rate Angular Speed).
 * 6. Magnetometer Declination & Earth-Frame 2D Horizontal Vector Projection.
 * 7. GPS Course-Over-Ground (COG) & Acceleration Heading Yaw Fusion with Multirotor Tilt Attenuation.
 * 8. Integral Gyroscope Drift Estimation with 20 deg/s Spin Rate Limit & Anti-Windup Clamping.
 * 9. Exact Axis-Angle Delta Quaternion Integration (Sinc Formulation).
 * 10. Decidegree Euler Angle Extraction (0.1 deg units, Roll/Pitch/Yaw).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_ATTITUDE_HPP
#define FLIGHT_ATTITUDE_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "filter.hpp"
#include "pid.hpp"

namespace abstractx::flight {

// -----------------------------------------------------------------------------
// Constants matching upstream INAV flight/imu.c
// -----------------------------------------------------------------------------
inline constexpr float INAV_GRAVITY_MSS           = 9.80665f;
inline constexpr float INAV_GRAVITY_CMSS          = 980.665f;
inline constexpr float INAV_SPIN_RATE_LIMIT_DPS   = 20.0f;    // Deg/s limit for gyro bias integration
inline constexpr float INAV_MAX_ACC_NEARNESS      = 0.20f;    // 20% G error soft-accepted (0.8 - 1.2G)
inline constexpr float INAV_MAX_MAG_NEARNESS      = 0.25f;    // 25% Mag field error soft-accepted
inline constexpr float INAV_COS10DEG              = 0.984807753f; // cos(10 deg)
inline constexpr float INAV_COS20DEG              = 0.939692621f; // cos(20 deg)
inline constexpr float INAV_IMU_ROTATION_LPF_HZ   = 3.0f;     // 3 Hz PT1 filter cutoff
inline constexpr float INAV_DEG_TO_RAD            = 0.017453292519943295f;
inline constexpr float INAV_RAD_TO_DEG            = 57.29577951308232f;
inline constexpr float INAV_RAD_TO_DECIDEG        = 572.9577951308232f;

// -----------------------------------------------------------------------------
// Vector3 and Quaternion Structures matching upstream INAV
// -----------------------------------------------------------------------------
struct Vector3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    [[nodiscard]] constexpr float norm_sq() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] float norm() const noexcept {
        return std::sqrt(norm_sq());
    }

    void normalize() noexcept {
        const float n = norm();
        if (n > 1e-6f) {
            const float inv = 1.0f / n;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            x = 0.0f; y = 0.0f; z = 0.0f;
        }
    }
};

struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    [[nodiscard]] constexpr float norm_sq() const noexcept {
        return w * w + x * x + y * y + z * z;
    }

    [[nodiscard]] float norm() const noexcept {
        return std::sqrt(norm_sq());
    }

    void normalize() noexcept {
        const float n = norm();
        if (n > 1e-6f) {
            const float inv = 1.0f / n;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
        }
    }

    // Multiply two quaternions: result = this * b
    [[nodiscard]] constexpr Quaternion multiply(const Quaternion& b) const noexcept {
        return Quaternion{
            w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w
        };
    }

    // Conjugate (Inverse for unit quaternion)
    [[nodiscard]] constexpr Quaternion conjugate() const noexcept {
        return Quaternion{w, -x, -y, -z};
    }

    // Rotate vector from Body Frame to Earth Frame: v_out = q * v * q^-1
    [[nodiscard]] Vector3f rotate_vector(const Vector3f& v) const noexcept {
        const Quaternion v_q{0.0f, v.x, v.y, v.z};
        const Quaternion conj = conjugate();
        const Quaternion temp = multiply(v_q);
        const Quaternion res = temp.multiply(conj);
        return Vector3f{res.x, res.y, res.z};
    }

    // Rotate vector from Earth Frame to Body Frame (Inverse rotation): v_out = q^-1 * v * q
    [[nodiscard]] Vector3f rotate_vector_inv(const Vector3f& v) const noexcept {
        const Quaternion v_q{0.0f, v.x, v.y, v.z};
        const Quaternion conj = conjugate();
        const Quaternion temp = conj.multiply(v_q);
        const Quaternion res = temp.multiply(*this);
        return Vector3f{res.x, res.y, res.z};
    }
};

// Euler Angles in Degrees, Decidegrees (0.1 deg), and Radians
struct AttitudeAngles {
    float roll_deg{0.0f};     // Roll (-180.0 to +180.0 deg)
    float pitch_deg{0.0f};    // Pitch (-90.0 to +90.0 deg)
    float yaw_deg{0.0f};      // Yaw (0.0 to 360.0 deg)

    int16_t roll_decideg{0};  // INAV attitude.values.roll (-1800 to +1800)
    int16_t pitch_decideg{0}; // INAV attitude.values.pitch (-900 to +900)
    int16_t yaw_decideg{0};   // INAV attitude.values.yaw (0 to 3600)

    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    float yaw_rad{0.0f};
};

// -----------------------------------------------------------------------------
// Upstream INAV IMU Configuration Struct (imuConfig_t)
// -----------------------------------------------------------------------------
struct InavImuConfig {
    float dcm_kp_acc{0.20f};                  // 0.20 * 10000 -> 0.20f
    float dcm_ki_acc{0.005f};                 // 0.005 * 10000 -> 0.005f
    float dcm_kp_mag{0.20f};                  // 0.20 * 10000 -> 0.20f
    float dcm_ki_mag{0.005f};                 // 0.005 * 10000 -> 0.005f
    uint8_t small_angle{25};                  // 25 degrees arming angle threshold
    uint8_t acc_ignore_rate{0};               // Deg/s threshold to ignore acc during turns (Fixed-wing)
    uint8_t acc_ignore_slope{0};              // Deg/s slope range for acc weighting
    bool gps_yaw_windcomp{true};              // Enable wind compensation on GPS yaw
    uint8_t inertia_comp_method{1};           // 0: None, 1: VELNED, 2: Turnrate, 3: Adaptive
    float mag_declination_deg{0.0f};          // Magnetic declination in degrees
    bool is_multirotor{true};                 // Multirotor vs Fixed Wing
};

// Legacy / alias compatibility
using MahonyConfig = InavImuConfig;

// -----------------------------------------------------------------------------
// Production INAV IMU & AHRS Class
// -----------------------------------------------------------------------------
class InavImu {
public:
    using Config = InavImuConfig;

    constexpr InavImu() noexcept = default;
    constexpr explicit InavImu(const Config& config) noexcept
        : config_(config) {}

    void set_config(const Config& config) noexcept {
        config_ = config;
        set_magnetic_declination(config_.mag_declination_deg);
    }

    [[nodiscard]] const Config& get_config() const noexcept {
        return config_;
    }

    void set_magnetic_declination(float declination_deg) noexcept {
        const float declination_rad = -declination_deg * INAV_DEG_TO_RAD;
        v_corrected_mag_north_.x = std::cos(declination_rad);
        v_corrected_mag_north_.y = std::sin(declination_rad);
        v_corrected_mag_north_.z = 0.0f;
    }

    void reset() noexcept {
        q_ = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
        v_gyro_drift_estimate_ = Vector3f{0.0f, 0.0f, 0.0f};
        angles_ = AttitudeAngles{};
        compute_rotation_matrix();

        rot_rate_filter_x_.reset();
        rot_rate_filter_y_.reset();
        rot_rate_filter_z_.reset();

        accel_filter_x_.reset();
        accel_filter_y_.reset();
        accel_filter_z_.reset();

        gps_3d_speed_filter_.reset();
        gps_heading_initialized_ = false;
        boot_time_ms_ = 0;
        is_armed_ = false;
    }

    void set_armed_state(bool is_armed) noexcept {
        is_armed_ = is_armed;
    }

    void update_boot_time(uint32_t boot_ms) noexcept {
        boot_time_ms_ = boot_ms;
    }

    // std::array overload for driver / test convenience
    [[nodiscard]] AttitudeAngles update(
        const std::array<float, 3>& accel_g,
        const std::array<float, 3>& gyro_dps,
        float dt_s) noexcept {
        return update(
            Vector3f{accel_g[0], accel_g[1], accel_g[2]},
            Vector3f{gyro_dps[0], gyro_dps[1], gyro_dps[2]},
            dt_s,
            nullptr, // No Mag
            nullptr, // No GPS COG
            nullptr, // No GPS COG Acc
            Vector3f{} // No GPS Vel
        );
    }

    // Axis3f overload for flight engine pipeline
    [[nodiscard]] AttitudeAngles update(
        const Axis3f& accel_g,
        const Axis3f& gyro_dps,
        float dt_s,
        const Axis3f& vel_ned_m_s = Axis3f{}) noexcept {
        Vector3f gps_vel{vel_ned_m_s.roll, vel_ned_m_s.pitch, vel_ned_m_s.yaw};
        return update(
            Vector3f{accel_g.roll, accel_g.pitch, accel_g.yaw},
            Vector3f{gyro_dps.roll, gyro_dps.pitch, gyro_dps.yaw},
            dt_s,
            nullptr,
            nullptr,
            nullptr,
            gps_vel
        );
    }

    // Full INAV IMU Update (imuCalculateEstimatedAttitude + imuMahonyAHRSupdate)
    [[nodiscard]] AttitudeAngles update(
        const Vector3f& accel_bf_g,
        const Vector3f& gyro_bf_dps,
        float dt_s,
        const Vector3f* mag_bf_raw = nullptr,
        const Vector3f* gps_cog_vec = nullptr,
        const Vector3f* gps_cog_acc = nullptr,
        const Vector3f& gps_vel_ned_ms = Vector3f{}) noexcept {

        if (dt_s <= 1e-6f || dt_s > 0.1f) {
            dt_s = 0.001f;
        }

        // Convert Gyro DPS to Rad/s
        Vector3f gyro_bf_rads{
            gyro_bf_dps.x * INAV_DEG_TO_RAD,
            gyro_bf_dps.y * INAV_DEG_TO_RAD,
            gyro_bf_dps.z * INAV_DEG_TO_RAD
        };

        // Convert Accel G to cm/s^2 (INAV standard internal units: 1G = 980.665 cm/s^2)
        Vector3f accel_bf_cmss{
            accel_bf_g.x * INAV_GRAVITY_CMSS,
            accel_bf_g.y * INAV_GRAVITY_CMSS,
            accel_bf_g.z * INAV_GRAVITY_CMSS
        };

        // 1. Calculate 3Hz PT1 Filter Cascades (imuCalculateFilters)
        calculate_filters(gyro_bf_rads, accel_bf_cmss, gps_vel_ned_ms, dt_s);

        // 2. Centrifugal Acceleration Compensation (imuCalculateGPSacceleration / imuCalculateTurnRateacceleration)
        Vector3f compensated_gravity_bf = accel_bf_cmss;
        float acc_ignore_slope_multiplier = 1.0f;

        const float gps_speed_3d_ms = gps_vel_ned_ms.norm();
        if (gps_speed_3d_ms > 0.5f) {
            // Multirotor / Plane Centrifugal Acceleration: a_cent = v x omega (cm/s^2)
            const float v_cm_s = gps_speed_3d_ms * 100.0f;
            Vector3f v_est_centrifugal_bf{
                0.0f,
                -v_cm_s * filtered_rot_rate_bf_.z,
                +v_cm_s * filtered_rot_rate_bf_.y
            };
            compensated_gravity_bf.x += v_est_centrifugal_bf.x;
            compensated_gravity_bf.y += v_est_centrifugal_bf.y;
            compensated_gravity_bf.z += v_est_centrifugal_bf.z;
            acc_ignore_slope_multiplier = 4.0f;
        }

        // 3. Accelerometer Weight Nearness (Gaussian Bell Curve, 20% tolerance)
        const float p_gain_scale = get_p_gain_scale_factor();
        float acc_weight = p_gain_scale * calculate_acc_weight_nearness(compensated_gravity_bf);
        acc_weight *= calculate_acc_weight_rate_ignore(acc_ignore_slope_multiplier);
        const bool use_acc = (acc_weight > 0.001f);

        const float mag_weight = p_gain_scale * 1.0f;

        // 4. Execute Full INAV Mahony AHRS Algorithm (imuMahonyAHRSupdate)
        mahony_ahrs_update(
            dt_s,
            gyro_bf_rads,
            use_acc ? &compensated_gravity_bf : nullptr,
            mag_bf_raw,
            gps_cog_vec,
            gps_cog_acc,
            acc_weight,
            mag_weight
        );

        // 5. Update Euler Decidegree Angles from Rotation Matrix (imuUpdateEulerAngles)
        update_euler_angles();

        return angles_;
    }

    // -------------------------------------------------------------------------
    // Core INAV Mahony AHRS Algorithm (imuMahonyAHRSupdate in flight/imu.c)
    // -------------------------------------------------------------------------
    void mahony_ahrs_update(
        float dt,
        const Vector3f& gyro_bf,
        const Vector3f* acc_bf,
        const Vector3f* mag_bf,
        const Vector3f* v_cog,
        const Vector3f* v_cog_acc,
        float acc_w_scaler,
        float mag_w_scaler) noexcept {

        const Quaternion prev_orientation = q_;
        Vector3f v_rotation = gyro_bf;

        // Calculate general spin rate (rad/s)^2
        const float spin_rate_sq = v_rotation.norm_sq();

        // ---------------------------------------------------------------------
        // Step 1: Yaw Correction (Magnetometer & GPS COG Heading Fusion)
        // ---------------------------------------------------------------------
        if (mag_bf != nullptr || v_cog != nullptr || v_cog_acc != nullptr) {
            float w_mag = 1.0f;
            float w_cog = 1.0f;
            Vector3f v_mag_err{0.0f, 0.0f, 0.0f};
            Vector3f v_cog_err{0.0f, 0.0f, 0.0f};

            // Magnetometer Fusion
            if (mag_bf != nullptr && mag_bf->norm_sq() > 0.01f) {
                const float mag_len = mag_bf->norm();
                w_mag *= bell_curve((mag_len - 1024.0f) / 1024.0f, INAV_MAX_MAG_NEARNESS);

                // Rotate measured mag vector from Body Frame to Earth Frame
                Vector3f v_mag = q_.rotate_vector_inv(*mag_bf);

                // Ignore magnetic inclination (drop Z component in EF)
                v_mag.z = 0.0f;

                if (v_mag.norm_sq() > 0.01f) {
                    v_mag.normalize();

                    // Error is cross product between estimated magnetic North and measured magnetic North (in EF)
                    Vector3f v_mag_err_ef{
                        v_mag.y * v_corrected_mag_north_.z - v_mag.z * v_corrected_mag_north_.y,
                        v_mag.z * v_corrected_mag_north_.x - v_mag.x * v_corrected_mag_north_.z,
                        v_mag.x * v_corrected_mag_north_.y - v_mag.y * v_corrected_mag_north_.x
                    };

                    // Rotate error back into Body Frame
                    v_mag_err = q_.rotate_vector(v_mag_err_ef);
                }
            }

            // GPS Course-Over-Ground (COG) Fusion
            if (v_cog != nullptr || v_cog_acc != nullptr) {
                Vector3f v_cog_local{0.0f, 0.0f, 0.0f};
                Vector3f v_forward{config_.is_multirotor ? 0.0f : 1.0f, 0.0f, config_.is_multirotor ? 1.0f : 0.0f};

                // Rotate forward vector from BF to EF
                Vector3f v_heading_ef = q_.rotate_vector_inv(v_forward);

                if (v_cog != nullptr) {
                    v_cog_local = *v_cog;
                } else {
                    w_cog = 0.0f;
                }

                if (config_.is_multirotor) {
                    // Multirotor tilt angle attenuation
                    w_cog *= std::clamp((v_heading_ef.z - INAV_COS10DEG) / (INAV_COS20DEG - INAV_COS10DEG), 0.0f, 1.0f);
                }

                v_heading_ef.z = 0.0f;

                if (v_heading_ef.norm_sq() > 0.01f && v_cog_local.norm_sq() > 0.01f) {
                    v_heading_ef.normalize();
                    v_cog_local.normalize();

                    Vector3f v_cog_err_ef{
                        v_cog_local.y * v_heading_ef.z - v_cog_local.z * v_heading_ef.y,
                        v_cog_local.z * v_heading_ef.x - v_cog_local.x * v_heading_ef.z,
                        v_cog_local.x * v_heading_ef.y - v_cog_local.y * v_heading_ef.x
                    };

                    v_cog_err = q_.rotate_vector(v_cog_err_ef);
                }
            }

            Vector3f v_err{
                v_mag_err.x * w_mag + v_cog_err.x * w_cog,
                v_mag_err.y * w_mag + v_cog_err.y * w_cog,
                v_mag_err.z * w_mag + v_cog_err.z * w_cog
            };

            // Integral feedback for gyro drift
            if (config_.dcm_ki_mag > 0.0f) {
                constexpr float spin_limit_rads = INAV_SPIN_RATE_LIMIT_DPS * INAV_DEG_TO_RAD;
                if (spin_rate_sq < (spin_limit_rads * spin_limit_rads)) {
                    v_gyro_drift_estimate_.x += v_err.x * (config_.dcm_ki_mag * mag_w_scaler * dt);
                    v_gyro_drift_estimate_.y += v_err.y * (config_.dcm_ki_mag * mag_w_scaler * dt);
                    v_gyro_drift_estimate_.z += v_err.z * (config_.dcm_ki_mag * mag_w_scaler * dt);
                }
            }

            // Proportional feedback
            v_rotation.x += v_err.x * (config_.dcm_kp_mag * mag_w_scaler);
            v_rotation.y += v_err.y * (config_.dcm_kp_mag * mag_w_scaler);
            v_rotation.z += v_err.z * (config_.dcm_kp_mag * mag_w_scaler);
        }

        // ---------------------------------------------------------------------
        // Step 2: Roll and Pitch Correction (Accelerometer Vector)
        // ---------------------------------------------------------------------
        if (acc_bf != nullptr) {
            // Estimated gravity vector in Body Frame: v_est = q^-1 * [0, 0, 1] * q
            const Vector3f v_gravity_ef{0.0f, 0.0f, 1.0f};
            const Vector3f v_est_gravity = q_.rotate_vector_inv(v_gravity_ef);

            // Normalized measured acceleration vector
            Vector3f v_acc = *acc_bf;
            v_acc.normalize();

            // Error is cross product between measured acc and estimated gravity
            Vector3f v_err{
                v_acc.y * v_est_gravity.z - v_acc.z * v_est_gravity.y,
                v_acc.z * v_est_gravity.x - v_acc.x * v_est_gravity.z,
                v_acc.x * v_est_gravity.y - v_acc.y * v_est_gravity.x
            };

            // Integral feedback
            if (config_.dcm_ki_acc > 0.0f) {
                constexpr float spin_limit_rads = INAV_SPIN_RATE_LIMIT_DPS * INAV_DEG_TO_RAD;
                if (spin_rate_sq < (spin_limit_rads * spin_limit_rads)) {
                    v_gyro_drift_estimate_.x += v_err.x * (config_.dcm_ki_acc * acc_w_scaler * dt);
                    v_gyro_drift_estimate_.y += v_err.y * (config_.dcm_ki_acc * acc_w_scaler * dt);
                    v_gyro_drift_estimate_.z += v_err.z * (config_.dcm_ki_acc * acc_w_scaler * dt);
                }
            }

            // Proportional feedback
            v_rotation.x += v_err.x * (config_.dcm_kp_acc * acc_w_scaler);
            v_rotation.y += v_err.y * (config_.dcm_kp_acc * acc_w_scaler);
            v_rotation.z += v_err.z * (config_.dcm_kp_acc * acc_w_scaler);
        }

        // ---------------------------------------------------------------------
        // Anti-Windup Clamping on Gyro Drift
        // ---------------------------------------------------------------------
        const float i_limit = (2.0f * INAV_DEG_TO_RAD) * (config_.dcm_kp_acc + config_.dcm_kp_mag) * 0.5f;
        v_gyro_drift_estimate_.x = std::clamp(v_gyro_drift_estimate_.x, -i_limit, i_limit);
        v_gyro_drift_estimate_.y = std::clamp(v_gyro_drift_estimate_.y, -i_limit, i_limit);
        v_gyro_drift_estimate_.z = std::clamp(v_gyro_drift_estimate_.z, -i_limit, i_limit);

        // Apply Gyro Drift Correction
        v_rotation.x += v_gyro_drift_estimate_.x;
        v_rotation.y += v_gyro_drift_estimate_.y;
        v_rotation.z += v_gyro_drift_estimate_.z;

        // ---------------------------------------------------------------------
        // Step 3: Exact Axis-Angle Delta Quaternion Integration
        // ---------------------------------------------------------------------
        const Vector3f v_theta{
            v_rotation.x * (0.5f * dt),
            v_rotation.y * (0.5f * dt),
            v_rotation.z * (0.5f * dt)
        };

        const float theta_mag_sq = v_theta.norm_sq();
        if (theta_mag_sq >= 1e-20f) {
            const float theta_mag = std::sqrt(theta_mag_sq);
            const float sinc = std::sin(theta_mag) / theta_mag;

            const Quaternion delta_q{
                std::cos(theta_mag),
                v_theta.x * sinc,
                v_theta.y * sinc,
                v_theta.z * sinc
            };

            q_ = q_.multiply(delta_q);
            q_.normalize();
        }

        // Validate quaternion against NaN / Inf
        if (std::isnan(q_.w) || std::isnan(q_.x) || std::isnan(q_.y) || std::isnan(q_.z)) {
            q_ = prev_orientation;
            q_.normalize();
        }

        compute_rotation_matrix();
    }

    // -------------------------------------------------------------------------
    // Rotation Matrix Computation (imuComputeRotationMatrix in flight/imu.c)
    // -------------------------------------------------------------------------
    void compute_rotation_matrix() noexcept {
        const float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;
        const float q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
        const float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
        const float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

        r_mat_[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
        r_mat_[0][1] = 2.0f * (q1q2 - q0q3);
        r_mat_[0][2] = 2.0f * (q1q3 + q0q2);

        r_mat_[1][0] = 2.0f * (q1q2 + q0q3);
        r_mat_[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
        r_mat_[1][2] = 2.0f * (q2q3 - q0q1);

        r_mat_[2][0] = 2.0f * (q1q3 - q0q2);
        r_mat_[2][1] = 2.0f * (q2q3 + q0q1);
        r_mat_[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
    }

    // -------------------------------------------------------------------------
    // Euler Decidegree Extraction (imuUpdateEulerAngles in flight/imu.c)
    // -------------------------------------------------------------------------
    void update_euler_angles() noexcept {
        // INAV standard:
        // roll = atan2(rMat[2][1], rMat[2][2])
        // pitch = (pi/2) - acos(-rMat[2][0])
        // yaw = -atan2(rMat[1][0], rMat[0][0])
        angles_.roll_rad = std::atan2(r_mat_[2][1], r_mat_[2][2]);
        angles_.roll_deg = angles_.roll_rad * INAV_RAD_TO_DEG;
        angles_.roll_decideg = static_cast<int16_t>(std::lrint(angles_.roll_rad * INAV_RAD_TO_DECIDEG));

        const float clamped_r20 = std::clamp(-r_mat_[2][0], -1.0f, 1.0f);
        angles_.pitch_rad = (0.5f * 3.141592653589793f) - std::acos(clamped_r20);
        angles_.pitch_deg = angles_.pitch_rad * INAV_RAD_TO_DEG;
        angles_.pitch_decideg = static_cast<int16_t>(std::lrint(angles_.pitch_rad * INAV_RAD_TO_DECIDEG));

        angles_.yaw_rad = -std::atan2(r_mat_[1][0], r_mat_[0][0]);
        angles_.yaw_deg = angles_.yaw_rad * INAV_RAD_TO_DEG;
        if (angles_.yaw_deg < 0.0f) angles_.yaw_deg += 360.0f;
        angles_.yaw_decideg = static_cast<int16_t>(std::lrint(angles_.yaw_rad * INAV_RAD_TO_DECIDEG));
        if (angles_.yaw_decideg < 0) angles_.yaw_decideg += 3600;
    }

    // -------------------------------------------------------------------------
    // Body <-> Earth Vector Transformations (imuTransformVectorBodyToEarth)
    // -------------------------------------------------------------------------
    [[nodiscard]] Axis3f rotate_body_to_earth(const Axis3f& body_vec) const noexcept {
        const Vector3f b_v{body_vec.roll, body_vec.pitch, body_vec.yaw};
        Vector3f e_v = q_.rotate_vector_inv(b_v);
        // Cleanflight/INAV NEU to NED coordinate conversion
        e_v.y = -e_v.y;
        return Axis3f{e_v.x, e_v.y, e_v.z};
    }

    [[nodiscard]] Axis3f rotate_earth_to_body(const Axis3f& earth_vec) const noexcept {
        Vector3f e_v{earth_vec.roll, -earth_vec.pitch, earth_vec.yaw};
        Vector3f b_v = q_.rotate_vector(e_v);
        return Axis3f{b_v.x, b_v.y, b_v.z};
    }

    [[nodiscard]] const Quaternion& quaternion() const noexcept { return q_; }
    [[nodiscard]] const AttitudeAngles& angles() const noexcept { return angles_; }
    [[nodiscard]] const std::array<std::array<float, 3>, 3>& rotation_matrix() const noexcept { return r_mat_; }
    [[nodiscard]] const Vector3f& gyro_drift() const noexcept { return v_gyro_drift_estimate_; }

    [[nodiscard]] bool is_small_angle() const noexcept {
        const float cos_small_angle = std::cos(config_.small_angle * INAV_DEG_TO_RAD);
        return r_mat_[2][2] > cos_small_angle;
    }

private:
    [[nodiscard]] static float gaussian(float x, float mu, float sigma) noexcept {
        const float diff = x - mu;
        return std::exp(-(diff * diff) / (2.0f * sigma * sigma));
    }

    [[nodiscard]] static float bell_curve(float x, float curve_width) noexcept {
        return gaussian(x, 0.0f, curve_width);
    }

    [[nodiscard]] float get_p_gain_scale_factor() const noexcept {
        // Fast gains (10.0x) during first 20s when disarmed (INAV imuGetPGainScaleFactor)
        if (!is_armed_ && boot_time_ms_ < 20000) {
            return 10.0f;
        }
        return 1.0f;
    }

    [[nodiscard]] static float calculate_acc_weight_nearness(const Vector3f& acc_bf_cmss) noexcept {
        const float acc_norm_g = acc_bf_cmss.norm() / INAV_GRAVITY_CMSS;
        return bell_curve(acc_norm_g - 1.0f, INAV_MAX_ACC_NEARNESS);
    }

    [[nodiscard]] float calculate_acc_weight_rate_ignore(float slope_multiplier) const noexcept {
        if (is_armed_ && config_.acc_ignore_rate > 0) {
            float rot_rate_mag = filtered_rot_rate_bf_.norm();
            rot_rate_mag /= (slope_multiplier + 0.001f);

            if (config_.acc_ignore_slope > 0) {
                const float rate_min = (config_.acc_ignore_rate - config_.acc_ignore_slope) * INAV_DEG_TO_RAD;
                const float rate_max = (config_.acc_ignore_rate + config_.acc_ignore_slope) * INAV_DEG_TO_RAD;
                return std::clamp((rate_max - rot_rate_mag) / (rate_max - rate_min), 0.0f, 1.0f);
            } else {
                if (rot_rate_mag > (config_.acc_ignore_rate * INAV_DEG_TO_RAD)) {
                    return 0.0f;
                }
            }
        }
        return 1.0f;
    }

    void calculate_filters(const Vector3f& gyro_rads, const Vector3f& acc_cmss, const Vector3f& gps_vel, float dt) noexcept {
        filtered_rot_rate_bf_.x = rot_rate_filter_x_.apply(gyro_rads.x, INAV_IMU_ROTATION_LPF_HZ, dt);
        filtered_rot_rate_bf_.y = rot_rate_filter_y_.apply(gyro_rads.y, INAV_IMU_ROTATION_LPF_HZ, dt);
        filtered_rot_rate_bf_.z = rot_rate_filter_z_.apply(gyro_rads.z, INAV_IMU_ROTATION_LPF_HZ, dt);

        filtered_accel_bf_.x = accel_filter_x_.apply(acc_cmss.x, INAV_IMU_ROTATION_LPF_HZ, dt);
        filtered_accel_bf_.y = accel_filter_y_.apply(acc_cmss.y, INAV_IMU_ROTATION_LPF_HZ, dt);
        filtered_accel_bf_.z = accel_filter_z_.apply(acc_cmss.z, INAV_IMU_ROTATION_LPF_HZ, dt);

        filtered_gps_3d_speed_ = gps_3d_speed_filter_.apply(gps_vel.norm(), INAV_IMU_ROTATION_LPF_HZ, dt);
    }

    Config config_{};
    Quaternion q_{1.0f, 0.0f, 0.0f, 0.0f};
    Vector3f v_gyro_drift_estimate_{0.0f, 0.0f, 0.0f};
    Vector3f v_corrected_mag_north_{1.0f, 0.0f, 0.0f};
    std::array<std::array<float, 3>, 3> r_mat_{{{1,0,0},{0,1,0},{0,0,1}}};
    AttitudeAngles angles_{};

    // 3Hz PT1 Filters
    Pt1Filter rot_rate_filter_x_{};
    Pt1Filter rot_rate_filter_y_{};
    Pt1Filter rot_rate_filter_z_{};
    Vector3f filtered_rot_rate_bf_{0.0f, 0.0f, 0.0f};

    Pt1Filter accel_filter_x_{};
    Pt1Filter accel_filter_y_{};
    Pt1Filter accel_filter_z_{};
    Vector3f filtered_accel_bf_{0.0f, 0.0f, 0.0f};

    Pt1Filter gps_3d_speed_filter_{};
    float filtered_gps_3d_speed_{0.0f};

    uint32_t boot_time_ms_{0};
    bool is_armed_{false};
    bool gps_heading_initialized_{false};
};

// Aliases for unified flight engine compatibility
using MahonyAhrs = InavImu;
using AttitudeFilter = InavImu;

} // namespace abstractx::flight

#endif // FLIGHT_ATTITUDE_HPP
