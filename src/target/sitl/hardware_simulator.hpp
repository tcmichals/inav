/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Linux 6-DOF Quadcopter Hardware Simulation Engine (SITL / HIL)
 */

#ifndef HARDWARE_SIMULATOR_HPP
#define HARDWARE_SIMULATOR_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "blackbox_logger.hpp"
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>

namespace abstractx::sitl {

class HardwareSimulator {
public:
    HardwareSimulator() noexcept : current_timestamp_ns_(1000000000ULL) {}

    void set_motor(size_t index, uint16_t pwm) noexcept {
        if (index < 4) {
            motor_pwm_[index] = std::clamp(pwm, static_cast<uint16_t>(1000), static_cast<uint16_t>(2000));
        }
    }

    // Step 6-DOF physics and hardware simulation by `dt_us` microseconds
    void step(uint64_t dt_us, SpscTlpRing<64>& telemetry_ring) noexcept {
        const float dt_s = static_cast<float>(dt_us) * 1e-6f;
        current_timestamp_ns_ += (dt_us * 1000ULL);
        step_count_++;

        // 1. Calculate normalized thrust for QuadX motors (0.0 to 1.0)
        std::array<float, 4> thrust{};
        for (size_t i = 0; i < 4; ++i) {
            thrust[i] = static_cast<float>(motor_pwm_[i] - 1000) / 1000.0f;
        }

        // 2. QuadX Rigid-Body Dynamics:
        // Motor 0: Rear-Right  (+Roll, -Pitch, -Yaw)
        // Motor 1: Front-Right (+Roll, +Pitch, +Yaw)
        // Motor 2: Rear-Left   (-Roll, -Pitch, +Yaw)
        // Motor 3: Front-Left  (-Roll, +Pitch, -Yaw)
        constexpr float TORQUE_GAIN = 1500.0f; // deg/s^2 per unit differential thrust
        constexpr float YAW_GAIN    = 600.0f;
        constexpr float DAMPING     = 4.0f;    // Aerodynamic rotational drag

        float torque_roll  = TORQUE_GAIN * ((thrust[0] + thrust[1]) - (thrust[2] + thrust[3]));
        float torque_pitch = TORQUE_GAIN * ((thrust[1] + thrust[3]) - (thrust[0] + thrust[2]));
        float torque_yaw   = YAW_GAIN    * ((thrust[1] + thrust[2]) - (thrust[0] + thrust[3]));

        // Angular acceleration -> rate integration (deg/s)
        gyro_rate_deg_s_[0] += (torque_roll  - DAMPING * gyro_rate_deg_s_[0]) * dt_s;
        gyro_rate_deg_s_[1] += (torque_pitch - DAMPING * gyro_rate_deg_s_[1]) * dt_s;
        gyro_rate_deg_s_[2] += (torque_yaw   - DAMPING * gyro_rate_deg_s_[2]) * dt_s;

        // Rate -> Attitude Angle integration (deg)
        roll_deg_  += gyro_rate_deg_s_[0] * dt_s;
        pitch_deg_ += gyro_rate_deg_s_[1] * dt_s;
        yaw_deg_   += gyro_rate_deg_s_[2] * dt_s;

        // Clamp attitudes to realistic domain
        roll_deg_  = std::clamp(roll_deg_,  -85.0f, 85.0f);
        pitch_deg_ = std::clamp(pitch_deg_, -85.0f, 85.0f);

        // 3. Simulated Body-Frame Accelerometer (1G gravity vector rotated into body frame)
        constexpr float DEG2RAD = 0.0174532925f;
        float sin_p = std::sin(pitch_deg_ * DEG2RAD);
        float cos_p = std::cos(pitch_deg_ * DEG2RAD);
        float sin_r = std::sin(roll_deg_ * DEG2RAD);
        float cos_r = std::cos(roll_deg_ * DEG2RAD);

        float acc_x_g = -sin_p;
        float acc_y_g =  sin_r * cos_p;
        float acc_z_g =  cos_r * cos_p;

        // 4. Pack TLP stream for ICM-42688-P IMU (2048 LSB/g, 16.4 LSB/(deg/s))
        if (imu_enabled_ && (step_count_ % 1 == 0)) {
            Tlp64 imu_tlp{};
            imu_tlp.wire.type = static_cast<uint8_t>(TlpType::DmaStream);
            imu_tlp.wire.channel = static_cast<uint8_t>(Channel::Telemetry);
            imu_tlp.wire.target_address = bar::ImuBase + reg::imu::ContinuousAddr;
            imu_tlp.wire.timestamp_ns = current_timestamp_ns_;
            imu_tlp.wire.sequence = static_cast<uint16_t>(step_count_);

            int16_t raw_ax = static_cast<int16_t>(acc_x_g * 2048.0f);
            int16_t raw_ay = static_cast<int16_t>(acc_y_g * 2048.0f);
            int16_t raw_az = static_cast<int16_t>(acc_z_g * 2048.0f);

            int16_t raw_gx = static_cast<int16_t>(gyro_rate_deg_s_[0] * 16.4f);
            int16_t raw_gy = static_cast<int16_t>(gyro_rate_deg_s_[1] * 16.4f);
            int16_t raw_gz = static_cast<int16_t>(gyro_rate_deg_s_[2] * 16.4f);

            // 14-byte ICM-42688-P payload
            imu_tlp.wire.payload[0] = static_cast<uint8_t>(raw_ax >> 8);
            imu_tlp.wire.payload[1] = static_cast<uint8_t>(raw_ax & 0xFF);
            imu_tlp.wire.payload[2] = static_cast<uint8_t>(raw_ay >> 8);
            imu_tlp.wire.payload[3] = static_cast<uint8_t>(raw_ay & 0xFF);
            imu_tlp.wire.payload[4] = static_cast<uint8_t>(raw_az >> 8);
            imu_tlp.wire.payload[5] = static_cast<uint8_t>(raw_az & 0xFF);

            imu_tlp.wire.payload[6] = static_cast<uint8_t>(raw_gx >> 8);
            imu_tlp.wire.payload[7] = static_cast<uint8_t>(raw_gx & 0xFF);
            imu_tlp.wire.payload[8] = static_cast<uint8_t>(raw_gy >> 8);
            imu_tlp.wire.payload[9] = static_cast<uint8_t>(raw_gy & 0xFF);
            imu_tlp.wire.payload[10]= static_cast<uint8_t>(raw_gz >> 8);
            imu_tlp.wire.payload[11]= static_cast<uint8_t>(raw_gz & 0xFF);

            telemetry_ring.push(imu_tlp);
        }
    }

    void handle_mem_write(uint32_t addr, uint32_t value) noexcept {
        if (addr == (bar::ImuBase + reg::imu::Control)) {
            imu_enabled_ = (value & 0x01) != 0;
        }
    }

    uint64_t timestamp_ns() const noexcept { return current_timestamp_ns_; }
    float roll_deg() const noexcept { return roll_deg_; }
    float pitch_deg() const noexcept { return pitch_deg_; }
    float yaw_deg() const noexcept { return yaw_deg_; }

private:
    uint64_t current_timestamp_ns_{0};
    uint64_t step_count_{0};
    bool imu_enabled_{true};

    std::array<uint16_t, 4> motor_pwm_{1000, 1000, 1000, 1000};
    std::array<float, 3> gyro_rate_deg_s_{0.0f, 0.0f, 0.0f};

    float roll_deg_{0.0f};
    float pitch_deg_{0.0f};
    float yaw_deg_{0.0f};
};

} // namespace abstractx::sitl

#endif // HARDWARE_SIMULATOR_HPP
