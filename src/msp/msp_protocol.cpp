/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MultiWii Serial Protocol (MSP v1/v2) C++20 Processor
 */

#include "msp_protocol.hpp"
#include "config_registry.hpp"

namespace abstractx::msp {

bool MspEngine::process_command(Cmd cmd, const std::span<const uint8_t>& rx_payload, MspFrame& tx_frame) noexcept {
    tx_frame.reset();
    tx_frame.command = cmd;

    auto& config = ConfigRegistry::get();

    switch (cmd) {
        case Cmd::ApiVersion:
            tx_frame.push_u8(2); // MSP protocol version
            tx_frame.push_u8(2); // API major
            tx_frame.push_u8(0); // API minor
            return true;

        case Cmd::FcVariant:
            // "INAV" FC Variant string
            tx_frame.push_u8('I');
            tx_frame.push_u8('N');
            tx_frame.push_u8('A');
            tx_frame.push_u8('V');
            return true;

        case Cmd::FcVersion:
            tx_frame.push_u8(3); // Major 3
            tx_frame.push_u8(0); // Minor 0
            tx_frame.push_u8(0); // Patch 0
            return true;

        case Cmd::BoardInfo:
            // Board Identifier "ASP6" (AbstractX PCIe TLP 64B)
            tx_frame.push_u8('A');
            tx_frame.push_u8('S');
            tx_frame.push_u8('P');
            tx_frame.push_u8('6');
            tx_frame.push_u16(0); // Hardware revision
            return true;

        case Cmd::Pid:
            // Output Roll, Pitch, Yaw PIDs (scaled by 10 for MSP wire format)
            for (int i = 0; i < 3; ++i) {
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.kp[i] * 10.0f));
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.ki[i] * 10.0f));
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.kd[i] * 1000.0f));
            }
            return true;

        case Cmd::SetPid:
            // Save PIDs received from iNav Configurator
            if (rx_payload.size() >= 9) {
                for (int i = 0; i < 3; ++i) {
                    config.pid.kp[i] = static_cast<float>(rx_payload[i * 3 + 0]) / 10.0f;
                    config.pid.ki[i] = static_cast<float>(rx_payload[i * 3 + 1]) / 10.0f;
                    config.pid.kd[i] = static_cast<float>(rx_payload[i * 3 + 2]) / 1000.0f;
                }
                return true;
            }
            return false;

        case Cmd::RawImu:
            // Dummy or HIL IMU stream (Accel X/Y/Z, Gyro X/Y/Z)
            tx_frame.push_u16(0); // Accel X
            tx_frame.push_u16(0); // Accel Y
            tx_frame.push_u16(512); // Accel Z (1G)
            tx_frame.push_u16(0); // Gyro X
            tx_frame.push_u16(0); // Gyro Y
            tx_frame.push_u16(0); // Gyro Z
            tx_frame.push_u16(0); // Mag X
            tx_frame.push_u16(0); // Mag Y
            tx_frame.push_u16(0); // Mag Z
            return true;

        case Cmd::Attitude:
            // Roll (0.1 deg), Pitch (0.1 deg), Yaw (1 deg)
            tx_frame.push_u16(0); // Roll 0 deg
            tx_frame.push_u16(0); // Pitch 0 deg
            tx_frame.push_u16(180); // Yaw 180 deg
            return true;

        default:
            return false;
    }
}

} // namespace abstractx::msp
