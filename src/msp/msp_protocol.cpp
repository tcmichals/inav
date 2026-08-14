/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 * Copyright (C) 2010-2014 MultiWii Contributors (Alexinparis, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - MSP Command Processor — All iNav Configurator Handshake Commands
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/io/msp.c
 *   - Upstream Cleanflight: src/main/io/msp.c
 *
 * Portable — accepts live state by const reference. Zero I/O, zero platform dependencies.
 */

#include "msp_protocol.hpp"
#include "config_registry.hpp"
#include "ez_tune.hpp"


namespace abstractx::msp {

bool MspEngine::process_command(Cmd cmd,
                                 const std::span<const uint8_t>& rx_payload,
                                 MspFrame& tx_frame,
                                 const MspLiveState& live) noexcept {
    tx_frame.reset();
    tx_frame.command = static_cast<uint16_t>(cmd);

    auto& config = ConfigRegistry::get();

    switch (cmd) {

        // ---- Identity & Handshake ----

        case Cmd::ApiVersion:
            tx_frame.push_u8(2);  // MSP protocol version
            tx_frame.push_u8(2);  // API major
            tx_frame.push_u8(0);  // API minor
            return true;

        case Cmd::FcVariant:
            tx_frame.push_string("INAV");
            return true;

        case Cmd::FcVersion:
            tx_frame.push_u8(7);  // Major
            tx_frame.push_u8(1);  // Minor
            tx_frame.push_u8(0);  // Patch
            return true;

        case Cmd::BoardInfo:
            // Board identifier (4 chars)
            tx_frame.push_string("ASP6");
            tx_frame.push_u16(0);  // Hardware revision
            tx_frame.push_u8(0);   // Board type (0 = FC)
            tx_frame.push_u8(2);   // Target capabilities (HAS_FLASH | HAS_VCP)
            // Target name string length + string
            tx_frame.push_u8(14); // length
            tx_frame.push_string("ABSTRACTX_SITL");
            return true;

        case Cmd::BuildInfo:
            // Build date (11 chars): "Aug 13 2026"
            tx_frame.push_string("Aug 13 2026");
            // Build time (8 chars): "00:00:00"
            tx_frame.push_string("00:00:00");
            // Git short hash (7 chars)
            tx_frame.push_string("abcdef0");
            return true;

        case Cmd::Uid:
            // 12-byte unique ID (3 x uint32)
            tx_frame.push_u32(0x41535058); // "ASPX"
            tx_frame.push_u32(0x53495448); // "SITH" (SITL Host)
            tx_frame.push_u32(0x00000001); // Instance 1
            return true;

        // ---- Status & Sensor Data ----

        case Cmd::Status:
            tx_frame.push_u16(static_cast<uint16_t>(live.cycle_time_us));  // Cycle time (us)
            tx_frame.push_u16(live.i2c_error_count);                        // I2C error count
            tx_frame.push_u16(live.sensor_flags);                           // Sensor flags (ACC|BARO|MAG|GPS)
            tx_frame.push_u32(live.arming_flags);                           // Flight mode flags
            tx_frame.push_u8(live.profile_index);                           // Current PID profile
            return true;

        case Cmd::StatusEx:
            tx_frame.push_u16(static_cast<uint16_t>(live.cycle_time_us));
            tx_frame.push_u16(live.i2c_error_count);
            tx_frame.push_u16(live.sensor_flags);
            tx_frame.push_u32(live.arming_flags);
            tx_frame.push_u8(live.profile_index);
            tx_frame.push_u16(0);   // System load percent * 10
            tx_frame.push_u16(0);   // Config profile count
            tx_frame.push_u8(0);    // Rate profile index
            tx_frame.push_u8(0);    // Arming disable flags count
            return true;

        case Cmd::RawImu:
            tx_frame.push_u16(static_cast<uint16_t>(live.accel_x));
            tx_frame.push_u16(static_cast<uint16_t>(live.accel_y));
            tx_frame.push_u16(static_cast<uint16_t>(live.accel_z));
            tx_frame.push_u16(static_cast<uint16_t>(live.gyro_x));
            tx_frame.push_u16(static_cast<uint16_t>(live.gyro_y));
            tx_frame.push_u16(static_cast<uint16_t>(live.gyro_z));
            tx_frame.push_u16(static_cast<uint16_t>(live.mag_x));
            tx_frame.push_u16(static_cast<uint16_t>(live.mag_y));
            tx_frame.push_u16(static_cast<uint16_t>(live.mag_z));
            return true;

        case Cmd::Attitude:
            tx_frame.push_u16(static_cast<uint16_t>(live.roll_decideg));
            tx_frame.push_u16(static_cast<uint16_t>(live.pitch_decideg));
            tx_frame.push_u16(static_cast<uint16_t>(live.yaw_deg));
            return true;

        case Cmd::Altitude:
            tx_frame.push_u32(static_cast<uint32_t>(live.altitude_cm));
            tx_frame.push_u16(static_cast<uint16_t>(live.vario_cms));
            return true;

        case Cmd::Servo:
            // 8 servo values (all centered at 1500 for now)
            for (int i = 0; i < 8; ++i) {
                tx_frame.push_u16(1500);
            }
            return true;

        case Cmd::Motor:
            for (int i = 0; i < 8; ++i) {
                tx_frame.push_u16(live.motor_us[static_cast<size_t>(i)]);
            }
            return true;

        case Cmd::Analog:
            tx_frame.push_u8(static_cast<uint8_t>(live.vbat_mv / 100));  // vbat (0.1V units)
            tx_frame.push_u16(live.mah_drawn);                            // mAh drawn
            tx_frame.push_u16(live.rssi);                                  // RSSI
            tx_frame.push_u16(static_cast<uint16_t>(live.current_ma / 10)); // Current (0.01A)
            return true;

        case Cmd::Rc:
            for (int i = 0; i < 16; ++i) {
                tx_frame.push_u16(live.rc_channels[static_cast<size_t>(i)]);
            }
            return true;

        case Cmd::SetRawRc:
            if (rx_payload.size() >= 16) {
                // Parse 16-bit PWM channel overrides
                for (size_t i = 0; i < rx_payload.size() / 2 && i < 16; ++i) {
                    uint16_t pwm = static_cast<uint16_t>(rx_payload[i * 2] | (rx_payload[i * 2 + 1] << 8));
                    // Update global live RC channel overrides
                    const_cast<MspLiveState&>(live).rc_channels[i] = pwm;
                }
                return true;
            }
            return false;

        case Cmd::RcTuning:
            // RC rates (simplified — all zeros for SITL)
            for (int i = 0; i < 10; ++i) {
                tx_frame.push_u8(0);
            }
            return true;

        // ---- Configuration ----

        case Cmd::Pid:
            for (int i = 0; i < 3; ++i) {
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.kp[i] * 10.0f));
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.ki[i] * 10.0f));
                tx_frame.push_u8(static_cast<uint8_t>(config.pid.kd[i] * 1000.0f));
            }
            return true;

        case Cmd::SetPid:
            if (rx_payload.size() >= 9) {
                for (int i = 0; i < 3; ++i) {
                    config.pid.kp[i] = static_cast<float>(rx_payload[static_cast<size_t>(i * 3 + 0)]) / 10.0f;
                    config.pid.ki[i] = static_cast<float>(rx_payload[static_cast<size_t>(i * 3 + 1)]) / 10.0f;
                    config.pid.kd[i] = static_cast<float>(rx_payload[static_cast<size_t>(i * 3 + 2)]) / 1000.0f;
                }
                return true;
            }
            return false;

        case Cmd::Misc:
            tx_frame.push_u16(config.motor.min_throttle);
            tx_frame.push_u16(config.motor.max_throttle);
            tx_frame.push_u16(config.motor.min_command);
            tx_frame.push_u16(1500);  // Failsafe throttle
            tx_frame.push_u8(0);      // GPS type
            tx_frame.push_u8(0);      // GPS baudrate index
            tx_frame.push_u8(0);      // multiWiiCurrentMeterOutput
            tx_frame.push_u16(0);     // RSSI channel
            tx_frame.push_u8(0);      // placeholder
            tx_frame.push_u16(0);     // mag declination
            tx_frame.push_u8(0);      // vbat scale
            tx_frame.push_u8(33);     // vbat min cell (3.3V)
            tx_frame.push_u8(43);     // vbat max cell (4.3V)
            tx_frame.push_u8(34);     // vbat warning cell (3.4V)
            return true;

        case Cmd::SetMisc:
            if (rx_payload.size() >= 6) {
                config.motor.min_throttle = static_cast<uint16_t>(rx_payload[0] | (rx_payload[1] << 8));
                config.motor.max_throttle = static_cast<uint16_t>(rx_payload[2] | (rx_payload[3] << 8));
                config.motor.min_command  = static_cast<uint16_t>(rx_payload[4] | (rx_payload[5] << 8));
                return true;
            }
            return false;

        case Cmd::BoxNames:
            // Mode names separated by ';'
            tx_frame.push_string("ARM;ANGLE;HORIZON;NAV RTH;NAV POSHOLD;HEADING HOLD;");
            return true;

        case Cmd::BoxIds:
            // Box IDs matching the names above
            tx_frame.push_u8(0);   // ARM
            tx_frame.push_u8(1);   // ANGLE
            tx_frame.push_u8(2);   // HORIZON
            tx_frame.push_u8(10);  // NAV RTH
            tx_frame.push_u8(11);  // NAV POSHOLD
            tx_frame.push_u8(6);   // HEADING HOLD
            return true;

        case Cmd::AccCalibration:
            // Acknowledge accelerometer level calibration
            return true;

        case Cmd::MagCalibration:
            // Acknowledge compass calibration mode
            return true;

        case Cmd::SetMotor:
            if (rx_payload.size() >= 8) {
                // Parse 8x uint16_t motor pulse overrides for bench test
                for (size_t i = 0; i < rx_payload.size() / 2 && i < 8; ++i) {
                    uint16_t pwm = static_cast<uint16_t>(rx_payload[i * 2] | (rx_payload[i * 2 + 1] << 8));
                    const_cast<MspLiveState&>(live).motor_us[i] = pwm;
                }
                return true;
            }
            return false;

        case Cmd::EepromWrite:
            return ConfigRegistry::save_to_file("config.bin");

        case Cmd::Reboot:
            // In SITL, reboot is a no-op (just acknowledge)
            return true;


        // ---- Navigation ----

        case Cmd::RawGps:
            tx_frame.push_u8(1);    // Fix 3D
            tx_frame.push_u8(12);   // 12 satellites
            tx_frame.push_u32(377749000);  // Lat
            tx_frame.push_u32(4070773296U); // Lon (-1224194000)
            tx_frame.push_u16(10);  // Alt (m)
            tx_frame.push_u16(0);   // Speed (cm/s)
            tx_frame.push_u16(0);   // Course
            tx_frame.push_u16(120); // HDOP 1.20
            return true;

        case Cmd::SetRawGps:
            // Acknowledge MSP GPS injection payload
            return rx_payload.size() >= 16;

        case Cmd::NavStatus:
            tx_frame.push_u8(0);   // NAV mode
            tx_frame.push_u8(0);   // NAV state
            tx_frame.push_u8(0);   // NAV active WP action
            tx_frame.push_u8(0);   // NAV active WP number
            tx_frame.push_u8(0);   // NAV error
            tx_frame.push_u16(0);  // Heading hold target
            return true;

        case Cmd::CompGps:
            tx_frame.push_u16(0);  // Distance to home (m)
            tx_frame.push_u16(0);  // Direction to home (deg)
            tx_frame.push_u8(0);   // GPS update flag
            return true;

        case Cmd::WpGetInfo:
            tx_frame.push_u8(0);   // Max waypoints (0 = unlimited for SITL)
            tx_frame.push_u8(1);   // WP valid flag
            tx_frame.push_u8(0);   // WP count
            return true;

        // ---- ESC Passthrough ----
        case Cmd::Set4WayIf:
            tx_frame.push_u8(config.motor.motor_count);  // Number of ESCs
            return true;

        // ---- INAV EZ-Tune MSP Commands (fc_msp.c:1764 & 3603) ----
        case Cmd::EzTuneGet: {
            const auto* ez = flight::ezTune();
            tx_frame.push_u8(ez->enabled ? 1 : 0);
            tx_frame.push_u16(ez->filter_hz);
            tx_frame.push_u8(ez->axis_ratio);
            tx_frame.push_u8(ez->response);
            tx_frame.push_u8(ez->damping);
            tx_frame.push_u8(ez->stability);
            tx_frame.push_u8(ez->aggressiveness);
            tx_frame.push_u8(ez->rate);
            tx_frame.push_u8(ez->expo);
            tx_frame.push_u8(ez->snappiness);
            return true;
        }

        case Cmd::EzTuneSet: {
            if (rx_payload.size() >= 10) {
                auto* ez = flight::ezTuneMutable();
                ez->enabled = (rx_payload[0] != 0);
                ez->filter_hz = static_cast<uint16_t>(rx_payload[1] | (rx_payload[2] << 8));
                ez->axis_ratio = rx_payload[3];
                ez->response = rx_payload[4];
                ez->damping = rx_payload[5];
                ez->stability = rx_payload[6];
                ez->aggressiveness = rx_payload[7];
                ez->rate = rx_payload[8];
                ez->expo = rx_payload[9];
                if (rx_payload.size() >= 11) {
                    ez->snappiness = rx_payload[10];
                }
                auto prof = flight::ezTuneUpdate();
                config.pid.kp[0] = prof.pid_config.kp.roll;
                config.pid.kp[1] = prof.pid_config.kp.pitch;
                config.pid.kp[2] = prof.pid_config.kp.yaw;

                config.pid.ki[0] = prof.pid_config.ki.roll;
                config.pid.ki[1] = prof.pid_config.ki.pitch;
                config.pid.ki[2] = prof.pid_config.ki.yaw;

                config.pid.kd[0] = prof.pid_config.kd.roll;
                config.pid.kd[1] = prof.pid_config.kd.pitch;
                config.pid.kd[2] = prof.pid_config.kd.yaw;
                return true;

            }
            return false;
        }

        default:
            return false;
    }
}


} // namespace abstractx::msp
