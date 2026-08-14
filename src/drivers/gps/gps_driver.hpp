/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Unified GPS Driver Engine & Auto-Baudrate Manager
 *
 * Handles UBX, NMEA, and MSP GPS data streams. Manages initialization
 * and auto-baudrate scanning sequence (9600 -> 19200 -> 38400 -> 57600 -> 115200).
 */

#ifndef GPS_DRIVER_HPP
#define GPS_DRIVER_HPP

#include "gps_types.hpp"
#include "ubx_parser.hpp"
#include "nmea_parser.hpp"
#include <span>
#include <array>
#include <optional>

namespace abstractx::drivers::gps {

// Baudrate scan sequence matching iNav / Betaflight
static constexpr std::array<uint32_t, 5> GPS_BAUDRATES{9600, 19200, 38400, 57600, 115200};

class GpsDriver {
public:
    explicit GpsDriver(GpsProvider provider = GpsProvider::Ublox) noexcept
        : provider_(provider) {}

    void set_provider(GpsProvider provider) noexcept {
        provider_ = provider;
        reset();
    }

    void reset() noexcept {
        ubx_parser_.reset();
        nmea_parser_.reset();
        baud_idx_ = 0;
        state_ = InitState::ScanningBaud;
        sample_count_ = 0;
    }

    // Process incoming raw bytes from UART or SITL stream
    std::optional<GpsSample> process_bytes(std::span<const uint8_t> data, uint64_t timestamp_ns = 0) noexcept {
        std::optional<GpsSample> result = std::nullopt;

        for (uint8_t b : data) {
            bool sample_ready = false;

            if (provider_ == GpsProvider::Ublox) {
                sample_ready = ubx_parser_.parse_byte(b, timestamp_ns);
                if (sample_ready) {
                    latest_sample_ = ubx_parser_.latest_sample();
                    result = latest_sample_;
                    sample_count_++;
                    state_ = InitState::Configured;
                }
            } else if (provider_ == GpsProvider::Nmea) {
                sample_ready = nmea_parser_.parse_byte(b, timestamp_ns);
                if (sample_ready) {
                    latest_sample_ = nmea_parser_.latest_sample();
                    result = latest_sample_;
                    sample_count_++;
                    state_ = InitState::Configured;
                }
            }
        }

        return result;
    }

    // Inject manual MSP GPS frame (from MSP_SET_RAW_GPS command 201)
    void inject_msp_gps(const GpsSample& sample) noexcept {
        latest_sample_ = sample;
        latest_sample_.valid = (sample.fix_type != GpsFixType::NoFix);
        sample_count_++;
        state_ = InitState::Configured;
    }

    const GpsSample& latest_sample() const noexcept { return latest_sample_; }
    GpsProvider provider() const noexcept { return provider_; }
    uint32_t current_baudrate() const noexcept { return GPS_BAUDRATES[baud_idx_]; }

    // Advance to next baudrate in scan cycle
    uint32_t next_baudrate() noexcept {
        baud_idx_ = (baud_idx_ + 1) % GPS_BAUDRATES.size();
        return GPS_BAUDRATES[baud_idx_];
    }

    uint64_t sample_count() const noexcept { return sample_count_; }

private:
    enum class InitState : uint8_t {
        ScanningBaud,
        SendingConfig,
        Configured
    };

    GpsProvider provider_{GpsProvider::Ublox};
    InitState state_{InitState::ScanningBaud};
    size_t baud_idx_{0};
    uint64_t sample_count_{0};

    UbxParser ubx_parser_{};
    NmeaParser nmea_parser_{};
    GpsSample latest_sample_{};
};

} // namespace abstractx::drivers::gps

#endif // GPS_DRIVER_HPP
