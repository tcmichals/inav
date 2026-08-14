/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - NMEA 0183 ASCII Protocol Parser
 *
 * Parses $GNGGA / $GPGGA and $GNRMC / $GPRMC ASCII sentence streams.
 * Zero dynamic memory allocations. Stateful line buffer with XOR checksum validation.
 */

#ifndef NMEA_PARSER_HPP
#define NMEA_PARSER_HPP

#include "gps_types.hpp"
#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <cstring>
#include <cstdlib>

namespace abstractx::drivers::gps {

class NmeaParser {
public:
    constexpr NmeaParser() noexcept = default;

    void reset() noexcept {
        line_idx_ = 0;
        in_sentence_ = false;
    }

    // Process a single byte from stream. Returns true if a valid GpsSample was updated.
    bool parse_byte(uint8_t byte, uint64_t timestamp_ns = 0) noexcept {
        if (byte == '$') {
            line_idx_ = 0;
            line_buf_[line_idx_++] = byte;
            in_sentence_ = true;
            return false;
        }

        if (!in_sentence_) return false;

        if (byte == '\r' || byte == '\n') {
            in_sentence_ = false;
            line_buf_[line_idx_] = '\0';
            return parse_sentence(timestamp_ns);
        }

        if (line_idx_ < line_buf_.size() - 1) {
            line_buf_[line_idx_++] = byte;
        } else {
            in_sentence_ = false; // Buffer overflow, reject
        }

        return false;
    }

    const GpsSample& latest_sample() const noexcept { return sample_; }

private:
    std::array<char, 128> line_buf_{};
    size_t line_idx_{0};
    bool in_sentence_{false};
    GpsSample sample_{};

    // Calculate ASCII XOR checksum between '$' and '*'
    static bool validate_checksum(const char* line, size_t len) noexcept {
        if (len < 6 || line[0] != '$') return false;

        size_t star_pos = 0;
        for (size_t i = 1; i < len; ++i) {
            if (line[i] == '*') {
                star_pos = i;
                break;
            }
        }
        if (star_pos == 0 || star_pos + 2 >= len) return false;

        uint8_t expected_crc = static_cast<uint8_t>(
            std::strtoul(&line[star_pos + 1], nullptr, 16));

        uint8_t calc_crc = 0;
        for (size_t i = 1; i < star_pos; ++i) {
            calc_crc ^= static_cast<uint8_t>(line[i]);
        }

        return calc_crc == expected_crc;
    }

    // Convert NMEA lat/lon string (ddmm.mmmmm) to 1e-7 degrees
    static int32_t parse_coordinate(const char* token, char dir) noexcept {
        if (!token || *token == '\0') return 0;

        double val = std::strtod(token, nullptr);
        int deg = static_cast<int>(val / 100.0);
        double minutes = val - (deg * 100.0);
        double deg_decimal = static_cast<double>(deg) + (minutes / 60.0);

        int32_t res_1e7 = static_cast<int32_t>(deg_decimal * 1e7);
        if (dir == 'S' || dir == 'W') res_1e7 = -res_1e7;
        return res_1e7;
    }

    bool parse_sentence(uint64_t timestamp_ns) noexcept {
        size_t len = std::strlen(line_buf_.data());
        if (!validate_checksum(line_buf_.data(), len)) return false;

        // Tokenize by comma
        std::array<const char*, 20> tokens{};
        size_t token_cnt = 0;

        char* ptr = line_buf_.data();
        tokens[token_cnt++] = ptr;

        for (size_t i = 0; i < len; ++i) {
            if (ptr[i] == ',' || ptr[i] == '*') {
                ptr[i] = '\0';
                if (token_cnt < tokens.size()) {
                    tokens[token_cnt++] = &ptr[i + 1];
                }
            }
        }

        if (token_cnt < 2) return false;

        // $GNGGA / $GPGGA parser
        if (std::strstr(tokens[0], "GGA") != nullptr && token_cnt >= 10) {
            sample_.timestamp_ns = timestamp_ns;

            int32_t lat = parse_coordinate(tokens[2], tokens[3][0]);
            int32_t lon = parse_coordinate(tokens[4], tokens[5][0]);

            uint8_t fix_qual = static_cast<uint8_t>(std::atoi(tokens[6]));
            uint8_t sats = static_cast<uint8_t>(std::atoi(tokens[7]));
            double hdop_val = std::strtod(tokens[8], nullptr);
            double alt_val = std::strtod(tokens[9], nullptr);

            sample_.latitude_1e7 = lat;
            sample_.longitude_1e7 = lon;
            sample_.altitude_cm = static_cast<int32_t>(alt_val * 100.0); // m to cm
            sample_.num_sats = sats;
            sample_.hdop_centi = static_cast<uint16_t>(hdop_val * 100.0);

            if (fix_qual == 1) {
                sample_.fix_type = GpsFixType::Fix3D;
            } else if (fix_qual == 2) {
                sample_.fix_type = GpsFixType::Fix3DDgps;
            } else {
                sample_.fix_type = GpsFixType::NoFix;
            }

            sample_.valid = (sample_.fix_type != GpsFixType::NoFix);
            return sample_.valid;
        }

        // $GNRMC / $GPRMC parser
        if (std::strstr(tokens[0], "RMC") != nullptr && token_cnt >= 9) {
            if (tokens[2][0] == 'A') { // Status: A = Valid
                double speed_knots = std::strtod(tokens[7], nullptr);
                double course_deg = std::strtod(tokens[8], nullptr);

                int32_t speed_cms = static_cast<int32_t>(speed_knots * 51.4444);
                uint16_t course_decideg = static_cast<uint16_t>(course_deg * 10.0);

                sample_.ground_course_decideg = course_decideg;
                // Calculate North/East velocity components from ground speed & heading
                double rad = course_deg * (3.14159265358979323846 / 180.0);
                sample_.vel_n_cms = static_cast<int16_t>(speed_cms * std::cos(rad));
                sample_.vel_e_cms = static_cast<int16_t>(speed_cms * std::sin(rad));
            }
            return false;
        }

        return false;
    }
};

} // namespace abstractx::drivers::gps

#endif // NMEA_PARSER_HPP
