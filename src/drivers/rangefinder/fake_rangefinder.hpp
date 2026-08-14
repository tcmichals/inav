/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Synthetic / Simulated Rangefinder Driver
 */

#ifndef DRIVERS_FAKE_RANGEFINDER_HPP
#define DRIVERS_FAKE_RANGEFINDER_HPP

#include "rangefinder_base.hpp"

namespace abstractx::drivers {

class FakeRangefinder : public RangefinderDriverBase {
public:
    constexpr explicit FakeRangefinder(const RangefinderConfig& config = RangefinderConfig{}) noexcept
        : RangefinderDriverBase(config) {}

    RangefinderSample update_simulated(float true_alt_agl_m, float roll_deg, float pitch_deg, uint64_t timestamp_us) noexcept {
        float cos_r = std::cos(roll_deg * (3.14159265f / 180.0f));
        float cos_p = std::cos(pitch_deg * (3.14159265f / 180.0f));

        float cos_tilt = cos_r * cos_p;
        float slant_range_m = (cos_tilt > 0.1f) ? (true_alt_agl_m / cos_tilt) : true_alt_agl_m;

        return process(slant_range_m, cos_r, cos_p, timestamp_us);
    }
};

} // namespace abstractx::drivers

#endif // DRIVERS_FAKE_RANGEFINDER_HPP
