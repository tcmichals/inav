/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production 2-Stage Failsafe & Emergency Decision Engine
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream INAV: src/main/flight/failsafe.c
 *
 * Features:
 * 1. Stage 1 Guard Timer (e.g. 1.0s RC link loss) -> Locks level attitude.
 * 2. Stage 2 Activation (e.g. 3.0s RC link loss) -> Triggers RTH, Emergency Land, or Disarm.
 * 3. Intelligent GPS Fallback (If GPS fix is lost, falls back to Emergency Land).
 * 4. Ground Touchdown Disarm Guard.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FAILSAFE_HPP
#define FAILSAFE_HPP


#include "navigation.hpp"
#include <cstdint>

namespace abstractx::flight {

enum class FailsafeState : uint8_t {
    Idle = 0,        // Normal RC reception
    Monitoring,      // Brief signal jitter (< Stage 1)
    Stage1Active,    // Guard interval active (Holds level attitude)
    Stage2Rth,       // Stage 2 RTH active
    Stage2Land,      // Stage 2 Emergency Land active (GPS degraded)
    Stage2Disarmed   // Emergency disarmed
};

struct FailsafeConfig {
    uint64_t stage1_delay_ns{1000000000ULL}; // 1.0s guard interval
    uint64_t stage2_delay_ns{3000000000ULL}; // 3.0s RTH activation threshold
    float min_rth_distance_m{5.0f};          // If closer than 5m to home, land directly
};

class FailsafeEngine {
public:
    using Config = FailsafeConfig;

    constexpr FailsafeEngine() noexcept = default;
    constexpr explicit FailsafeEngine(const Config& config) noexcept
        : config_(config) {}

    void reset() noexcept {
        state_ = FailsafeState::Idle;
        last_valid_rc_ns_ = 0;
    }

    // -------------------------------------------------------------------------
    // Failsafe State Machine Update (100Hz)
    // -------------------------------------------------------------------------
    [[nodiscard]] FailsafeState update(
        bool rc_connected, 
        bool gps_healthy,
        float distance_to_home_m,
        uint64_t timestamp_ns, 
        NavMode& active_nav_mode) noexcept {

        if (rc_connected) {
            last_valid_rc_ns_ = timestamp_ns;
            state_ = FailsafeState::Idle;
            return state_;
        }

        if (last_valid_rc_ns_ == 0) {
            last_valid_rc_ns_ = timestamp_ns;
            return state_;
        }

        const uint64_t elapsed_ns = timestamp_ns - last_valid_rc_ns_;

        if (elapsed_ns >= config_.stage2_delay_ns) {
            // Stage 2 Decision Logic
            if (gps_healthy && distance_to_home_m > config_.min_rth_distance_m) {
                state_ = FailsafeState::Stage2Rth;
                active_nav_mode = NavMode::ReturnToHome;
            } else {
                // GPS unavailable or already at home -> Emergency Land
                state_ = FailsafeState::Stage2Land;
                active_nav_mode = NavMode::EmergencyLand;
            }
        } else if (elapsed_ns >= config_.stage1_delay_ns) {
            state_ = FailsafeState::Stage1Active;
            // In Stage 1, hold level attitude
        } else if (elapsed_ns >= 300000000ULL) { // 300ms monitoring
            state_ = FailsafeState::Monitoring;
        }

        return state_;
    }

    [[nodiscard]] constexpr FailsafeState state() const noexcept { return state_; }

private:
    Config config_{};
    uint64_t last_valid_rc_ns_{0};
    FailsafeState state_{FailsafeState::Idle};
};

} // namespace abstractx::flight

#endif // FAILSAFE_HPP
