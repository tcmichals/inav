/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Status Indicator LEDs & WS2812 Light Controller Driver
 */

#ifndef LED_STRIP_DRIVER_HPP
#define LED_STRIP_DRIVER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>
#include <array>

namespace abstractx::drivers {

// Standard Onboard Indicator LEDs
enum class StatusLed : uint8_t {
    Armed       = 0, // Red Status LED (Solid = Armed, Flash = Disarmed)
    GpsFix      = 1, // Blue GPS LED (Solid = 3D Lock, Flash = Searching)
    RcConnected = 2, // Green Link LED (Solid = RC Connected, Off = Failsafe)
    Warning     = 3  // Yellow Warning / Error LED
};

struct RgbColor {
    uint8_t r{0};
    uint8_t g{0};
    uint8_t b{0};

    static constexpr RgbColor Red()    { return {255, 0, 0}; }
    static constexpr RgbColor Green()  { return {0, 255, 0}; }
    static constexpr RgbColor Blue()   { return {0, 0, 255}; }
    static constexpr RgbColor Yellow() { return {255, 255, 0}; }
    static constexpr RgbColor Cyan()   { return {0, 255, 255}; }
    static constexpr RgbColor Off()    { return {0, 0, 0}; }
};

template <size_t LedCount = 32>
class LedStripDriver {
public:
    constexpr LedStripDriver() noexcept = default;

    void set_led(size_t index, RgbColor color) noexcept {
        if (index < LedCount) {
            leds_[index] = color;
        }
    }

    // Automatically update LED strip colors based on flight status (Armed, GPS, RC Link, Nav Mode)
    void update_status_indicators(bool armed, bool gps_fix, bool rc_connected, uint8_t nav_mode) noexcept {
        // LED 0: Armed Indicator (Red = Armed, Green = Disarmed)
        set_led(0, armed ? RgbColor::Red() : RgbColor::Green());

        // LED 1: GPS Fix Indicator (Blue = 3D Lock, Yellow = Searching)
        set_led(1, gps_fix ? RgbColor::Blue() : RgbColor::Yellow());

        // LED 2: RC Connection Indicator (Green = Connected, Red = Failsafe)
        set_led(2, rc_connected ? RgbColor::Green() : RgbColor::Red());

        // LED 3: Navigation Mode Indicator (Cyan = RTH / Waypoint)
        set_led(3, (nav_mode > 0) ? RgbColor::Cyan() : RgbColor::Off());
    }

    // Build 64B Memory Write TLP to update LED hardware over Virtual BAR
    Tlp64 make_update_tlp(size_t led_index, uint8_t tag) const noexcept {
        uint32_t addr = bar::SystemBase + 0xC00 + static_cast<uint32_t>(led_index * 4);
        uint32_t color_val = (static_cast<uint32_t>(leds_[led_index].r) << 16) |
                             (static_cast<uint32_t>(leds_[led_index].g) << 8) |
                             (static_cast<uint32_t>(leds_[led_index].b));
        return Tlp64::make_mem_write(addr, color_val, tag);
    }

private:
    std::array<RgbColor, LedCount> leds_{};
};

} // namespace abstractx::drivers

#endif // LED_STRIP_DRIVER_HPP
