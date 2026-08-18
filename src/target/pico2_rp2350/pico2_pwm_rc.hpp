/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - 6-Channel Parallel PWM RC Receiver Driver for RP2350 Pico 2 W
 *
 * Pin Assignment (6 Inputs):
 *   - GP8  (Pin 11): CH1 Roll / Aileron
 *   - GP9  (Pin 12): CH2 Pitch / Elevator
 *   - GP16 (Pin 21): CH3 Throttle
 *   - GP17 (Pin 22): CH4 Yaw / Rudder
 *   - GP18 (Pin 24): CH5 Aux 1 (Arm Switch)
 *   - GP19 (Pin 25): CH6 Aux 2 (Flight Mode)
 *
 * Captures 50 Hz PWM servo pulse widths (1000-2000 µs) with edge timestamps.
 */

#ifndef PICO2_PWM_RC_HPP
#define PICO2_PWM_RC_HPP

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "spsc_tlp_ring.hpp"
#include "pcie_bar_map.hpp"
#include "asp_tlp64.hpp"
#include <array>
#include <cstdint>
#include <algorithm>

namespace abstractx::target::pico2 {

class Pico2PwmReceiver {
public:
    static constexpr size_t NUM_CHANNELS = 6;
    static constexpr std::array<uint, NUM_CHANNELS> PINS = {8u, 9u, 16u, 17u, 18u, 19u};

    static void init() noexcept {
        for (size_t i = 0; i < NUM_CHANNELS; ++i) {
            uint pin = PINS[i];
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_down(pin);
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }

        gpio_set_irq_callback(&gpio_irq_handler);
        irq_set_enabled(IO_IRQ_BANK0, true);
    }

    [[nodiscard]] static bool is_connected() noexcept {
        uint64_t now = time_us_64();
        // Failsafe triggers if no throttle pulse seen within 100 ms (10 missed 50Hz frames)
        return (now - last_pulse_us_[2] < 100000ULL) && (channels_[2] >= 850 && channels_[2] <= 2150);
    }

    [[nodiscard]] static uint16_t get_channel(size_t ch) noexcept {
        if (ch >= NUM_CHANNELS) { return 1500u; }
        return channels_[ch];
    }

    static void poll_and_push_tlp(SpscTlpRing<64>& ring, uint8_t tag) noexcept {
        Tlp64 tlp = Tlp64::make_mem_write(bar::RcBase, 0u, tag);
        tlp.wire.timestamp_ns = static_cast<uint64_t>(time_us_64()) * 1000u;

        for (size_t i = 0; i < NUM_CHANNELS; ++i) {
            uint16_t val = is_connected() ? channels_[i] : (i == 2 ? 900u : 1500u); // Throttle min on failsafe
            tlp.wire.payload[i * 2]     = static_cast<uint8_t>(val >> 8);
            tlp.wire.payload[i * 2 + 1] = static_cast<uint8_t>(val & 0xFF);
        }

        (void)ring.push(tlp);
    }

private:
    static inline volatile uint32_t rise_time_us_[NUM_CHANNELS] = {0};
    static inline volatile uint16_t channels_[NUM_CHANNELS]     = {1500, 1500, 1000, 1500, 1000, 1000};
    static inline volatile uint64_t last_pulse_us_[NUM_CHANNELS] = {0};

    static void gpio_irq_handler(uint gpio, uint32_t events) noexcept {
        uint64_t now_us = time_us_64();

        for (size_t i = 0; i < NUM_CHANNELS; ++i) {
            if (PINS[i] == gpio) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    rise_time_us_[i] = static_cast<uint32_t>(now_us);
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    uint32_t pulse = static_cast<uint32_t>(now_us) - rise_time_us_[i];
                    // Glitch filter: standard RC pulse is 900-2100 µs
                    if (pulse >= 800 && pulse <= 2200) {
                        channels_[i] = static_cast<uint16_t>(pulse);
                        last_pulse_us_[i] = now_us;
                    }
                }
                break;
            }
        }
    }
};

} // namespace abstractx::target::pico2

#endif // PICO2_PWM_RC_HPP
