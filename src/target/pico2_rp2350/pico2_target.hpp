/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Raspberry Pi RP2350 Pico 2 Target Platform Adapter
 */

#ifndef PICO2_TARGET_HPP
#define PICO2_TARGET_HPP

#include "target_interface.hpp"
#include "flash_storage.hpp"
#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>
#include <string_view>
#include <span>

namespace abstractx::target::pico2 {

constexpr uint32_t PICO2_FLASH_CONFIG_OFFSET = 0x1F0000;

class Pico2Target {
public:
    std::string_view name() const noexcept { return "RP2350 Pico 2 (Dual Cortex-M33 + PIO)"; }

    bool init() noexcept {
        init_core0_peripherals();
        return true;
    }

    uint32_t reg_read32(uint32_t /*addr*/) noexcept {
        return 0;
    }

    void reg_write32(uint32_t /*addr*/, uint32_t /*val*/) noexcept {}

    void process_telemetry(SpscTlpRing<64>& /*ring*/) noexcept {}

    static void init_core0_peripherals() noexcept;
    static void init_core1_flight_loop() noexcept;

    static uint64_t get_hardware_timestamp_ns() noexcept;
    static bool flash_write(uint32_t offset, std::span<const uint8_t> data) noexcept;
    static bool flash_read(uint32_t offset, std::span<uint8_t> data) noexcept;
};

static_assert(concepts::IsPlatform<Pico2Target>, "Pico2Target must satisfy IsPlatform concept");

} // namespace abstractx::target::pico2

#endif // PICO2_TARGET_HPP
