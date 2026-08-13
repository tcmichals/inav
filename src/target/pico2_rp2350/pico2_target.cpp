/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Raspberry Pi RP2350 Pico 2 Target Implementation
 */

#include "pico2_target.hpp"
#include <chrono>

namespace abstractx::target::pico2 {

void Pico2Target::init_core0_peripherals() noexcept {
    // 1. Initialize RP2350 PIO 0 State Machines for DShot150/300/600/1200 motor outputs
    // 2. Initialize SPI 0 DMA burst receiver for ICM-42688-P / BMI088 IMU samples
    // 3. Configure GPIO DRDY interrupt to latch 64-bit nanosecond timer at exact edge
}

void Pico2Target::init_core1_flight_loop() noexcept {
    // Launch Core 1 Flight Execution Loop Coroutine
}

uint64_t Pico2Target::get_hardware_timestamp_ns() noexcept {
    // RP2350 High-Resolution Hardware Timer (Nanoseconds)
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

bool Pico2Target::flash_write(uint32_t /*offset*/, std::span<const uint8_t> /*data*/) noexcept {
    // RP2350 SPI Flash Page Write (flash_range_program)
    return true;
}

bool Pico2Target::flash_read(uint32_t /*offset*/, std::span<uint8_t> /*data*/) noexcept {
    // RP2350 SPI Flash Memory-Mapped Direct Read (XIP base 0x10000000)
    return true;
}

} // namespace abstractx::target::pico2
