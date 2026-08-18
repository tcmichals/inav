/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - RP2350 Pico 2 W Platform Target Implementation
 *
 * Core 0 peripherals:
 *   - SPI1 bus init (GP10/11/12/13) for ICM-42688P at 24 MHz
 *   - I2C1 bus init (GP6/GP7) for BMP280 + QMC5883L at 400 kHz
 *   - UART0 init (GP0/GP1) for GPS at 115200 baud
 *   - GP14 GPIO IRQ for IMU DRDY edge timestamp latch
 *
 * All pin assignments loaded from ConfigRegistry::get().sensor at runtime.
 */

#include "pico2_target.hpp"
#include "config_registry.hpp"
#include "bus_concepts.hpp"
#include <chrono>

#if defined(PICO_BOARD)
#  include "hardware/spi.h"
#  include "hardware/i2c.h"
#  include "hardware/uart.h"
#  include "hardware/gpio.h"
#  include "pico/time.h"
#endif

namespace abstractx::target::pico2 {

// ---------------------------------------------------------------------------
// Singleton bus instances constructed from SensorConfig defaults.
// Constructed once on first use; zero dynamic allocation.
// ---------------------------------------------------------------------------
static drivers::bus::Pico2SpiBus& imu_spi_bus() noexcept {
    const auto& sc = ConfigRegistry::get().sensor;
    static drivers::bus::Pico2SpiBus instance{sc.imu_spi};
    return instance;
}

static drivers::bus::Pico2I2cBus& baro_mag_i2c_bus() noexcept {
    const auto& sc = ConfigRegistry::get().sensor;
    static drivers::bus::Pico2I2cBus instance{sc.baro_mag_i2c};
    return instance;
}

// ---------------------------------------------------------------------------
// Core 0: Init all physical buses from SensorConfig (called before multicore)
// ---------------------------------------------------------------------------
void Pico2Target::init_core0_peripherals() noexcept {
    const auto& sc = ConfigRegistry::get().sensor;

    // 1. SPI1 — IMU bus (24 MHz, mode 3: CPOL=1 CPHA=1)
    (void)imu_spi_bus().init();

    // 2. I2C1 — Barometer + Magnetometer (400 kHz Fast-Mode)
    (void)baro_mag_i2c_bus().init();

    // 3. UART0 — GPS (115200 baud UBX/NMEA)
#if defined(PICO_BOARD)
    uart_init(uart0, sc.gps_baud);
    gpio_set_function(sc.gps_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(sc.gps_rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart0, 8u, 1u, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
#else
    (void)sc;
#endif

    // 4. GP14 — IMU DRDY: configure as input with falling-edge IRQ
    //    The DrdyAwaiter (or PIO DMA IRQ handler on RP2350) latches
    //    a 64-bit nanosecond timestamp via time_us_64() × 1000 on this edge.
#if defined(PICO_BOARD)
    gpio_init(sc.imu_spi.drdy_pin);
    gpio_set_dir(sc.imu_spi.drdy_pin, GPIO_IN);
    gpio_pull_down(sc.imu_spi.drdy_pin); // active-high INT1
    // IRQ registered in imu bottom-half sample_loop on RP2350 PIO path
#endif
}

void Pico2Target::init_core1_flight_loop() noexcept {
    // Core 1 is launched via multicore_launch_core1() in pico2_main.cpp.
    // The flight engine coroutine is started there — nothing needed here.
}

uint64_t Pico2Target::get_hardware_timestamp_ns() noexcept {
#if defined(PICO_BOARD)
    return static_cast<uint64_t>(time_us_64()) * 1000u;
#else
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
#endif
}

bool Pico2Target::flash_write(uint32_t offset,
                               std::span<const uint8_t> data) noexcept {
#if defined(PICO_BOARD)
    // RP2350 QSPI flash page-program (256-byte aligned writes)
    // flash_range_erase / flash_range_program from pico-sdk
    (void)offset; (void)data;
    // TODO: implement flash_range_program wrapper
    return true;
#else
    (void)offset; (void)data;
    return true;
#endif
}

bool Pico2Target::flash_read(uint32_t offset,
                              std::span<uint8_t> data) noexcept {
#if defined(PICO_BOARD)
    // RP2350 XIP base: 0x10000000 + offset (memory-mapped read, no SPI needed)
    static constexpr uint32_t FLASH_XIP_BASE = 0x10000000u;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(FLASH_XIP_BASE + offset);
    for (size_t i = 0u; i < data.size(); ++i) { data[i] = src[i]; }
    return true;
#else
    (void)offset; (void)data;
    return true;
#endif
}

// ---------------------------------------------------------------------------
// Accessors for main.cpp to obtain the singleton buses (no new/malloc)
// ---------------------------------------------------------------------------
drivers::bus::Pico2SpiBus& Pico2Target::get_imu_spi_bus() noexcept {
    return imu_spi_bus();
}

drivers::bus::Pico2I2cBus& Pico2Target::get_baro_mag_i2c_bus() noexcept {
    return baro_mag_i2c_bus();
}

} // namespace abstractx::target::pico2
