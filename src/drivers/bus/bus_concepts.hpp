/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Hardware Bus Concept Layer
 *
 * C++20 concepts for SPI and I2C buses + three concrete implementations:
 *   Pico2SpiBus  — RP2350 hardware_spi + GPIO CS   (real hardware)
 *   FakeSpiBus   — SITL / CppUTest injectable bytes
 *   Pico2I2cBus  — RP2350 hardware_i2c              (real hardware)
 *   FakeI2cBus   — SITL / CppUTest injectable bytes
 *
 * Rules (MISRA C++:2023 / NASA Power-of-10):
 *   - Zero dynamic allocation in flight path
 *   - All public functions [[nodiscard]] noexcept
 *   - Fixed-width types only (uint8_t, uint32_t)
 *   - std::span for all buffer arguments (no raw pointers)
 *   - No recursion; all loops have fixed upper bound
 */

#ifndef BUS_CONCEPTS_HPP
#define BUS_CONCEPTS_HPP

#include "coroutine_task.hpp"
#include <cstdint>
#include <span>
#include <concepts>
#include <array>
#include <algorithm>

#if defined(PICO_BOARD)
#  include "hardware/spi.h"
#  include "hardware/i2c.h"
#  include "hardware/gpio.h"
#  include "pico/time.h"
#endif

namespace abstractx::drivers::bus {

// ---------------------------------------------------------------------------
// Pin / Baud configuration (populated from MasterConfig on boot)
// ---------------------------------------------------------------------------

struct SpiPinConfig {
    uint8_t  sck_pin{10u};       // GP10 — SPI1 SCK
    uint8_t  mosi_pin{11u};      // GP11 — SPI1 MOSI
    uint8_t  miso_pin{12u};      // GP12 — SPI1 MISO
    uint8_t  cs_pin{13u};        // GP13 — IMU CS  (active low)
    uint8_t  drdy_pin{14u};      // GP14 — IMU INT1 DRDY
    uint32_t baud_hz{24000000u}; // 24 MHz for ICM-42688P
};

struct I2cPinConfig {
    uint8_t  sda_pin{6u};        // GP6  — I2C1 SDA (Baro + Mag)
    uint8_t  scl_pin{7u};        // GP7  — I2C1 SCL
    uint32_t baud_hz{400000u};   // 400 kHz Fast-Mode
};

// ---------------------------------------------------------------------------
// C++20 Concept: IsSpiBus
// ---------------------------------------------------------------------------
template <typename T>
concept IsSpiBus = requires(
    T& bus, uint8_t reg, uint8_t val,
    std::span<uint8_t> rx, std::span<const uint8_t> tx, uint32_t ms)
{
    { bus.write_reg(reg, val) }              -> std::same_as<bool>;
    { bus.read_reg(reg) }                    -> std::same_as<uint8_t>;
    { bus.read_burst(reg, rx) }              -> std::same_as<bool>;
    { bus.transfer(tx, rx) }                 -> std::same_as<bool>;
    { bus.delay_ms(ms) }                     -> std::same_as<void>;
    { bus.async_write_reg(reg, val) }        -> std::same_as<Task<bool>>;
    { bus.async_read_reg(reg) }              -> std::same_as<Task<uint8_t>>;
    { bus.async_read_burst(reg, rx) }        -> std::same_as<Task<bool>>;
};

// ---------------------------------------------------------------------------
// C++20 Concept: IsI2cBus
// ---------------------------------------------------------------------------
template <typename T>
concept IsI2cBus = requires(
    T& bus, uint8_t dev, uint8_t reg, uint8_t val,
    std::span<uint8_t> rx, std::span<const uint8_t> tx, uint32_t ms)
{
    { bus.write_reg(dev, reg, val) }         -> std::same_as<bool>;
    { bus.read_reg(dev, reg) }               -> std::same_as<uint8_t>;
    { bus.read_regs(dev, reg, rx) }          -> std::same_as<bool>;
    { bus.write_bytes(dev, tx) }             -> std::same_as<bool>;
    { bus.delay_ms(ms) }                     -> std::same_as<void>;
    { bus.async_write_reg(dev, reg, val) }   -> std::same_as<Task<bool>>;
    { bus.async_read_reg(dev, reg) }         -> std::same_as<Task<uint8_t>>;
    { bus.async_read_regs(dev, reg, rx) }    -> std::same_as<Task<bool>>;
    { bus.async_write_bytes(dev, tx) }       -> std::same_as<Task<bool>>;
};

// ===========================================================================
// Pico2SpiBus — RP2350 hardware_spi peripheral
// ===========================================================================
class Pico2SpiBus {
public:
    explicit constexpr Pico2SpiBus(const SpiPinConfig& cfg) noexcept : cfg_{cfg} {}

    [[nodiscard]] bool init() noexcept {
#if defined(PICO_BOARD)
        spi_init(spi1, cfg_.baud_hz);
        spi_set_format(spi1, 8u, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        gpio_set_function(cfg_.sck_pin,  GPIO_FUNC_SPI);
        gpio_set_function(cfg_.mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(cfg_.miso_pin, GPIO_FUNC_SPI);
        gpio_init(cfg_.cs_pin);
        gpio_set_dir(cfg_.cs_pin, GPIO_OUT);
        gpio_put(cfg_.cs_pin, 1u); // deassert CS
#endif
        return true;
    }

    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val) noexcept {
#if defined(PICO_BOARD)
        uint8_t buf[2] = { static_cast<uint8_t>(reg & 0x7Fu), val };
        cs_select();
        spi_write_blocking(spi1, buf, 2u);
        cs_deselect();
#else
        (void)reg; (void)val;
#endif
        return true;
    }

    [[nodiscard]] Task<bool> async_write_reg(uint8_t reg, uint8_t val) noexcept {
        co_return write_reg(reg, val);
    }

    [[nodiscard]] uint8_t read_reg(uint8_t reg) noexcept {
#if defined(PICO_BOARD)
        uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80u), 0x00u };
        uint8_t rx[2] = { 0x00u, 0x00u };
        cs_select();
        spi_write_read_blocking(spi1, tx, rx, 2u);
        cs_deselect();
        return rx[1];
#else
        (void)reg;
        return fake_rx_;
#endif
    }

    [[nodiscard]] Task<uint8_t> async_read_reg(uint8_t reg) noexcept {
        co_return read_reg(reg);
    }

    [[nodiscard]] bool read_burst(uint8_t reg, std::span<uint8_t> rx) noexcept {
        if (rx.empty() || rx.size() > 64u) { return false; }
#if defined(PICO_BOARD)
        uint8_t cmd = static_cast<uint8_t>(reg | 0x80u);
        cs_select();
        spi_write_blocking(spi1, &cmd, 1u);
        spi_read_blocking(spi1, 0x00u, rx.data(), rx.size());
        cs_deselect();
#else
        (void)reg;
        std::fill(rx.begin(), rx.end(), fake_rx_);
#endif
        return true;
    }

    [[nodiscard]] Task<bool> async_read_burst(uint8_t reg, std::span<uint8_t> rx) noexcept {
        co_return read_burst(reg, rx);
    }

    [[nodiscard]] bool transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept {
        if (tx.size() != rx.size() || tx.empty()) { return false; }
#if defined(PICO_BOARD)
        cs_select();
        spi_write_read_blocking(spi1, tx.data(), rx.data(), tx.size());
        cs_deselect();
#else
        (void)tx;
        std::fill(rx.begin(), rx.end(), fake_rx_);
#endif
        return true;
    }

    void delay_ms(uint32_t ms) noexcept {
#if defined(PICO_BOARD)
        sleep_ms(ms);
#else
        (void)ms;
#endif
    }

    // SITL / test injection only
    void set_fake_rx(uint8_t b) noexcept { fake_rx_ = b; }

    [[nodiscard]] uint8_t drdy_pin() const noexcept { return cfg_.drdy_pin; }

private:
    SpiPinConfig cfg_{};
    uint8_t      fake_rx_{0x00u};

    void cs_select()   noexcept {
#if defined(PICO_BOARD)
        gpio_put(cfg_.cs_pin, 0u);
#endif
    }
    void cs_deselect() noexcept {
#if defined(PICO_BOARD)
        gpio_put(cfg_.cs_pin, 1u);
#endif
    }
};
static_assert(IsSpiBus<Pico2SpiBus>);

// ===========================================================================
// FakeSpiBus — SITL / CppUTest injectable bus
// ===========================================================================
class FakeSpiBus {
public:
    void inject(std::span<const uint8_t> data) noexcept {
        size_t n = std::min(data.size(), buf_.size());
        for (size_t i = 0u; i < n; ++i) { buf_[i] = data[i]; }
        len_ = static_cast<uint8_t>(n);
        pos_ = 0u;
    }
    void inject_byte(uint8_t b) noexcept { buf_[0]=b; len_=1u; pos_=0u; }

    [[nodiscard]] bool    write_reg(uint8_t, uint8_t) noexcept  { return true; }
    [[nodiscard]] uint8_t read_reg(uint8_t) noexcept            { return next(); }
    [[nodiscard]] bool read_burst(uint8_t, std::span<uint8_t> rx) noexcept {
        for (auto& b : rx) { b = next(); } return true;
    }
    [[nodiscard]] bool transfer(std::span<const uint8_t>, std::span<uint8_t> rx) noexcept {
        for (auto& b : rx) { b = next(); } return true;
    }
    void delay_ms(uint32_t) noexcept {}

    [[nodiscard]] Task<bool> async_write_reg(uint8_t, uint8_t) noexcept { co_return true; }
    [[nodiscard]] Task<uint8_t> async_read_reg(uint8_t) noexcept { co_return next(); }
    [[nodiscard]] Task<bool> async_read_burst(uint8_t, std::span<uint8_t> rx) noexcept {
        for (auto& b : rx) { b = next(); }
        co_return true;
    }

private:
    std::array<uint8_t, 64u> buf_{};
    uint8_t len_{0u}, pos_{0u};
    uint8_t next() noexcept { return (pos_ < len_) ? buf_[pos_++] : 0x00u; }
};
static_assert(IsSpiBus<FakeSpiBus>);

// ===========================================================================
// Pico2I2cBus — RP2350 hardware_i2c peripheral (I2C1, GP6/GP7)
// ===========================================================================
class Pico2I2cBus {
public:
    explicit constexpr Pico2I2cBus(const I2cPinConfig& cfg) noexcept : cfg_{cfg} {}

    [[nodiscard]] bool init() noexcept {
#if defined(PICO_BOARD)
        i2c_init(i2c1, cfg_.baud_hz);
        gpio_set_function(cfg_.sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(cfg_.scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(cfg_.sda_pin);
        gpio_pull_up(cfg_.scl_pin);
#endif
        return true;
    }

    [[nodiscard]] bool write_reg(uint8_t dev, uint8_t reg, uint8_t val) noexcept {
#if defined(PICO_BOARD)
        uint8_t buf[2] = { reg, val };
        return i2c_write_blocking(i2c1, dev, buf, 2u, false) == 2;
#else
        (void)dev; (void)reg; (void)val;
        return true;
#endif
    }

    [[nodiscard]] Task<bool> async_write_reg(uint8_t dev, uint8_t reg, uint8_t val) noexcept {
        co_return write_reg(dev, reg, val);
    }

    [[nodiscard]] uint8_t read_reg(uint8_t dev, uint8_t reg) noexcept {
        uint8_t val{0u};
        (void)read_regs(dev, reg, std::span<uint8_t>(&val, 1u));
        return val;
    }

    [[nodiscard]] Task<uint8_t> async_read_reg(uint8_t dev, uint8_t reg) noexcept {
        co_return read_reg(dev, reg);
    }

    [[nodiscard]] bool read_regs(uint8_t dev, uint8_t reg, std::span<uint8_t> rx) noexcept {
        if (rx.empty()) { return false; }
#if defined(PICO_BOARD)
        if (i2c_write_blocking(i2c1, dev, &reg, 1u, true) != 1) { return false; }
        return i2c_read_blocking(i2c1, dev, rx.data(), rx.size(), false)
               == static_cast<int>(rx.size());
#else
        (void)dev;
        (void)reg;
        std::fill(rx.begin(), rx.end(), fake_rx_);
        return true;
#endif
    }

    [[nodiscard]] Task<bool> async_read_regs(uint8_t dev, uint8_t reg, std::span<uint8_t> rx) noexcept {
        co_return read_regs(dev, reg, rx);
    }

    [[nodiscard]] bool write_bytes(uint8_t dev, std::span<const uint8_t> tx) noexcept {
        if (tx.empty()) { return false; }
#if defined(PICO_BOARD)
        return i2c_write_blocking(i2c1, dev, tx.data(), tx.size(), false)
               == static_cast<int>(tx.size());
#else
        (void)dev;
        return true;
#endif
    }

    [[nodiscard]] Task<bool> async_write_bytes(uint8_t dev, std::span<const uint8_t> tx) noexcept {
        co_return write_bytes(dev, tx);
    }

    void delay_ms(uint32_t ms) noexcept {
#if defined(PICO_BOARD)
        sleep_ms(ms);
#else
        (void)ms;
#endif
    }

    void set_fake_rx(uint8_t b) noexcept { fake_rx_ = b; }

private:
    I2cPinConfig cfg_{};
    uint8_t      fake_rx_{0x00u};
};
static_assert(IsI2cBus<Pico2I2cBus>);

// ===========================================================================
// FakeI2cBus — SITL / CppUTest injectable I2C bus
// ===========================================================================
class FakeI2cBus {
public:
    void inject(std::span<const uint8_t> data) noexcept {
        size_t n = std::min(data.size(), buf_.size());
        for (size_t i = 0u; i < n; ++i) { buf_[i] = data[i]; }
        len_ = static_cast<uint8_t>(n);
        pos_ = 0u;
    }
    void inject_byte(uint8_t b) noexcept { buf_[0]=b; len_=1u; pos_=0u; }

    [[nodiscard]] bool write_reg(uint8_t, uint8_t, uint8_t) noexcept { return true; }
    [[nodiscard]] uint8_t read_reg(uint8_t dev, uint8_t reg) noexcept {
        uint8_t val{0u};
        (void)read_regs(dev, reg, std::span<uint8_t>(&val, 1u));
        return val;
    }
    [[nodiscard]] bool read_regs(uint8_t, uint8_t, std::span<uint8_t> rx) noexcept {
        for (auto& b : rx) { b = next(); } return true;
    }
    [[nodiscard]] bool write_bytes(uint8_t, std::span<const uint8_t>) noexcept { return true; }
    void delay_ms(uint32_t) noexcept {}

    [[nodiscard]] Task<bool> async_write_reg(uint8_t, uint8_t, uint8_t) noexcept { co_return true; }
    [[nodiscard]] Task<uint8_t> async_read_reg(uint8_t dev, uint8_t reg) noexcept { co_return read_reg(dev, reg); }
    [[nodiscard]] Task<bool> async_read_regs(uint8_t, uint8_t, std::span<uint8_t> rx) noexcept {
        for (auto& b : rx) { b = next(); }
        co_return true;
    }
    [[nodiscard]] Task<bool> async_write_bytes(uint8_t, std::span<const uint8_t>) noexcept { co_return true; }

private:
    std::array<uint8_t, 64u> buf_{};
    uint8_t len_{0u}, pos_{0u};
    uint8_t next() noexcept { return (pos_ < len_) ? buf_[pos_++] : 0x00u; }
};
static_assert(IsI2cBus<FakeI2cBus>);

} // namespace abstractx::drivers::bus

#endif // BUS_CONCEPTS_HPP
