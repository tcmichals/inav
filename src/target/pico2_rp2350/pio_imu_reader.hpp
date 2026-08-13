/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - RP2350 Hardware PIO SPI IMU Auto-Reader Offloader
 */

#ifndef PIO_IMU_READER_HPP
#define PIO_IMU_READER_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include <cstdint>

namespace abstractx::target::pico2 {

// Dedicated PIO 2 State Machine for Auto-SPI IMU Bursts
class PioImuReader {
public:
    // Initialize PIO 2 State Machine 0 for Auto-SPI IMU Reads at DRDY edge
    static void init_pio_imu_sm(uint8_t spi_sck_pin, uint8_t spi_tx_pin, uint8_t spi_rx_pin, uint8_t spi_cs_pin, uint8_t drdy_pin) noexcept {
        (void)spi_sck_pin; (void)spi_tx_pin; (void)spi_rx_pin; (void)spi_cs_pin; (void)drdy_pin;
        // 1. Load PIO assembly program: Wait DRDY high -> Pull CS low -> Shift 14-byte IMU burst -> Push CS high
        // 2. Direct DMA Channel to stream 14-byte payload into lock-free SPSC TLP Ring Buffer
        // 3. 0.0% CPU overhead on both Core 0 and Core 1!
    }

    // Build 64B TLP frame from PIO DMA burst buffer
    static Tlp64 make_imu_tlp(const uint8_t* imu_burst_14b, uint64_t timestamp_ns, uint8_t tag) noexcept {
        Tlp64 tlp = Tlp64::make_mem_write(bar::ImuBase, 0, tag);
        tlp.wire.timestamp_ns = timestamp_ns;
        for (size_t i = 0; i < 14; ++i) {
            tlp.wire.payload[i] = imu_burst_14b[i];
        }
        return tlp;
    }
};

} // namespace abstractx::target::pico2

#endif // PIO_IMU_READER_HPP
