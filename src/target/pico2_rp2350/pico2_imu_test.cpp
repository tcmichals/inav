/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated ICM-42688-P IMU Bench Diagnostic for Pico 2 W
 *
 * Hardware Wiring:
 *   - GPIO 10 (Pin 14) -> SPI1 SCK
 *   - GPIO 11 (Pin 15) -> SPI1 MOSI / SDI
 *   - GPIO 12 (Pin 16) -> SPI1 MISO / SDO
 *   - GPIO 13 (Pin 17) -> IMU CS
 *   - GPIO 14 (Pin 19) -> IMU INT1 / DRDY
 *   - 3V3     (Pin 36) -> VDD / VDDIO (3.3V)
 *   - GND     (Pin 38) -> GND
 *
 * Diagnostic Output:
 *   - USB Serial CDC (115200 baud, /dev/ttyACM0)
 */

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "bus_concepts.hpp"
#include "icm42688p.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdio>
#include <cmath>

using namespace abstractx;

static SpscTlpRing<64u> g_imu_test_ring;

int main() {
    // 1. Initialise USB Serial CDC
    stdio_init_all();
    
    // Wait for USB serial connection (or continue after 3s)
    for (int i = 0; i < 30; ++i) {
        ::sleep_ms(100);
    }

    printf("\n============================================================\n");
    printf(" INAV-ABSTRACTX: RP2350 PICO 2 W / ICM-42688-P BENCH TEST\n");
    printf("============================================================\n");

    // 2. Configure Hardware SPI1 Pins
    drivers::bus::SpiPinConfig spi_cfg{};
    spi_cfg.sck_pin  = 10u;
    spi_cfg.mosi_pin = 11u;
    spi_cfg.miso_pin = 12u;
    spi_cfg.cs_pin   = 13u;
    spi_cfg.drdy_pin = 14u;
    spi_cfg.baud_hz  = 24000000u; // 24 MHz SPI

    drivers::bus::Pico2SpiBus imu_bus{spi_cfg};
    if (!imu_bus.init()) {
        printf("[ERROR] Failed to initialise SPI1 hardware bus!\n");
        while (true) { ::sleep_ms(500); }
    }
    printf("[PASS] SPI1 Initialised at 24 MHz (SCK=GP10, MOSI=GP11, MISO=GP12, CS=GP13)\n");

    // 3. Construct ICM-42688-P Driver
    drivers::imu::ImuConfig imu_cfg{};
    imu_cfg.odr_hz          = 1000u; // 1 kHz ODR
    imu_cfg.accel_range_g   = 16.0f;
    imu_cfg.gyro_range_dps  = 2000.0f;
    imu_cfg.enable_drdy_int = true;

    drivers::imu::Icm42688PDriver<drivers::bus::Pico2SpiBus> imu_driver{imu_bus, imu_cfg};

    // 4. Run Asynchronous Initialisation (WHO_AM_I, Soft Reset, AAF Filters)
    printf("[INFO] Probing ICM-42688-P WHO_AM_I register (0x75)...\n");
    auto init_task = imu_driver.async_init();
    while (!init_task.done()) {
        init_task.resume();
        ::sleep_ms(1);
    }

    if (!imu_driver.is_initialized()) {
        printf("[FAIL] ICM-42688-P Initialisation failed! Check wiring & 3.3V power.\n");
        while (true) { ::sleep_ms(1000); }
    }
    printf("[PASS] ICM-42688-P Detected & Initialised (WHO_AM_I = 0x47, AAF Filters Active)\n");

    // 5. Launch Coroutine Sample Loop
    auto sample_loop_task = imu_driver.sample_loop(g_imu_test_ring);

    // 6. Zero-Motion Gyro Calibration (1000 samples)
    printf("[INFO] Quadcopter must remain stationary. Calibrating gyro bias (1000 samples)...\n");
    float bias_x = 0.0f, bias_y = 0.0f, bias_z = 0.0f;
    uint32_t calib_samples = 0u;

    while (calib_samples < 1000u) {
        sample_loop_task.resume();

        auto opt_tlp = g_imu_test_ring.pop();
        if (opt_tlp.has_value()) {
            auto sample = drivers::imu::Icm42688PDriver<drivers::bus::Pico2SpiBus>::parse_tlp(*opt_tlp);
            bias_x += sample.gyro_deg_s[0];
            bias_y += sample.gyro_deg_s[1];
            bias_z += sample.gyro_deg_s[2];
            calib_samples++;
        }
        ::sleep_us(500);
    }

    bias_x /= 1000.0f;
    bias_y /= 1000.0f;
    bias_z /= 1000.0f;

    printf("[PASS] Gyro Zero-Motion Bias: X=%+0.3f, Y=%+0.3f, Z=%+0.3f deg/s\n", bias_x, bias_y, bias_z);
    printf("------------------------------------------------------------\n");
    printf(" LIVE SENSOR STREAM (Press Ctrl+C in terminal to exit)\n");
    printf(" Timestamp (ms) | Temp (C) | Accel X, Y, Z (g)        | Gyro X, Y, Z (deg/s)\n");
    printf("------------------------------------------------------------\n");

    // 7. Continuous Streaming Loop
    uint32_t sample_count = 0u;
    uint64_t last_report_us = time_us_64();
    uint32_t rate_counter = 0u;

    while (true) {
        sample_loop_task.resume();

        auto opt_tlp = g_imu_test_ring.pop();
        if (opt_tlp.has_value()) {
            auto sample = drivers::imu::Icm42688PDriver<drivers::bus::Pico2SpiBus>::parse_tlp(*opt_tlp);
            rate_counter++;
            sample_count++;

            // Print at 20 Hz to avoid flooding USB CDC buffer
            if (sample_count % 50 == 0) {
                uint32_t ms = static_cast<uint32_t>(sample.timestamp_ns / 1000000ULL);
                printf("[%8lu ms] | %+5.1f C  | %+6.3f, %+6.3f, %+6.3f g | %+7.2f, %+7.2f, %+7.2f dps\n",
                       ms,
                       sample.temperature_c,
                       sample.accel_g[0], sample.accel_g[1], sample.accel_g[2],
                       sample.gyro_deg_s[0] - bias_x,
                       sample.gyro_deg_s[1] - bias_y,
                       sample.gyro_deg_s[2] - bias_z);
            }
        }

        uint64_t now_us = time_us_64();
        if (now_us - last_report_us >= 1000000u) {
            // Rate metrics can be printed here if needed
            rate_counter = 0u;
            last_report_us = now_us;
        }

        ::sleep_us(250);
    }

    return 0;
}
