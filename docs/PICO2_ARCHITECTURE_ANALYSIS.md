# RP2350 Target Architecture & Design Analysis

This document provides a technical overview comparing traditional MCU flight controller target implementations with the RP2350 dual-core architecture used in `inav-abstractx`.

---

## 1. Technical Design Matrix

| Design Feature | Traditional Single-Loop C Implementation | RP2350 Dual-Core C++20 Implementation | Functional Difference |
| :--- | :--- | :--- | :--- |
| **I/O Processing** | Hardware Timers & CPU Interrupts | **PIO State Machines** (PIO0 Motors, PIO1 RC, PIO2 Auto-SPI IMU) | Offloads I/O bit-timing from CPU cores |
| **Core Allocation** | Core 0 Monolithic Loop | **Core 0 (Peripheral/Telemetry) + Core 1 (Flight Loop)** | Isolates PID/EKF3 math on Core 1 |
| **Pin Function Switching** | Static Hardware Pin Mappings | **Dynamic PIO Microcode Reloading** | Allows same pins (GPIO 2..5) to switch between DShot and 1-Wire UART |
| **Task Model** | Polling microsecond loop in C | **Zero-Allocation C++20 Coroutines (`Task<void>`)** | Eliminates dynamic heap allocation for coroutine frames |
| **Hardware Abstraction** | Target-specific `#ifdef` directives | **C++20 Concepts HAL (`concepts::IsPlatform`)** | Provides a single unified interface across targets |

---

## 2. Historical Context & Technical Evolution

1. **Evolution of Embedded Flight Control**: Legacy C codebases were designed for early single-core microcontrollers (e.g. STM32F1/F4) where hardware timers (`TIM1`, `SPI`) provided reliable performance. These architectures established the foundation for modern drone flight controllers.
2. **Modern MCU Capabilities**: Newer microcontrollers like the RP2040 and RP2350 introduce Programmable I/O (PIO) state machines and dual ARM Cortex-M33 cores. Utilizing C++20 features (coroutines, compile-time concepts, `std::span`) allows flight controllers to leverage these hardware offloaders while maintaining code portability.
