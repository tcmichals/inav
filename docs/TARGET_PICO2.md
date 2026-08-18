# Raspberry Pi RP2350 (Pico 2 / Pico 2 W) Target Platform Specification

## 1. Dual-Core Architecture & Real-Time Isolation

The Raspberry Pi RP2350 incorporates dual ARM Cortex-M33 cores (or dual Hazard3 RISC-V cores) running at 150 MHz (overclockable to 300+ MHz), 520 KB on-chip SRAM, and 3 Programmable I/O (PIO) blocks with 12 state machines.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                  RP2350 TARGET ARCHITECTURE                               |
+──────────────────────────────────────────┬────────────────────────────────────────────────+
| CORE 0 (Background I/O & Networking)     | CORE 1 (Hard Real-Time 1kHz Flight Engine)     |
+──────────────────────────────────────────┼────────────────────────────────────────────────+
| - CYW43439 Wi-Fi AP & lwIP TCP Stack     | - 1kHz Strict Real-Time Coroutine Flight Loop  |
| - MSP Server (Port 5760 / UART)          | - Sensor Filtering (PT1/PT2, Gyro Kalman)      |
| - Flash Storage Persistence (0x1F0000)   | - Rate PID (Feedforward 2.0, Anti-Gravity)     |
| - BareCTF Telemetry Logging Stream       | - INAV Inertial-Complementary Pos Estimator    |
| - Hardware Status & Arming Logic         | - Multi-Airframe Motor/Servo Mixer             |
+──────────────────────────────────────────┴────────────────────────────────────────────────+
                                           │
                        Lock-Free SPSC Ring Buffers (spsc_tlp_ring.hpp)
                                           │
                                           ▼
+───────────────────────────────────────────────────────────────────────────────────────────+
|                             TRIPLE-PIO HARDWARE OFFLOADERS                                |
|                                (0.0% Flight Loop CPU Load)                                |
+──────────────────────────────────────────┬────────────────────────────────────────────────+
| PIO 0 (SM 0..3): DShot150/300/600/1200   | 4x Dedicated Motor PWM / DShot Waveform Gen    |
| PIO 1 (SM 0..1): CRSF / SBUS Serial RX   | High-Speed 420 kBaud Serial Telemetry Receiver |
| PIO 2 (SM 0..1): Auto-SPI IMU Reader     | Hardware DRDY Sync -> 14B DMA IMU Burst Reader |
+──────────────────────────────────────────┴────────────────────────────────────────────────+
```

---

## 2. Pinout & GPIO Allocation Table (Pico 2 & Pico 2 W)

| Pin # | GPIO Pin | Function | Peripheral / PIO Allocation | Connected Hardware |
| :--- | :--- | :--- | :--- | :--- |
| 1 | **GPIO 0** | UART0 TX | UART0 / PIO1 | Serial GPS TX / MSP Telemetry |
| 2 | **GPIO 1** | UART0 RX | UART0 / PIO1 | Serial GPS RX / MSP Telemetry |
| 4 | **GPIO 2** | Motor 1 Output | **PIO 0 State Machine 0** | ESC 1 (DShot300/600) |
| 5 | **GPIO 3** | Motor 2 Output | **PIO 0 State Machine 1** | ESC 2 (DShot300/600) |
| 6 | **GPIO 4** | Motor 3 Output | **PIO 0 State Machine 2** | ESC 3 (DShot300/600) |
| 7 | **GPIO 5** | Motor 4 Output | **PIO 0 State Machine 3** | ESC 4 (DShot300/600) |
| 9 | **GPIO 6** | I2C1 SDA | I2C1 Hardware Controller | DPS310 / BMP280 Baro & QMC5883L Mag |
| 10 | **GPIO 7** | I2C1 SCL | I2C1 Hardware Controller | DPS310 / BMP280 Baro & QMC5883L Mag |
| 11 | **GPIO 8** | RC Receiver RX | **PIO 1 State Machine 0** | ExpressLRS / CRSF / SBUS Receiver |
| 12 | **GPIO 9** | RC Telemetry TX| **PIO 1 State Machine 1** | CRSF Telemetry Back-channel |
| 14 | **GPIO 10**| SPI1 SCK | SPI1 / **PIO 2 State Machine 0** | ICM-42688-P / BMI088 SPI Bus |
| 15 | **GPIO 11**| SPI1 MOSI | SPI1 / **PIO 2 State Machine 0** | ICM-42688-P / BMI088 SPI Bus |
| 16 | **GPIO 12**| SPI1 MISO | SPI1 / **PIO 2 State Machine 0** | ICM-42688-P / BMI088 SPI Bus |
| 17 | **GPIO 13**| IMU CS | GPIO Output | Primary IMU Chip Select |
| 19 | **GPIO 14**| IMU DRDY | GPIO IRQ / PIO Trigger | High-Resolution Nanosecond Timestamp Latch |
| 20 | **GPIO 15**| WS2812 LED Strip| PIO 1 State Machine 2 | Status RGB LED Strip |
| 36 | **3V3_OUT** | 3.3V Power Out  | Linear Regulator | Clean 3.3V Sensor Power Rail |
| 38 | **GND**     | Power Ground    | Ground Bus       | Star Ground Reference |
| 39 | **VSYS**    | 5V Power Input  | System Power Bus | 5V BEC from ESC / PDB |
| 34 | **GPIO 28** | ADC2 / VBat Sense| ADC Voltage Divider| External LiPo Battery Monitor |

---

## 3. Flash Storage Configuration Offset

* **Sector Base**: `0x1F0000` (Sector 496 in 4MB SPI Flash; outside code execution XIP space).
* **Memory-Mapped Direct Read**: Direct memory pointer read via XIP base `0x101F0000`.
* **Flash Writes**: Managed safely on Core 0 via `flash_range_program()` and `flash_range_erase()`.

---

## 4. CMake Compilation & UF2 Generation

The build system manages the **Raspberry Pi Pico SDK v2.0+** automatically via `FetchContent`:

```bash
cd /home/tcmichals/ssdData/projects/home/inav

# 1. Configure CMake with Pico 2 W board definition
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release

# 2. Build Pico 2 W UF2 binary
cmake --build build_pico2w -j$(nproc)
```

The resulting `inav_abstractx_pico2w.uf2` binary can be copied directly to the RP2350 USB mass-storage bootloader.

---

## 5. High-Frequency 8 kHz & 16 kHz Real-Time Timing Budgets

Core 1 can execute rate control at up to **16 kHz (62.5 µs period)** when paired with high-ODR IMUs (ICM-42688-P) and DShot600/1200:

| Loop Frequency | Window (µs) | CPU Cycles @ 150 MHz | CPU Cycles @ 300 MHz | Core 1 Math Time (@ 300 MHz) | Core 1 CPU Load (@ 300 MHz) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1 kHz** | 1,000.0 µs | 150,000 cycles | 300,000 cycles | ~12.1 µs | **~4.0%** |
| **8 kHz** | 125.0 µs | 18,750 cycles | 37,500 cycles | ~12.1 µs | **~9.7%** |
| **16 kHz** | 62.5 µs | 9,375 cycles | 18,750 cycles | ~12.1 µs | **~19.4%** |

* **Sensor Mode**: ICM-42688-P native 16 kHz / 32 kHz gyro mode streaming over 24 MHz SPI.
* **Actuator Protocol**: DShot600 (26.7 µs frame) or DShot1200 (13.3 µs frame) on PIO 0.
* **Multi-Rate Decimation**: 16 kHz Rate PID $\to$ 1 kHz Attitude/Angle mode $\to$ 100 Hz Baro $\to$ 10 Hz GPS Nav.
