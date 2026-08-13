# Physical Hardware Validation & Testing Guidelines

This document outlines the step-by-step procedures for testing and validating physical hardware components (RC receivers, motor mixing, ESC outputs, GPS navigation, and sensor buses) on the RP2350 Pico 2 W, Linux SBC + FPGA (`rt_offloader`), and SITL target platforms.

---

## 1. Physical Component Testing Matrix

| Hardware Component | Protocol / Interface | Validation Target | Diagnostic Check (`pico2_hw_test`) |
| :--- | :--- | :--- | :--- |
| **Parallel PWM RC Receiver** | Multi-channel PWM (`pwm_rc.hpp`) | Pulse widths $800\text{--}2200\ \mu\text{s}$, failsafe trigger when Throttle $< 900\ \mu\text{s}$ | `results.pwm_rc_decoding_ok` |
| **CRSF / SBUS Receiver** | Serial RX (`crsf.hpp` / `sbus.hpp`) | 420K baud frame decoding, channel mapping | `results.crsf_rx_ok` |
| **Motor Output & Mixing** | DShot150/300/600 (`dshot.hpp`) & QuadX Mixer | Output range $1000\text{--}2000\ \mu\text{s}$, zero saturation under maximum roll/pitch PID demand | `results.motor_mixing_ok` & `dshot_tx_ok` |
| **GPS Module** | NMEA / UBX Serial UART (`0x3000`) | 3D fix lock, latitude/longitude parsing, RTH steering vector generation | `results.gps_ubx_ok` |
| **SPI IMU (ICM-42688-P)** | PIO2 Auto-SPI DMA (`pio_imu_reader.hpp`) | 8 kHz ($125\ \mu\text{s}$) polling rate, 64-bit nanosecond timestamp delta ($\Delta t = 125,000\text{ ns}$) | `results.imu_spi_ok` |
| **I2C Barometer & Compass** | I2C0 Bus (`bmp280.hpp` / `qmc5883l.hpp`) | Pressure $> 0.0\text{ Pa}$, 3-axis Gauss magnetic vector | `results.baro_i2c_ok` & `mag_i2c_ok` |

---

## 2. Step-by-Step Hardware Test Procedures

### A. Parallel PWM RC Input Decoding Check
1. Connect multi-wire parallel PWM receiver signals to `GPIO 6..11`.
2. Power on RC transmitter and move Roll/Pitch/Throttle/Yaw sticks.
3. Run `pico2_hw_test`. Verify pulse widths align with stick inputs ($1000\ \mu\text{s}$ low, $1500\ \mu\text{s}$ center, $2000\ \mu\text{s}$ high).
4. Power off RC transmitter. Verify `rc.failsafe == true` when Throttle drops below $900\ \mu\text{s}$.

### B. Motor Mixing & ESC Signal Check
1. Connect ESC control wires to motor output pins (`GPIO 2..5`).
2. Armed state: Verify motors output idle command ($1150\ \mu\text{s}$ or DShot `48`).
3. Apply full Roll/Pitch/Yaw stick inputs. Verify QuadX mixer distributes power across all 4 motors without saturating or wrapping around 16-bit values.

### C. GPS Navigation & RTH Target Check
1. Connect GPS module UART TX/RX to `GPIO 8/9` (UART1).
2. Place drone outdoors with clear view of the sky until 3D fix lock is established.
3. Trigger Return-To-Home (RTH) mode. Verify `NavigationEngine` computes pitch/roll tilt vectors directing the craft toward home coordinates (`home_lat`, `home_lon`).

---

## 3. Hardware Test Execution Command

```bash
cd /home/tcmichals/ssdData/projects/home/inav/build
cmake --build . --target pico2_hw_test
./pico2_hw_test
```
