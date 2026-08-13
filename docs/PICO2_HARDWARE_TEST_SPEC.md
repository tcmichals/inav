# On-Device Hardware Validation Specification (`pico2_hw_test`)

> [!IMPORTANT]
> **HARDWARE VALIDATION HARNESS FOR PICO 2 W & LINUX**
> The `pico2_hw_test` target ([`pico2_hw_test.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pico2_hw_test.cpp)) runs on both real RP2350 Pico 2 W hardware and Linux SITL to validate physical sensor polling, I/O bus rates, and serial protocol timing.

---

## 1. Hardware Diagnostic Checks (`pico2_hw_test`)

| Diagnostic Test | Hardware Peripherals Verified | Pass Criteria / Expected Metric |
| :--- | :--- | :--- |
| **SPI IMU Sampling Rate** | ICM-42688-P over PIO2 Auto-SPI DMA | Verified 8 kHz ($125\ \mu\text{s}$) polling rate |
| **I2C Barometer Communication** | BMP280 / MS5611 over I2C0 Bus | Pressure reading $> 0.0\text{ Pa}$ |
| **I2C Magnetometer Communication**| QMC5883L over I2C0 Bus | Valid 3-axis Gauss magnetic field reading |
| **DShot Motor Waveforms** | PIO0 DShot outputs on GPIO 2..5 | Successful 16-bit DShot packet creation |
| **CRSF RC Serial Receiver** | PIO1 Serial RX Capture | Channel pulse validation ($800\text{--}2200\ \mu\text{s}$) |
| **MSP Serial Protocol Engine** | MSP v1/v2 over UART / TCP 5760 | Packet header `$M<` and `$M>` verification |

---

## 2. Execution Commands

```bash
cd /home/tcmichals/ssdData/projects/home/inav/build

# 1. Build and run desktop hardware test harness
cmake --build . --target pico2_hw_test
./pico2_hw_test

# 2. Flash to RP2350 Pico 2 W board via UF2 / SWD
# arm-none-eabi-gdb ./pico2_hw_test
```
