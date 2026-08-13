# Raspberry Pi Pico 2 W Pinout & Peripheral Assignment Specification

> [!IMPORTANT]
> **PARALLEL PWM RC INPUT & DSHOT MOTOR CONFIGURATION**
> Below is the complete pinout assignment for the **RP2350 Pico 2 W** supporting **Legacy Multi-Channel Parallel PWM RC Inputs** ([`pwm_rc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/pwm_rc.hpp)) and **Digital DShot Motor Outputs** ([`dshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/dshot.hpp)) across both **Linux/FPGA** and **Pico 2 W**!

---

## 1. GPIO Pin Assignment Table (Legacy Parallel PWM RC + DShot Motors)

| Subsystem | Peripheral Protocol | Assigned GPIO Pins | Function / Signals | Remaining Free Pins |
| :--- | :--- | :--- | :--- | :--- |
| **Auto-SPI IMU (ICM-42688-P)** | PIO 2 Auto-SPI Bus | `GPIO 16..20` | SCK, MOSI, MISO, CS, DRDY (5 pins) | 21 pins |
| **I2C Compass & Barometer** | I2C 0 Bus | `GPIO 0, 1` | SDA, SCL (2 pins) | 19 pins |
| **DShot Motor Outputs (M1..M4)**| PIO 0 DShot Waveforms | `GPIO 2, 3, 4, 5` | DShot150/300/600 Motor Outputs (4 pins) | 15 pins |
| **Parallel PWM RC Inputs (CH1..6)**| PIO 1 Parallel Pulse Captures | `GPIO 6, 7, 8, 9, 10, 11` | Roll, Pitch, Throttle, Yaw, AUX1, AUX2 (6 pins) | 9 pins |
| **WS2812 RGB LED Strip** | PIO Single-wire Output | `GPIO 12` | Status LED Strip (1 pin) | 8 pins |
| **Buzzer / Status Indicator** | Digital Output | `GPIO 15` | Audio Failsafe / Arming Buzzer (1 pin) | 7 pins |
| **Battery Voltage & Current ADC** | Analog ADC 0 / ADC 1 | `GPIO 26, 27` | ADC0 (Voltage), ADC1 (Current) (2 pins) | **5 Free Pins Remaining!** |

---

## 2. Unassigned Spare Pins Available (`GPIO 13, 14, 21, 22, 28`)

- **5 Unassigned Spare Pins**: `GPIO 13`, `GPIO 14`, `GPIO 21`, `GPIO 22`, `GPIO 28`.
- **Expansion**: Available for GPS Serial UART, camera shutter, or additional motor outputs.

---

## 3. Platform Parity Across Linux/FPGA & Pico 2 W

The exact same driver code ([`pwm_rc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/pwm_rc.hpp) and [`dshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/dshot.hpp)) parses parallel PWM RC inputs and generates DShot motor frames across **Linux SBC + FPGA** (Zynq-7020 / Cubie A5E), **Desktop Linux SITL**, and **RP2350 Pico 2 W**!
