# Offline Logic Analyzer Hardware Testing Guide

This guide details how to perform offline hardware signal verification using an **8-Pin Saleae** or **Kingst Logic Analyzer** connected to the RP2350 Pico 2 W, Linux SBC + FPGA, or custom flight hardware.

---

## 1. Logic Analyzer Pin Hookup Guide (8 Channels)

| Channel | RP2350 Pico 2 Pin | Physical Signal | Verification Target / Protocol Decoder |
| :--- | :--- | :--- | :--- |
| **CH 0** | `GPIO 2` | Motor 1 DShot Output | DShot150/300/600 pulse timing ($3.33\ \mu\text{s}$ period for DShot300) |
| **CH 1** | `GPIO 3` | Motor 2 DShot Output | DShot pulse width timing |
| **CH 2** | `GPIO 4` | Motor 3 DShot Output | DShot pulse width timing |
| **CH 3** | `GPIO 5` | Motor 4 DShot Output | DShot pulse width timing |
| **CH 4** | `GPIO 6` | RC Receiver Signal | CRSF (420K Baud Async) / Parallel PWM ($1000\text{--}2000\ \mu\text{s}$) |
| **CH 5** | `GPIO 16` | SPI IMU SCK Clock | SPI Bus Clock Frequency (10 MHz Mode 3) |
| **CH 6** | `GPIO 17` | SPI IMU MOSI Data | SPI Register Write Payloads |
| **CH 7** | `GPIO 20` | SPI IMU DRDY Interrupt | 8 kHz ($125\ \mu\text{s}$) DRDY pulse rate |
| **GND** | `GND` | Common Ground | Reference ground plane |

---

## 2. Recommended Software Tools

1. **Saleae Logic 2 Software**:
   - **Download**: [Saleae Logic 2 GUI](https://www.saleae.com/downloads/).
   - **Python Automation API**: `pip install saleae-logic2` for automated signal captures.
   - **Built-in Decoders**: DShot, SPI, Async Serial, I2C.

2. **Sigrok / PulseView (Open Source Alternative)**:
   - **Download**: `sudo apt install pulseview sigrok-cli`.
   - **Driver Support**: Direct support for Saleae 8-pin and Kingst logic analyzers (`kingst-la2016`).

3. **Automated Offline CSV Verification**:
   - Run `python3 tools/validate_logic_trace.py trace.csv` to automatically check timing bounds.

---

## 3. Offline Verification Workflow

```bash
# 1. Capture logic trace with sigrok-cli (Saleae / Kingst LA)
sigrok-cli --driver saleae-logic16 --channels D0,D1,D2,D3,D4,D5,D6,D7 \
           --config samplerate=24MHz --time 5s -o trace.sr

# 2. Export signal channels to CSV
sigrok-cli -i trace.sr -O csv > trace.csv

# 3. Execute automated python trace verification
python3 tools/validate_logic_trace.py trace.csv
```
