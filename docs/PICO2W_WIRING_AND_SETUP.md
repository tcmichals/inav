# Raspberry Pi Pico 2 W Flight Controller Wiring & Setup Guide

> [!IMPORTANT]
> **TARGET HARDWARE**: Raspberry Pi Pico 2 W (RP2350 microcontroller with CYW43439 Wi-Fi/Bluetooth).
> **FLIGHT CONTROL STACK**: `inav-abstractx` C++20 Zero-Allocation Engine.

---

## 1. Hardware Pinout & Wiring Diagram

```text
                                  TOP VIEW (Pico 2 W)
                               +──────────────────────+
            (UART0 TX / GPS RX) | [01] GP0      GP28 [40] | (ADC2 / Spare)
            (UART0 RX / GPS TX) | [02] GP1      ADC_VREF [39] | 
                                | [03] GND       GP29 [38] | (ADC3 / VSYS_ADC Battery Monitor)
             (ESC 1 / DShot SM0)| [04] GP2       3V3_EN [37] | 
             (ESC 2 / DShot SM1)| [05] GP3      3V3_OUT [36] | (3.3V Power to Sensors)
             (ESC 3 / DShot SM2)| [06] GP4       3V3_GND [35] | 
             (ESC 4 / DShot SM3)| [07] GP5          GP22 [34] | (Spare GPIO)
                                | [08] GND           GND [33] | 
     (I2C1 SDA / Baro & Compass)| [09] GP6          GP21 [32] | (Spare GPIO)
     (I2C1 SCL / Baro & Compass)| [10] GP7          GP20 [31] | (Spare GPIO)
           (CRSF Receiver RX/SM0)| [11] GP8          GP19 [30] | (Spare SPI0)
          (CRSF Telemetry TX/SM1)| [12] GP9          GP18 [29] | (Spare SPI0)
                                | [13] GND           GND [28] | 
            (SPI1 SCK / IMU Bus)| [14] GP10         GP17 [27] | (Spare SPI0)
           (SPI1 MOSI / IMU Bus)| [15] GP11         GP16 [26] | (Spare SPI0)
           (SPI1 MISO / IMU Bus)| [16] GP12          GND [25] | 
             (IMU Chip Select CS)| [17] GP13         GP15 [24] | (WS2812 RGB LED Strip / SM2)
                                | [18] GND          GP14 [23] | (IMU DRDY Interrupt)
                                | [19] RUN          GP14 [22] | 
                                | [20] BOOTSEL      VBUS [21] | (5V USB Input)
                                +──────────────────────+
```

---

## 2. Complete Component Wiring Table

### A. Motors / ESCs (DShot300 / DShot600 on PIO 0)
| Motor / ESC | Pico 2 W Pin | GPIO Pin | Function | PIO State Machine |
| :--- | :--- | :--- | :--- | :--- |
| **Motor 1 (Rear Right)** | **Pin 4** | `GPIO 2` | DShot Output | PIO 0 State Machine 0 |
| **Motor 2 (Front Right)**| **Pin 5** | `GPIO 3` | DShot Output | PIO 0 State Machine 1 |
| **Motor 3 (Rear Left)**  | **Pin 6** | `GPIO 4` | DShot Output | PIO 0 State Machine 2 |
| **Motor 4 (Front Left)** | **Pin 7** | `GPIO 5` | DShot Output | PIO 0 State Machine 3 |
| **ESC Signal Ground**    | **Pin 3 / 8 / 13** | `GND` | Ground Reference | Common Ground Bus |

### B. Primary 6-Axis IMU (SPI1 Bus / PIO 2)
*Supported Chips: InvenSense ICM-42688-P, Bosch BMI088, InvenSense MPU-6000*
| IMU Sensor Pin | Pico 2 W Pin | GPIO Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **VCC (3.3V)** | **Pin 36** | `3V3_OUT` | 3.3V Power | Clean, regulated 3.3V rail |
| **GND** | **Pin 18** | `GND` | Power Ground | Star ground connection |
| **SCK / SCL** | **Pin 14** | `GPIO 10` | SPI1 Clock | 24 MHz SPI Clock |
| **SDI / MOSI** | **Pin 15** | `GPIO 11` | SPI1 Master-Out | Data to IMU |
| **SDO / MISO** | **Pin 16** | `GPIO 12` | SPI1 Master-In | Data from IMU |
| **CS / SS** | **Pin 17** | `GPIO 13` | Chip Select | Active Low |
| **INT / DRDY** | **Pin 23** | `GPIO 14` | Data Ready IRQ | High-resolution timestamp latch |

### C. Barometer & Magnetometer (I2C1 Bus)
*Supported Chips: DPS310, BMP280, MS5611, QMC5883L, IST8310*
| Sensor Pin | Pico 2 W Pin | GPIO Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **VCC (3.3V)** | **Pin 36** | `3V3_OUT` | 3.3V Power | Shared 3.3V rail |
| **GND** | **Pin 8** | `GND` | Power Ground | Common ground |
| **SDA** | **Pin 9** | `GPIO 6` | I2C1 Data | 400 kHz Fast-Mode I2C |
| **SCL** | **Pin 10** | `GPIO 7` | I2C1 Clock | 400 kHz Fast-Mode I2C |

### D. RC Receiver (CRSF / ExpressLRS / SBUS on PIO 1)
| Receiver Pin | Pico 2 W Pin | GPIO Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **5V Power** | **Pin 39 / VBUS** | `5V / VBUS` | Receiver Power | From 5V BEC |
| **GND** | **Pin 13** | `GND` | Ground | Common ground |
| **CRSF TX (Ch 1)** | **Pin 11** | `GPIO 8` | RC RX (PIO 1 SM0) | 420 kBaud Serial Input |
| **CRSF RX (Ch 2)** | **Pin 12** | `GPIO 9` | Telemetry TX (PIO 1 SM1)| 420 kBaud Telemetry Output |

### E. GPS Module (Ublox UBX / NMEA on UART0)
| GPS Module Pin | Pico 2 W Pin | GPIO Pin | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **5V Power** | **Pin 39 / VBUS** | `5V / VBUS` | GPS Power | 5V BEC |
| **GND** | **Pin 3** | `GND` | Ground | Common ground |
| **GPS TX** | **Pin 2** | `GPIO 1` | UART0 RX | 115200 Baud UBX/NMEA |
| **GPS RX** | **Pin 1** | `GPIO 0` | UART0 TX | 115200 Baud UBX Config |

### F. Power & Battery Voltage Monitoring
| Power Function | Pico 2 W Pin | GPIO Pin | Specifications |
| :--- | :--- | :--- | :--- |
| **5V System Power (BEC)** | **Pin 38** | `VSYS` | 5.0V Input from ESC / Power Board |
| **Battery Voltage Sense (VBat)** | **Pin 39** | `GPIO 29 (ADC3)` | On-board 3:1 voltage divider monitoring VSYS/VBat |

---

## 3. Step-by-Step Firmware Building & Flashing

### Step 1: Compile the Bare-Metal UF2 Firmware
From your development workstation:
```bash
cd /home/tcmichals/ssdData/projects/home/inav

# Configure CMake for Pico 2 W target
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release

# Build firmware binary
cmake --build build_pico2w -j$(nproc)
```

### Step 2: Flash onto the Pico 2 W
1. Disconnect the Pico 2 W from battery power.
2. Press and **hold the white `BOOTSEL` button** on the Pico 2 W board.
3. While holding `BOOTSEL`, plug the USB cable into your computer.
4. Release the `BOOTSEL` button once the **`RPI-RP2`** drive appears on your desktop.
5. Drag and drop (or copy) the compiled binary:
   ```bash
   cp build_pico2w/inav_pico2w.uf2 /media/$USER/RPI-RP2/
   ```
6. The Pico 2 W will automatically reboot and start the flight control engine.

---

## 4. Connecting INAV Configurator

The Pico 2 W runs a dual-core stack where **Core 0 broadcasts a dedicated Wi-Fi Access Point**:

1. **Connect to Wi-Fi Network**:
   * **SSID**: `inav-pico2w`
   * **Password**: None (Open AP) / `inavflight`
2. **Open INAV Configurator**:
   * Connection Type: **TCP**
   * IP Address: **`192.168.4.1`**
   * Port: **`5760`**
   * Click **Connect**.
3. **Alternative USB Serial Connection**:
   * Select **COM / `/dev/ttyACM0`** at **115200 baud** and click **Connect**.

---

## 5. Pre-Flight Configuration & Calibration Checklist

Before installing propellers on the aircraft:

1. [ ] **Accelerometer Calibration**: Place drone on a flat level surface and click **Calibrate Accelerometer** in the Configurator.
2. [ ] **Receiver Check**: Power transmitter, move sticks, and verify Roll, Pitch, Yaw, and Throttle channels respond between **1000 µs and 2000 µs** (Mid: **1500 µs**).
3. [ ] **Failsafe Check**: Switch off transmitter; verify Configurator flags `FAILSAFE` active and switches mode to `RTH` / `Emergency Land`.
4. [ ] **Motor Direction & DShot Test**: In the Motors tab, enable motor test mode, slowly spin motors 1–4, and confirm proper rotation direction (Props-in or Props-out).
5. [ ] **GPS 3D Fix**: Verify GPS acquires 6+ satellites and displays a valid 3D Home fix before arming.
6. [ ] **Arming Switch**: Verify Arm switch transitions from `DISARMED` to `ARMED` with 0 blocking flags.
