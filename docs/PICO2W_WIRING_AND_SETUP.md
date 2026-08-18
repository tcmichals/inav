# Raspberry Pi Pico 2 W Flight Controller Wiring & Setup Guide

> [!IMPORTANT]
> **TARGET HARDWARE**: Raspberry Pi Pico 2 W (RP2350 microcontroller with CYW43439 Wi-Fi/Bluetooth).
> **FLIGHT CONTROL STACK**: `inav-abstractx` C++20 Zero-Allocation Engine.

---

## 1. Hardware Pinout & Wiring Diagram

The diagram below reflects the official 40-pin DIP package for the **Raspberry Pi Pico 2 W (RP2350)** with **6-Channel Parallel PWM RC Inputs**:

```text
                                  TOP VIEW (Pico 2 W)
                                +──────────────────────+
         (UART0 TX / GPS Config) | [01] GP0        VBUS [40] | (5V USB Input Power)
           (UART0 RX / GPS Data) | [02] GP1        VSYS [39] | (5V BEC Main Power Input)
                 (Digital Ground)| [03] GND         GND [38] | (Power Ground)
            (Motor 1 / DShot SM0)| [04] GP2      3V3_EN [37] | (3.3V Regulator Enable)
            (Motor 2 / DShot SM1)| [05] GP3     3V3_OUT [36] | (3.3V Sensor Power Rail)
            (Motor 3 / DShot SM2)| [06] GP4    ADC_VREF [35] | (ADC Reference Voltage)
            (Motor 4 / DShot SM3)| [07] GP5        GP28 [34] | (ADC2 / VBat Voltage Sense)
                 (Digital Ground)| [08] GND        AGND [33] | (Analog Ground)
      (I2C1 SDA / Baro & Compass)| [09] GP6        GP27 [32] | (ADC1 / Current Sense)
      (I2C1 SCL / Baro & Compass)| [10] GP7        GP26 [31] | (ADC0 / Analog RSSI)
             (RC PWM CH1 / Roll)| [11] GP8         RUN [30] | (Hardware Reset Button)
            (RC PWM CH2 / Pitch)| [12] GP9        GP22 [29] | (Buzzer / Beeper Output)
                 (Digital Ground)| [13] GND         GND [28] | (Digital Ground)
            (SPI1 SCK / IMU Clock)| [14] GP10       GP21 [27] | (Spare GPIO / I2C0 SCL)
             (SPI1 MOSI / IMU SDI)| [15] GP11       GP20 [26] | (Spare GPIO / I2C0 SDA)
             (SPI1 MISO / IMU SDO)| [16] GP12       GP19 [25] | (RC PWM CH6 / Aux 2 Flight Mode)
              (IMU Chip Select CS)| [17] GP13       GP18 [24] | (RC PWM CH5 / Aux 1 Arm Switch)
                 (Digital Ground)| [18] GND         GND [23] | (Digital Ground)
            (IMU DRDY / INT1 IRQ)| [19] GP14       GP17 [22] | (RC PWM CH4 / Yaw Rudder)
        (WS2812 RGB Status LED/SM2)| [20] GP15       GP16 [21] | (RC PWM CH3 / Throttle)
                                +──────────────────────+
```

---

## 2. Complete Component Wiring Table

### A. Primary 6-Axis IMU (SPI1 Bus / PIO 2)
*Supported Chips: InvenSense ICM-42688-P, Bosch BMI088, InvenSense MPU-6000*

| IMU Sensor Pin | Pico 2 W Pin | GPIO Pin | Signal Function | Wiring Notes |
| :--- | :--- | :--- | :--- | :--- |
| **VDD / VDDIO** | **Pin 36** | `3V3_OUT` | 3.3V Power | Clean 3.3V sensor rail |
| **GND** | **Pin 18** | `GND` | Power Ground | Star ground reference |
| **SCK / SCL** | **Pin 14** | `GPIO 10` | SPI1 Clock | 24 MHz SPI Clock |
| **SDI / MOSI** | **Pin 15** | `GPIO 11` | SPI1 Master-Out | Data into IMU |
| **SDO / MISO** | **Pin 16** | `GPIO 12` | SPI1 Master-In | Data from IMU |
| **CS / SS** | **Pin 17** | `GPIO 13` | Chip Select | Active Low |
| **INT1 / DRDY** | **Pin 19** | `GPIO 14` | Data Ready IRQ | High-resolution timestamp latch |

---

### B. Motors / ESCs (DShot300 / DShot600 on PIO 0)

| Motor / ESC | Pico 2 W Pin | GPIO Pin | Function | PIO State Machine |
| :--- | :--- | :--- | :--- | :--- |
| **Motor 1 (Rear Right)** | **Pin 4** | `GPIO 2` | DShot Signal | PIO 0 State Machine 0 |
| **Motor 2 (Front Right)**| **Pin 5** | `GPIO 3` | DShot Signal | PIO 0 State Machine 1 |
| **Motor 3 (Rear Left)**  | **Pin 6** | `GPIO 4` | DShot Signal | PIO 0 State Machine 2 |
| **Motor 4 (Front Left)** | **Pin 7** | `GPIO 5` | DShot Signal | PIO 0 State Machine 3 |
| **ESC Signal Ground**    | **Pin 3 / 8 / 13** | `GND` | Ground Reference | Common ground bus |

---

### C. Barometer & Magnetometer (I2C1 Bus)
*Supported Chips: DPS310, BMP280, MS5611, QMC5883L, IST8310*

| Sensor Pin | Pico 2 W Pin | GPIO Pin | Function | Wiring Notes |
| :--- | :--- | :--- | :--- | :--- |
| **VCC (3.3V)** | **Pin 36** | `3V3_OUT` | 3.3V Power | Shared 3.3V sensor rail |
| **GND** | **Pin 8** | `GND` | Ground | Digital ground |
| **SDA** | **Pin 9** | `GPIO 6` | I2C1 Data | 400 kHz Fast-Mode I2C |
| **SCL** | **Pin 10** | `GPIO 7` | I2C1 Clock | 400 kHz Fast-Mode I2C |

---

### D. 6-Channel Parallel PWM RC Receiver

Connect individual 3-pin servo leads (Signal, 5V, GND) from your classic 6-channel PWM receiver:

| PWM Channel | Receiver Pin | Pico 2 W Pin | GPIO Pin | Signal Function | Expected Range |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **CH1** | Channel 1 Signal | **Pin 11** | `GPIO 8` | Roll (Aileron) | 1000–2000 µs (Mid: 1500) |
| **CH2** | Channel 2 Signal | **Pin 12** | `GPIO 9` | Pitch (Elevator)| 1000–2000 µs (Mid: 1500) |
| **CH3** | Channel 3 Signal | **Pin 21** | `GPIO 16`| Throttle | 1000–2000 µs (Min: 1000) |
| **CH4** | Channel 4 Signal | **Pin 22** | `GPIO 17`| Yaw (Rudder) | 1000–2000 µs (Mid: 1500) |
| **CH5** | Channel 5 Signal | **Pin 24** | `GPIO 18`| Aux 1 (Arm Switch) | Low: Disarm / High: Arm |
| **CH6** | Channel 6 Signal | **Pin 25** | `GPIO 19`| Aux 2 (Flight Mode)| Angle / Horizon / Nav RTH |
| **5V Power**| Receiver VCC (+5V)| **Pin 39** | `VSYS` | Receiver 5V Rail | From 5V BEC |
| **Ground** | Receiver Ground (-)| **Pin 13 / 23**| `GND` | Power Ground | Common ground reference |

---

### E. GPS Module (Ublox UBX / NMEA on UART0)

| GPS Module Pin | Pico 2 W Pin | GPIO Pin | Function | Wiring Notes |
| :--- | :--- | :--- | :--- | :--- |
| **5V Power** | **Pin 39 / VSYS**| `5V / VSYS` | GPS Power | From 5V BEC |
| **GND** | **Pin 3** | `GND` | Ground | Common ground |
| **GPS TX** | **Pin 2** | `GPIO 1` | UART0 RX | 115200 Baud UBX/NMEA Data |
| **GPS RX** | **Pin 1** | `GPIO 0` | UART0 TX | 115200 Baud UBX Configuration |

---

### F. Power & Battery Voltage Monitoring

| Power Function | Pico 2 W Pin | GPIO Pin | Specifications |
| :--- | :--- | :--- | :--- |
| **5V System Power (BEC)** | **Pin 39** | `VSYS` | 5.0V Input from ESC / Power Distribution Board |
| **Battery Voltage Sense (VBat)**| **Pin 34** | `GPIO 28 (ADC2)`| Voltage divider (e.g. 10:1 or 11:1 for up to 6S LiPo) |
| **Current Sensor Sense** | **Pin 32** | `GPIO 27 (ADC1)`| Analog current shunt / Hall sensor input |

---

## 3. Step-by-Step Firmware Building & Flashing

### Step 1: Compile Firmware (Debug & Release)

Use the ARM GNU toolchain and Raspberry Pi Pico SDK located in `~/.tools`:

```bash
# Export toolchain paths
export PATH=/home/tcmichals/.tools/gcc-arm-none-eabi/bin:$PATH
export PICO_SDK_PATH=/home/tcmichals/.tools/pico-sdk

# Build Debug Firmware (for SWD Debugging with -g3 -O0)
cmake -B build_pico2w_debug -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Debug
cmake --build build_pico2w_debug -j$(nproc)

# Build Release Firmware (for production flight with -O3)
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release
cmake --build build_pico2w -j$(nproc)
```

The build produces two UF2 and ELF images in `build_pico2w/` and `build_pico2w_debug/`:
- **`inav_pico2_imu_test.elf` / `.uf2`**: Dedicated bench diagnostic firmware specifically for testing the ICM-42688-P sensor over USB serial and SWD.
- **`inav_abstractx_pico2w.elf` / `.uf2`**: Full dual-core autonomous flight controller firmware.

---

### Step 2: Flash onto the Pico 2 W

1. Disconnect the Pico 2 W from all power.
2. Press and **hold the white `BOOTSEL` button** on the board.
3. While holding `BOOTSEL`, plug the USB cable into your computer.
4. Release `BOOTSEL` once the **`RPI-RP2`** mass-storage drive appears.
5. Copy the UF2 binary to the drive:
   ```bash
   # For standalone ICM-42688-P IMU bench testing:
   cp build_pico2w_debug/inav_pico2_imu_test.uf2 /media/$USER/RPI-RP2/

   # Or for full flight controller operation:
   cp build_pico2w_debug/inav_abstractx_pico2w.uf2 /media/$USER/RPI-RP2/
   ```
6. The board will automatically reboot and start running.

---

## 4. Hardware SWD Debugging (VS Code + OpenOCD / Picoprobe)

### SWD Debug Header Pinout

On the Raspberry Pi Pico 2 W, connect your SWD debugger (Raspberry Pi Debug Probe, Picoprobe, or J-Link) to the 3-pin debug header:

```text
    +─────────────+
    |  [1] SWCLK  |  -> Connect to Debugger SWCLK
    |  [2] GND    |  -> Connect to Debugger Ground
    |  [3] SWDIO  |  -> Connect to Debugger SWDIO
    +─────────────+
```

### Launching VS Code Debugger

1. Open VS Code in the project root.
2. Switch to the **Run & Debug** panel (`Ctrl+Shift+D`).
3. Select one of the pre-configured debug launchers:
   - **`Debug Pico 2 W - ICM-42688-P Test (Cortex-Debug OpenOCD)`**: Automatically launches OpenOCD via `~/.tools/oss-cad-suite/bin/openocd`, attaches `arm-none-eabi-gdb`, flashes the ELF, breaks at `main()`, and supports full single-stepping and register inspection.
   - **`Debug Pico 2 W - Flight Engine (Cortex-Debug OpenOCD)`**: Debugs the full dual-core flight controller.
   - **`Debug Pico 2 W - GDB Remote Port 3333`**: For connecting to an external OpenOCD/pyocd server instance.
4. Press **F5** to start debugging.

---

## 5. Running the ICM-42688-P Bench Diagnostic Test

When running `inav_pico2_imu_test.uf2`:

1. Connect the Pico 2 W via USB.
2. Open a serial terminal at **115200 baud**:
   ```bash
   picocom -b 115200 /dev/ttyACM0
   ```
3. The board will output the diagnostic sequence:
   ```text
   ============================================================
    INAV-ABSTRACTX: RP2350 PICO 2 W / ICM-42688-P BENCH TEST
   ============================================================
   [PASS] SPI1 Initialised at 24 MHz (SCK=GP10, MOSI=GP11, MISO=GP12, CS=GP13)
   [INFO] Probing ICM-42688-P WHO_AM_I register (0x75)...
   [PASS] ICM-42688-P Detected & Initialised (WHO_AM_I = 0x47, AAF Filters Active)
   [INFO] Quadcopter must remain stationary. Calibrating gyro bias (1000 samples)...
   [PASS] Gyro Zero-Motion Bias: X=+0.124, Y=-0.082, Z=+0.015 deg/s
   ------------------------------------------------------------
    LIVE SENSOR STREAM
    Timestamp (ms) | Temp (C) | Accel X, Y, Z (g)        | Gyro X, Y, Z (deg/s)
   ------------------------------------------------------------
   [    1050 ms]   |  +31.4 C | +0.012, -0.008, +1.002 g |   +0.02,   -0.01,   +0.00 dps
   ```
