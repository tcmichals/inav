# RP2350 Pico 2 W Master Target Specification & Peripheral Table

> [!IMPORTANT]
> **PICO 2 W BARE-METAL DUAL-CORE HARDWARE MAP (ONBOARD QSPI FLASH ONLY)**
> On the **RP2350 Pico 2 W**, execution is partitioned between **Core 0** (CYW43439 Wi-Fi AP, TCP 5760 Configurator, USB CDC, raw onboard QSPI Flash logging, I2C, and PIO state machines) and **Core 1** (100% dedicated 8 kHz C++20 flight loop) ([`pico2w_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pico2w_target.hpp)). Storage is **Onboard QSPI NOR Flash Only** (no SD card required).

---

## 1. RP2350 Pico 2 W Master Peripheral & Core Allocation Table

| ARM Core | Functional Domain | Hardware Peripherals & Modules | Hardware Pins / Interfaces | Performance Metric |
| :--- | :--- | :--- | :--- | :--- |
| **Core 0** | **PIO State Machines** | **PIO0 Engine**: DShot Motor Output Waves | `GPIO 2..5` (Motors 1--4) | Sub-nanosecond pulse generation |
| **Core 0** | **PIO State Machines** | **PIO1 Engine**: CRSF RC Serial RX / Parallel PWM | `GPIO 6..11` (RC RX Inputs) | 420K baud serial capture / PWM width |
| **Core 0** | **PIO State Machines** | **PIO2 Engine**: Auto-SPI IMU DMA Reader | `GPIO 16..20` (ICM-42688-P SPI) | 8 kHz ($125\ \mu\text{s}$) burst DMA polling |
| **Core 0** | **I2C Peripherals** | QMC5883L Magnetometer & BMP280 Barometer | `GPIO 0/1` (I2C0 SDA/SCL) | 100 Hz compass & 50 Hz pressure |
| **Core 0** | **Wi-Fi Configurator** | CYW43439 Wi-Fi AP (`INAV_PICO2W`) | TCP Port `5760` (`192.168.4.1`) | iNav Configurator MSP tuning |
| **Core 0** | **USB Virtual Serial**| USB CDC Virtual COM Port (`/dev/ttyACM0`) | USB Micro-B / Type-C | Full CLI & Configurator MSP access |
| **Core 0** | **Flash Logging** | **Onboard QSPI NOR Flash Only** | Direct QSPI Bus (`FLASH_CS`) | Raw 4MB / 16MB sector page writes |
| **Core 1** | **Dedicated Flight Loop**| 15-State EKF3, Betaflight 3-Axis PID, 3D RTH Nav | Lock-Free SPSC Ring Buffer (`g_telemetry_ring`) | **100% CPU reserved** for 8 kHz flight math ($< 15\ \mu\text{s}$ loop execution time) |

---

## 2. Onboard QSPI Flash Memory Map

```
  0x10000000 ┌──────────────────────────────────────────────┐
             │ Firmware Binary Image (inav_abstractx.uf2)   │ ~180 KB
  0x1003F000 ├──────────────────────────────────────────────┤
             │ ConfigRegistry Persistent Settings           │ 4 KB Sector
  0x10040000 ├──────────────────────────────────────────────┤
             │ Blackbox Flight Log Space (Raw TLP Packets)  │ ~3.7 MB
  0x10400000 └──────────────────────────────────────────────┘
```
