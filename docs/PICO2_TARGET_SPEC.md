# Raspberry Pi RP2350 Pico 2 & Pico 2 W Target Platform Specification

> [!IMPORTANT]
> **FULL TRIPLE-PIO HARDWARE OFFLOADING ARCHITECTURE**
> In **`inav-abstractx`**, all physical hardware protocols (**DShot ESC outputs**, **CRSF/SBUS RC serial**, and **Auto-SPI IMU sensor reads**) are offloaded 100% onto the **RP2350's 3 PIO blocks (PIO0, PIO1, PIO2)**!

---

## 1. RP2350 Triple-PIO State Machine Allocation

```
+-----------------------------------------------------------------------------------+
|                           RP2350 PICO 2 / PICO 2 W PLATFORM                       |
+------------------------------------+----------------------------------------------+
| HARDWARE PIO OFFLOADERS (0.0% CPU) | CORE 1: Zero-Alloc Flight Loop               |
+------------------------------------+----------------------------------------------+
| - PIO 0 SM 0..3: DShot150..1200    | - EKF3 15-State Sensor Fusion                |
| - PIO 1 SM 0..1: CRSF/SBUS Serial  | - Betaflight 3-Axis PID Dynamics             |
| - PIO 2 SM 0..1: Auto-SPI IMU Burst| - iNav 3D Autonomous Navigation              |
| - Core 0: CYW43439 Wi-Fi/BT Router | - C++20 Template QuadX Mixer                 |
+------------------------------------+----------------------------------------------+
|                    Lock-Free SPSC 64-Byte TLP Ring Buffer                         |
+-----------------------------------------------------------------------------------+
```

| PIO Block | State Machine | Protocol / Device | Hardware Function | CPU Load |
| :--- | :--- | :--- | :--- | :--- |
| **PIO 0** | SM 0..3 | DShot150/300/600/1200 | 4..8 Motor PWM/DShot Waveform Generator | **0.0% CPU** |
| **PIO 1** | SM 0..1 | CRSF / SBUS Serial RX | High-speed 420kBaud serial receiver | **0.0% CPU** |
| **PIO 2** | SM 0..1 | Auto-SPI IMU Reader | Wait DRDY -> Auto-burst 14B IMU payload to DMA | **0.0% CPU** |

---

## 2. On-Chip SPI Flash Storage Layout (`PICO2_FLASH_CONFIG_OFFSET`)

- Sector Base: `0x1F0000` (Sector 500 in RP2350 SPI Flash).
- Memory-Mapped Direct Read (XIP Base: `0x101F0000`).
- Decoupled configuration persistence via `FlashStorageAdapter` API ([`flash_storage.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/flash_storage.hpp)).

---

## 3. Building Pico 2 W Firmware Binary (`inav_abstractx_pico2w.uf2`)

```bash
cd /home/tcmichals/ssdData/projects/home/inav

# 1. Configure CMake with Pico 2 W SDK toolchain
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DPICO_SDK_PATH=~/.pico-sdk

# 2. Build Pico 2 W UF2 binary
cmake --build build_pico2w -j$(nproc)
```

Connect wirelessly to the Pico 2 W Wi-Fi AP (`iNav-Pico2W`) and tune PID parameters live in **iNav Configurator** over TCP `192.168.4.1:5760`!
