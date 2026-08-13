# RP2350 Pico 2 W Target Specification & Wi-Fi Configurator Setup

> [!IMPORTANT]
> **PICO 2 W WI-FI AP & USB CDC SERIAL CONFIGURATOR SUPPORT**
> On the **RP2350 Pico 2 W**, **iNav Configurator** connects wirelessly via the onboard **CYW43439 Wi-Fi chip** (Access Point `INAV_PICO2W` on IP `192.168.4.1:5760`) or via physical **USB CDC Serial** ([`pico2w_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pico2w_target.hpp)).

---

## 1. Pico 2 W Dual-Connection Modes

| Connection Method | Configurator Type | Connection Parameters | Core Allocation |
| :--- | :--- | :--- | :--- |
| **Wi-Fi Access Point** | TCP | IP: `192.168.4.1`, Port: `5760` (SSID: `INAV_PICO2W`) | Managed on **Core 0** (CYW43439 Driver) |
| **USB CDC Virtual COM** | Serial | Baud: `115200` / `921600` (`/dev/ttyACM0`) | Managed on **Core 0** (TinyUSB Driver) |

---

## 2. Pico 2 W Core Task Partitioning

```
  RP2350 Pico 2 W Dual ARM Cortex-M33 Cores
  ┌─────────────────────────────────────────────────────────────┐
  │                                                             │
  │  Core 0: Peripherals, Wi-Fi Telemetry & Configurator        │
  │  - CYW43439 Driver: Wi-Fi Access Point (`INAV_PICO2W`)      │
  │  - TCP 5760 Listener & USB CDC Virtual Serial COM           │
  │  - PIO0 Engine: DShot Motor Outputs (`GPIO 2..5`)           │
  │  - PIO1 Engine: CRSF Serial Receiver (`GPIO 6`)             │
  │  - PIO2 Engine: Auto-SPI IMU DMA (`GPIO 16..20`)            │
  │                                                             │
  ├─────────────────────────────────────────────────────────────┤
  │                                                             │
  │  Core 1: Hard Real-Time 8 kHz Flight Control Loop           │
  │  - Zero-Allocation C++20 Coroutine Loop (`run_flight_loop`) │
  │  - 15-State EKF3 Multi-Sensor Fusion & Betaflight PID       │
  │                                                             │
  └─────────────────────────────────────────────────────────────┘
```
