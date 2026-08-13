# The Ultimate Fusion: iNav Autonomous Navigation + Betaflight Flight Dynamics

> [!IMPORTANT]
> **THE VISION BEHIND `inav-abstractx`**
> - **Why iNav?** iNav is the gold standard for autonomous GPS navigation, Return-To-Home (RTH), 3D Waypoints, Safehomes, and Mission Planning, supported by the feature-rich **iNav Configurator** GUI.
> - **Why Betaflight?** Betaflight leads in modern flight dynamics, acro response, bidirectional DShot RPM filtering, PT1/PT2/PT3 dynamic D-term filtering, and feedforward tuning.
> - **The `inav-abstractx` Fusion**: Merges **iNav autonomous navigation** with **Betaflight-grade modern C++20 PID dynamics**, driven by **AbstractX 64-byte PCIe TLPs** and compatible with **iNav Configurator**.

---

## 1. Architectural Highlights

```
+-----------------------------------------------------------------------------------+
|                            iNAV CONFIGURATOR GUI                                  |
|         (USB / UART / PTY MSP v1/v2 Protocol - Waypoints, PIDs, OSD, Setup)      |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|               iNAV-ABSTRACTX CORE FLIGHT ENGINE (C++20 ZERO-HEAP)                 |
|                                                                                   |
|  [iNav Autonomous Navigation Engine]      [Betaflight-Grade PID Flight Dynamics]  |
|  - GPS Position Hold & Return-To-Home     - Dynamic PT1/PT2 D-Term Filtering     |
|  - 3D Waypoint Mission Controller          - Bidirectional DShot RPM Filtering    |
|  - Wind & Altitude Sensor Fusion          - Feedforward & TPA Scaling            |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                  C++20 ABSTRACTX PARALLEL I/O COROUTINE ENGINE                    |
|       - `co_await pcie_reg_read_async(addr)`                                      |
|       - `co_await tlp_stream_wait(channel)`                                       |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                  CANONICAL 64-BYTE PCIe TLP BUS (`asp-tlp-64b`)                   |
|       - Sub-microsecond Hardware Timestamps latched at exact DRDY edge            |
|       - Lock-free Single-Producer Single-Consumer (SPSC) shared memory rings      |
+-----------------------------------------------------------------------------------+
```

---

## 2. Feature Fusion Breakdown

| Feature | Legacy iNav | Legacy Betaflight | `inav-abstractx` Architecture |
| :--- | :--- | :--- | :--- |
| **Configurator GUI** | iNav Configurator (Excellent) | Betaflight Configurator | **Full iNav Configurator Compatibility** over MSP v1/v2 |
| **GPS Navigation** | Full RTH, Waypoints, Safehomes | Basic GPS Rescue | **Full iNav Autonomous Navigation & Waypoint Engine** |
| **PID Filtering** | Standard LPF filters | Dynamic PT1/PT2/PT3 & RPM Filter | **Betaflight-grade C++20 Dynamic Filtering & Feedforward** |
| **Hardware Abstraction** | Target-specific C HALs | MCU-specific C HALs | **AbstractX Virtual 32-Bit BAR Space & 64B TLPs** |
| **Linker Scripts** | Custom `.pg_registry` `.ld` files | Custom `.pg_registry` `.ld` files | **C++20 Zero-Linker Configuration Registry** (`MasterConfig`) |
| **Memory Allocation** | Dynamic heap usage (`malloc`) | Dynamic heap usage (`malloc`) | **Strict Zero-Heap Allocation** (`0 bytes` during flight) |
| **Hardware Timestamping** | Software IRQ latency ($20\text{--}50\ \mu\text{s}$) | Software IRQ latency ($20\text{--}50\ \mu\text{s}$) | **Sub-microsecond 64-bit Hardware Timestamps** ($<20\text{ ns}$) |
| **Simulation** | Complex SITL builds | SITL forks | **Built-in Linux Hardware Simulator** (IMU, Baro, GPS, ESCs) |
