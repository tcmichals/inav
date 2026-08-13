# iNav Configurator Integration & Setup Guide

This document details how to connect the official **iNav Configurator GUI** to `inav-abstractx` over TCP port 5760 on Linux SITL, RP2350 Pico 2 W Wi-Fi, or Linux SBC targets.

---

## 1. Quick Connection Steps

1. Launch **iNav Configurator** (v6.0+ / v7.0+).
2. Set connection type to **TCP**.
3. Target IP Address: `127.0.0.1` (for SITL) or `192.168.4.1` (Pico 2 W Wi-Fi Access Point).
4. Port: **`5760`**.
5. Click **Connect**.

```
  ┌────────────────────────┐                   ┌────────────────────────┐
  │  iNav Configurator GUI │ ─── TCP 5760 ───> │  inav-abstractx SITL   │
  │  (3D Model, PID, CLI)  │ <── MSP v1/v2 ─── │  (msp_server.cpp)     │
  └────────────────────────┘                   └────────────────────────┘
```

---

## 2. Supported MSP Commands & Configurator Tabs

| Configurator Tab | MSP Command Handlers (`msp_server.cpp`) | Implementation Verification |
| :--- | :--- | :--- |
| **Setup / 3D Model** | `Cmd::ApiVersion`, `Cmd::FcVariant`, `Cmd::Attitude` | Real-time roll/pitch/yaw model rendering |
| **Sensors** | `Cmd::RawImu`, `Cmd::Altitude`, `Cmd::CompassHeading` | Real-time 8 kHz IMU, Baro, & Mag telemetry graphs |
| **PID Tuning** | `Cmd::Pid`, `Cmd::SetPid` | Live 3-axis PID loop tuning |
| **Motors** | `Cmd::Motor`, `Cmd::SetMotor` | Individual DShot motor channel output testing |
| **CLI Command Line** | CLI Engine ([`cli_engine.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/cli_engine.hpp)) | Commands: `version`, `status`, `dump`, `set`, `save` |
