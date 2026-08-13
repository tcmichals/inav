# iNav Configurator TCP Connector & Integration Guide

> [!IMPORTANT]
> **TCP PORT 5760 CONNECTOR FOR INAV CONFIGURATOR**
> In **`inav-abstractx`**, the non-blocking TCP socket server ([`tcp_configurator_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/tcp_configurator_server.hpp)) listens on **TCP Port 5760** to handle incoming MSP v1/v2 connections from the official **iNav Configurator GUI**.

---

## 1. Quick Connection Steps

1. Launch **iNav Configurator** (v6.0+ / v7.0+).
2. Set connection type to **TCP**.
3. Target IP Address: `127.0.0.1` (for SITL) or `192.168.4.1` (Pico 2 W Wi-Fi Access Point).
4. Port: **`5760`**.
5. Click **Connect**.

```
  ┌────────────────────────┐                   ┌────────────────────────────┐
  │  iNav Configurator GUI │ ─── TCP 5760 ───> │  TcpConfiguratorServer     │
  │  (3D Model, PID, CLI)  │ <── MSP v1/v2 ─── │  (tcp_configurator_server) │
  └────────────────────────┘                   └────────────────────────────┘
```

---

## 2. Server Architecture & Configurator Feature Support

| Configurator Tab | MSP Commands Handled | Server File |
| :--- | :--- | :--- |
| **Setup / 3D Model** | `Cmd::ApiVersion`, `Cmd::FcVariant`, `Cmd::Attitude` | [`tcp_configurator_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/tcp_configurator_server.hpp) |
| **Sensors** | `Cmd::RawImu`, `Cmd::Altitude`, `Cmd::CompassHeading` | [`msp_server.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/msp_server.cpp) |
| **PID Tuning** | `Cmd::Pid`, `Cmd::SetPid` | [`config_registry.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/config_registry.hpp) |
| **Motors** | `Cmd::Motor`, `Cmd::SetMotor` | [`dshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/dshot.hpp) |
| **CLI Command Line** | CLI Engine (`status`, `version`, `dump`, `set`, `save`) | [`cli_engine.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/cli_engine.hpp) |
