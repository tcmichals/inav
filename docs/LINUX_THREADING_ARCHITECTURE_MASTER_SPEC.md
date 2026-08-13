# Linux Single Real-Time Core Threading Architecture Specification

> [!IMPORTANT]
> **STRICT SINGLE REAL-TIME CORE ARCHITECTURE (`isolcpus=3`)**
> In **`inav-abstractx`**, Linux execution uses **EXACTLY ONE ISOLATED REAL-TIME CORE** (CPU Core 3, `SCHED_FIFO` Priority 99) ([`linux_rt_hardener.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_rt_hardener.hpp)).
>
> Single Real-Time CPU Core 3 executes the 8 kHz C++20 coroutine flight loop **AND** the non-blocking Linux native `epoll` I/O reactor concurrently ($< 4.3\ \mu\text{s}$ total frame execution time, leaving $> 96\%$ CPU idle headroom). All non-realtime background tasks (Disk Logging, TCP Configurator) run on non-RT Core 0 under standard Linux OS (`SCHED_OTHER`).

---

## 1. Single Real-Time Core System Architecture

```
  Linux ARM Host (Single Real-Time Core 3 + Standard OS Core 0)
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 1. SINGLE ISOLATED REAL-TIME CORE 3 (`isolcpus=3`, SCHED_FIFO Priority 99)│
  │    - 8 kHz Zero-Alloc C++20 Coroutine Loop (`run_flight_loop`)          │
  │    - 15-State EKF3 Multi-Sensor Fusion & Betaflight PID Dynamics        │
  │    - Linux Native `epoll` Asynchronous Hardware I/O Reactor           │
  │    - Total Worst-Case Frame Time: < 4.3 us (Leaving >96% CPU Headroom)  │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │ Lock-Free SPSC Ring Buffers
                                       │ (Zero Real-Time Blocking)
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 2. NON-REAL-TIME BACKGROUND CORE 0 (Standard Linux OS, SCHED_OTHER)     │
  │    - Background Disk Logger Thread (`BlackboxLogger` to SD/eMMC flash)  │
  │    - Background TCP Socket Server (`TcpConfiguratorServer` Port 5760)  │
  │    - RPMsg Kernel Driver (/dev/rpmsg0 to A5E RISC-V & FPGA)             │
  └─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. System Thread Allocation Matrix

| Thread Domain | POSIX Scheduling Policy | Priority | CPU Core Allocation | Core Isolation | System Responsibilities |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Unified Real-Time Thread** | `SCHED_FIFO` | **99** (Max) | **Core 3 (SINGLE RT CORE)** | `isolcpus=3` | 8 kHz flight loop coroutine, EKF3 fusion, Betaflight PID, and non-blocking `epoll` hardware I/O poll |
| **Background Disk Logger** | `SCHED_OTHER` | Default | **Core 0 (Non-RT OS Core)** | Standard OS | Writes binary CTF/BBL logs to SD card / eMMC flash ([`blackbox_logger.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/blackbox_logger.hpp)) |
| **Background TCP Server** | `SCHED_OTHER` | Default | **Core 0 (Non-RT OS Core)** | Standard OS | TCP socket server listening on port `5760` for iNav Configurator MSP tuning ([`tcp_configurator_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/tcp_configurator_server.hpp)) |
