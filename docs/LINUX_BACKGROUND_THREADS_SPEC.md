# Linux Disk I/O & TCP Telemetry Background Thread Specification

> [!IMPORTANT]
> **BACKGROUND THREAD ISOLATION FOR DISK & TCP NETWORKING**
> Storage devices (SD cards / eMMC flash block garbage collection) can stall for **$10\text{--}500\text{ ms}$**, and TCP sockets (`5760` iNav Configurator) can block during network retransmissions.
>
> To guarantee hard real-time execution, **`inav-abstractx`** pushes logging packets and telemetry frames into lock-free SPSC ring buffers. Dedicated background threads on non-isolated CPU cores (`SCHED_OTHER` on Core 0) drain these rings asynchronously ([`blackbox_logger.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/blackbox_logger.hpp)).

---

## 1. System Thread Architecture & Priority Matrix

| Thread Name | Scheduling Policy | Assigned CPU Core | Blocking Potential | Responsibilities |
| :--- | :--- | :--- | :--- | :--- |
| **Flight Control Thread** | `SCHED_FIFO` **Priority 99** | Isolated Core 3 (`isolcpus=3`) | **0.0 ms (Zero Blocking)** | 8 kHz C++20 coroutine loop, EKF3 fusion, Betaflight PID dynamics |
| **Hardware I/O Thread** | `SCHED_FIFO` **Priority 98** | Isolated Core 2 (`isolcpus=2`) | **$< 2.5\ \mu\text{s}$** | SPI/I2C/UART/RPMsg non-blocking hardware dispatching |
| **Background Disk Logger** | `SCHED_OTHER` (Standard) | Non-RT Core 0 | $10\text{--}500\text{ ms}$ (SD Wear-Leveling) | Drains `g_blackbox_ring` and writes binary CTF/BBL logs to disk |
| **Background TCP Server** | `SCHED_OTHER` (Standard) | Non-RT Core 0 | $10\text{--}100\text{ ms}$ (Wi-Fi/Ethernet) | Manages socket port `5760` for iNav Configurator MSP commands |

---

## 2. Complete Multi-Thread System Architecture Diagram

```
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 1. HARD REAL-TIME FLIGHT THREAD (Isolated CPU Core 3, SCHED_FIFO 99)    │
  │    - 8 kHz Zero-Alloc C++20 Coroutine Loop (< 5 us execution).          │
  │    - Emits Blackbox log TLPs into g_blackbox_ring (< 10 ns).           │
  │    - Emits MSP Telemetry TLPs into g_msp_tx_ring (< 10 ns).             │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │ Lock-Free SPSC Ring Buffers
                                       │ (Zero Real-Time Blocking)
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 2. NON-REAL-TIME BACKGROUND THREADS (Standard CPU Core 0, SCHED_OTHER) │
  │    - Background Disk Logger Thread: Writes CTF/BBL logs to SD/eMMC.     │
  │    - Background TCP Server Thread: Listens on TCP 5760 for Configurator.│
  └─────────────────────────────────────────────────────────────────────────┘
```
