# Linux POSIX Dual Real-Time Thread Hardening Specification

> [!IMPORTANT]
> **POSIX DUAL REAL-TIME THREAD ARCHITECTURE**
> To achieve deterministic hard real-time execution on Linux targets, **`inav-abstractx`** partitions system responsibilities between **two dedicated POSIX real-time threads** ([`linux_rt_hardener.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_rt_hardener.hpp)).

---

## 1. Dual Real-Time Thread Configuration Matrix

| Thread Role | Scheduling Policy & Priority | Assigned CPU Core | Memory Locking | System Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| **Flight Control Thread** | `SCHED_FIFO` Priority 99 | Isolated CPU Core 3 (`isolcpus=3`) | `mlockall` Physical RAM | Executes 8 kHz C++20 coroutine loop, EKF3 fusion, PID dynamics, and 3D navigation math |
| **Hardware I/O Thread** | `SCHED_FIFO` Priority 98 | Isolated CPU Core 2 (`isolcpus=2`) | `mlockall` Physical RAM | Executes SPI/I2C/UART hardware transactions via `linux_async_io_dispatcher.hpp` |

---

## 2. Linux Dual-Core Affinity & Priority Architecture

```
                          Linux Dual Real-Time Threads
                                       │
        ┌──────────────────────────────┴──────────────────────────────┐
        ▼                                                             ▼
  Flight Control Thread                                      Hardware I/O Thread
  - Priority: SCHED_FIFO 99                                  - Priority: SCHED_FIFO 98
  - CPU Affinity: Core 3 (isolcpus=3)                        - CPU Affinity: Core 2 (isolcpus=2)
  - Memory: mlockall (Physical RAM)                          - Memory: mlockall (Physical RAM)
  - Tasks: EKF3, PID, Navigation                             - Tasks: spidev, i2c-dev, ttyS1
  - Execution Latency: < 10 ns coroutine swap                - Timestamping: DRDY Hardware Latched
```
