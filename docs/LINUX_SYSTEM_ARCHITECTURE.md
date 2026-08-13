# Linux Quad-Core Flight System Architecture & Threading Specification

> [!IMPORTANT]
> **MASTER LINUX SYSTEM ARCHITECTURE & 4-CORE ALLOCATION**
> In **`inav-abstractx`**, Linux quad-core execution is partitioned to isolate hard real-time flight control loop math ($125\ \mu\text{s}$ period at 8 kHz) from hardware I/O latencies, MIPI-CSI camera processing, kernel preemption delays, disk flash page stalls ($500\text{ ms}$ wear-leveling), and TCP socket buffer retransmissions.

---

## 1. 4-Core Hardware CPU Allocation Matrix

| CPU Core | Core Isolation & Policy | Priority | Assigned Thread Domain | Responsibilities & Peripherals |
| :--- | :--- | :--- | :--- | :--- |
| **CPU Core 3** | **Isolated Real-Time** (`isolcpus=3`, `SCHED_FIFO`) | **99** (Highest) | **Flight Control Loop** | 8 kHz C++20 zero-alloc coroutine flight loop, 15-state EKF3 sensor fusion, Betaflight 3-axis PID dynamics, 3D RTH navigation ([`src/main.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/main.cpp)). Zero blocking I/O calls. |
| **CPU Core 2** | **General Real-Time I/O** (`isolcpus=2`, `SCHED_FIFO`) | **98** | **Hardware I/O Reactor** | Linux native `epoll_wait()` event reactor for `/dev/spidev0.0` (SPI IMU), `/dev/i2c-0` (Baro/Mag), and `/dev/ttyS1` (Serial GPS/CRSF) ([`linux_epoll_reactor.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_epoll_reactor.hpp)). |
| **CPU Core 1** | **Coprocessor Transport** (`isolcpus=1`, `SCHED_FIFO`) | **97** | **RPMsg / FPGA Transport** | VirtIO RPMsg `/dev/rpmsg0` shared memory transport to A5E RISC-V co-processor & `rt_offloader` FPGA Verilog cores ([`a5e_riscv_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/linux_sbc/a5e_riscv_target.hpp)). |
| **CPU Core 0** | **Standard OS & Networking** (`SCHED_OTHER`) | Default | **OS, Camera & Services** | Shared by standard Linux OS kernel, MIPI-CSI Camera V4L2 Optical Flow pipeline (`/dev/video0`), TCP Port 5760 iNav Configurator server, UDP Port 19000 telemetry streamer, Disk Blackbox Logger, and Battery/Health monitors. |

---

## 2. Complete Linux Thread Functional Matrix

| Thread Name | POSIX Policy & Core | Priority | Hardware / File Descriptor | Function & Performance Target |
| :--- | :--- | :--- | :--- | :--- |
| **1. Flight Control Thread** | `SCHED_FIFO` (Core 3) | 99 | Internal SPSC Ring | 8 kHz C++20 coroutine loop. $< 5.0\ \mu\text{s}$ execution time ($4\%$ CPU load). |
| **2. Hardware I/O Reactor** | `SCHED_FIFO` (Core 2) | 98 | `/dev/spidev0.0`, `/dev/i2c-0`, `/dev/ttyS1` | Non-blocking `epoll` reactor for SPI/I2C/UART (< 1 us wakeup latency). |
| **3. RPMsg / FPGA Transport** | `SCHED_FIFO` (Core 1) | 97 | `/dev/rpmsg0` | VirtIO zero-copy shared memory bridge to RISC-V co-processor & FPGA. |
| **4. Camera / Optical Flow** | `SCHED_OTHER` (Core 0)| Default | `/dev/video0` (V4L2) | MIPI-CSI camera image capture, ground velocity optical flow calculation ($V_x, V_y$). |
| **5. TCP Configurator Server**| `SCHED_OTHER` (Core 0)| Default | TCP Port `5760` | Manages MSP v1/v2 connections from official iNav Configurator GUI. |
| **6. UDP Telemetry Streamer**| `SCHED_OTHER` (Core 0)| Default | UDP Port `19000` | Streams live high-speed CTF/BBL log datagrams without blocking flight loop. |
| **7. Disk Blackbox Logger** | `SCHED_OTHER` (Core 0)| Default | `/dev/mmcblk0p1` | Writes binary CTF/BBL telemetry logs to SD card / eMMC storage. |
| **8. System Health Monitor** | `SCHED_OTHER` (Core 0)| Default | `/dev/adc0` & Sysfs | 10 Hz monitor for battery voltage sag, current draw, and CPU temperature. |

---

## 3. End-to-End Quad-Core System Architecture Diagram

```
  Linux ARM Host Quad-Core System (Allwinner Cubie A5E / x86 / Raspberry Pi)
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ CPU CORE 3: ISOLATED REAL-TIME FLIGHT CONTROL (isolcpus=3, SCHED_FIFO 99)│
  │ - 8 kHz Zero-Alloc C++20 Coroutine Loop (< 5 us frame time).            │
  │ - 15-State EKF3 Multi-Sensor Fusion & Betaflight PID Dynamics.          │
  │ - 0.0 ms Blocking Time (Pure Memory-to-Memory Math).                    │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
           Lock-Free Atomic Single-Producer Single-Consumer (SPSC) Rings
                                       │
  ┌────────────────────────────────────┼────────────────────────────────────┐
  │                                    │                                    │
  ▼                                    ▼                                    ▼
  CPU CORE 2: REAL-TIME HARDWARE I/O   CPU CORE 1: RPMSG TRANSPORT          CPU CORE 0: OS, CAMERA & NETWORKING
  (isolcpus=2, SCHED_FIFO 98)          (isolcpus=1, SCHED_FIFO 97)          (Standard OS, SCHED_OTHER)
  - epoll_wait() Event Reactor         - VirtIO RPMsg /dev/rpmsg0           - MIPI-CSI Camera (/dev/video0)
  - /dev/spidev0.0 (SPI IMU)           - A5E RISC-V Co-Processor Bridge     - TCP 5760 Configurator
  - /dev/i2c-0 (Baro / Compass)        - rt_offloader FPGA Verilog Cores    - UDP 19000 Telemetry Streamer
  - /dev/ttyS1 (Serial GPS / CRSF)     - Dual-SPI / QSPI Hardware Bus       - Disk Blackbox Logger (SD/eMMC)
                                                                            - System Battery / Temp Monitor
```
