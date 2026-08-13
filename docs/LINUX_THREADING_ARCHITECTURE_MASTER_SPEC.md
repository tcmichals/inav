# Linux Multi-Threading Architecture Master Specification

> [!IMPORTANT]
> **COMPLETE LINUX MULTI-THREADING ARCHITECTURE**
> In **`inav-abstractx`**, Linux multi-threading is structured to guarantee that hard real-time flight control loop math ($125\ \mu\text{s}$ period at 8 kHz) is **100% isolated** from hardware I/O latencies, kernel preemption delays, disk flash page stalls ($500\text{ ms}$ wear-leveling), and TCP socket buffer retransmissions.

---

## 1. Complete Linux Thread Allocation & Priority Matrix

| Thread Name | POSIX Scheduling Policy | Priority | Assigned CPU Core | Core Isolation (`isolcpus`) | Memory Locking | System Responsibilities |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Flight Control Thread** | `SCHED_FIFO` | **99** (Highest) | **CPU Core 3** | `isolcpus=3` | `mlockall` Physical RAM | 8 kHz C++20 zero-alloc coroutine flight loop, 15-state EKF3, Betaflight 3-axis PID dynamics, 3D RTH navigation ([`src/main.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/main.cpp)) |
| **Hardware I/O Reactor** | `SCHED_FIFO` | **98** | **CPU Core 2** | `isolcpus=2` | `mlockall` Physical RAM | Linux native `epoll_wait()` event reactor for `/dev/spidev0.0`, `/dev/i2c-0`, and `/dev/ttyS1` ([`linux_epoll_reactor.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_epoll_reactor.hpp)) |
| **RPMsg / SPI Worker** | `SCHED_FIFO` | **97** | **CPU Core 1** | `isolcpus=1` | `mlockall` Physical RAM | VirtIO RPMsg `/dev/rpmsg0` shared memory transport to A5E RISC-V co-processor & FPGA ([`a5e_riscv_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/linux_sbc/a5e_riscv_target.hpp)) |
| **Background Disk Logger** | `SCHED_OTHER` | Default | **CPU Core 0** | Standard Linux OS | Standard Paged Memory | Writes binary CTF/BBL telemetry logs to SD card / eMMC flash ([`blackbox_logger.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/blackbox_logger.hpp)) |
| **Background TCP Server** | `SCHED_OTHER` | Default | **CPU Core 0** | Standard Linux OS | Standard Paged Memory | TCP socket server listening on port `5760` for iNav Configurator MSP tuning ([`tcp_configurator_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/tcp_configurator_server.hpp)) |

---

## 2. Lock-Free SPSC Ring Inter-Thread Communication

All inter-thread communication uses **Single-Producer Single-Consumer (SPSC) atomic ring buffers** ([`spsc_tlp_ring.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/spsc_tlp_ring.hpp)) with C++20 atomic thread-fence memory barriers (`std::atomic_thread_fence`).

- **Zero Mutex Locks**: No `std::mutex`, no condition variables, no POSIX semaphores.
- **Zero Real-Time Blocking**: Pushing or popping a 64-byte TLP takes **$< 10\text{ nanoseconds}$**.
