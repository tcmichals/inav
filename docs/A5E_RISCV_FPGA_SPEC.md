# Allwinner Cubie A5E RISC-V Co-Processor & FPGA Offloader Specification

> [!IMPORTANT]
> **HETEROGENEOUS A5E RISC-V + FPGA HARDWARE ARCHITECTURE**
> On the **Allwinner Cubie A5E Linux SBC**, hardware offloading is partitioned between the Linux ARM host cores, the embedded **A5E RISC-V co-processor core**, and the **`rt_offloader` FPGA** ([`a5e_riscv_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/linux_sbc/a5e_riscv_target.hpp)).

---

## 1. Heterogeneous Tri-Core Task Allocation

```
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 1. Linux ARM Cortex-A53 Host Cores (Allwinner Cubie A5E)                │
  │    - Runs Linux Real-Time thread hardener (mlockall, SCHED_FIFO 99/98).  │
  │    - Runs 8 kHz EKF3 15-state estimator & Betaflight 3-axis PID dynamics.│
  │    - Runs Wi-Fi / Ethernet MSP Configurator server on TCP 5760.          │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
                Shared Memory Lock-Free SPSC TLP Ring Buffer
                                       │
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 2. Embedded A5E RISC-V Co-Processor Core (Bare-Metal C++20 HAL)         │
  │    - Dedicated hard real-time I/O coprocessor.                          │
  │    - Latches 64-bit nanosecond hardware timestamps at DRDY clock edge.   │
  │    - Manages high-speed Dual-SPI / PCIe TLP bus to FPGA.                │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
                      Dual-SPI / PCIe TLP Hardware Bus
                                       │
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 3. FPGA Target (`rt_offloader` Verilog Cores)                           │
  │    - Verilog DShot Motor Core (DShot150/300/600 pulse generation).       │
  │    - Verilog Parallel PWM / PPM RC Capture Core.                        │
  │    - Verilog Auto-SPI IMU DMA Core (8 kHz ICM-42688-P burst reader).    │
  └─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Technical Characteristics

1. **Deterministic Latency**: The A5E RISC-V core runs bare-metal without OS context switching, providing deterministic sub-microsecond communication with the `rt_offloader` FPGA.
2. **Nanosecond Timestamping**: The A5E RISC-V core latches 64-bit nanosecond hardware timestamps (`timestamp_ns`) when the FPGA IMU `DRDY` interrupt fires.
3. **Transport Parity**: Uses the exact same 64-byte TLP packet structure (`asp_tlp64.hpp`) used on RP2350 Pico 2 W and Linux SITL targets.
