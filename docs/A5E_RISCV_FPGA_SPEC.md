# Allwinner Cubie A5E RISC-V RPMsg & SPI Dual-Transport Specification

> [!IMPORTANT]
> **HETEROGENEOUS A5E RISC-V RPMsg / SPI TRANSPORT ARCHITECTURE**
> On the **Allwinner Cubie A5E Linux SBC**, communication with the embedded **A5E RISC-V co-processor** and **`rt_offloader` FPGA** uses a dedicated **POSIX Real-Time Worker Thread 3** (`SCHED_FIFO` Priority 97, CPU Core 1) ([`a5e_riscv_target.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/linux_sbc/a5e_riscv_target.hpp)).
>
> Data transport supports two dynamic modes:
> 1. **RPMsg over virtio (`/dev/rpmsg0`)**: Linux ARM Host $\leftrightarrow$ A5E RISC-V Co-Processor $\leftrightarrow$ FPGA.
> 2. **Direct SPI (`/dev/spidev0.0`)**: Linux ARM Host $\leftrightarrow$ FPGA.

---

## 1. Multi-Thread POSIX & Hardware Topology

```
                                  Linux ARM Host (Cubie A5E)
                                              │
        ┌─────────────────────────────────────┼─────────────────────────────────────┐
        ▼                                     ▼                                     ▼
  Thread 1: Flight Loop              Thread 2: General I/O              Thread 3: RPMsg Worker
  - SCHED_FIFO Priority 99           - SCHED_FIFO Priority 98           - SCHED_FIFO Priority 97
  - CPU Core 3 (isolcpus=3)          - CPU Core 2 (isolcpus=2)          - CPU Core 1 (isolcpus=1)
  - 8 kHz EKF3 & PID Math            - I2C Baro / Compass               - /dev/rpmsg0 or /dev/spidev0.0
        │                                                                           │
        └───────────────────────────────┬───────────────────────────────────────────┘
                                        │ Lock-Free SPSC TLP Ring Buffer
                                        ▼
                          Embedded A5E RISC-V Co-Processor
                          - Bare-metal zero-copy virtio ring
                          - Latches DRDY 64-bit nanosecond timestamps
                                        │
                                        ▼ Dual-SPI / PCIe TLP Bus
                          `rt_offloader` FPGA Target
                          - Verilog DShot, PWM, and Auto-SPI IMU
```

---

## 2. Dual Transport Modes

| Transport Mode | Interface Device | Hardware Path | Advantages |
| :--- | :--- | :--- | :--- |
| **`TransportMode::RpMsg_VirtIo`** | `/dev/rpmsg0` | Linux ARM $\leftrightarrow$ A5E RISC-V Co-Processor $\leftrightarrow$ FPGA | Zero-copy shared memory virtio ring; RISC-V offloads FPGA control |
| **`TransportMode::SpiDev_Direct`** | `/dev/spidev0.0` | Linux ARM $\leftrightarrow$ FPGA | Direct hardware bypass mode for FPGA register debugging |
