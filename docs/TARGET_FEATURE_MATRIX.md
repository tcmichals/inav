# Target Platform Feature Matrix (`tcmichals/inav`)

> [!IMPORTANT]
> **PLATFORM INDEPENDENCE**
> Thanks to **AbstractX Virtual BAR Memory Mapping** and **C++20 Zero-Allocation Coroutines**, the exact same C++20 flight software engine runs across all three targets without code modification.
>
> Below is the feature and hardware transport matrix across our 3 target platforms:

---

## Target Feature Comparison Matrix

| Feature / Capability | 1. Desktop SITL / HIL Simulator | 2. Linux SBC + FPGA (Cubie A5E / Zynq) | 3. RP2350 Pico 2 (Pico 2) |
| :--- | :--- | :--- | :--- |
| **Target Directory** | `src/target/sitl/` | `src/target/linux_sbc/` | `src/target/pico2_rp2350/` |
| **Physical Transport** | Virtual BAR memory struct | `/dev/asp_dma` Char Dev / Dual-SPI | Lock-free SRAM SPSC 64B TLP ring |
| **IMU Sensor Polling** | Simulated ICM-42688-P Auto-DMA | SystemVerilog Hardware RTL Core | Core 0 + PIO State Machines |
| **Hardware Timestamping**| 64-bit simulated nanosecond clock | Sub-microsecond FPGA clock ($<20\text{ ns}$) | PIO Timer Counter ($<1\ \mu\text{s}$) |
| **iNav Configurator MSP** | TCP `5760` & Pseudo-tty (`/dev/pty`)| USB VCP, UART Serial, TCP Wi-Fi | USB VCP & UART Serial |
| **BareCTF Binary Tracing** | UDP `19000`, TCP `19000`, File | UDP Broadcast `19000`, File | USB TLP Stream, UART Stream |
| **Motor Output Engine** | Simulated DShot / OneShot | FPGA Hardware DShot MAC | RP2350 PIO DShot Generator |
| **Airframe Support** | All 8 Presets (Quad, Hex, Octo, VTOL)| All 8 Presets (Quad, Hex, Octo, VTOL)| All 8 Presets (Quad, Hex, Octo, VTOL)|
| **BLHeli/AM32 Passthrough**| Emulated 4way-if | 4way-if over `/dev/asp_dma` | 4way-if over USB Serial |

---

## Target Implementation Details

### Target 1: Desktop SITL / HIL Simulator (`src/target/sitl/`)
- Runs natively on Linux x86_64 or ARM laptops.
- Allows running full hardware-in-the-loop (HIL) flight simulations and connecting **iNav Configurator** live over `localhost:5760`.

### Target 2: Linux SBC + FPGA (`src/target/linux_sbc/`)
- Targeted for **Allwinner Cubie A5E**, **QMTECH Zynq-7020**, or Raspberry Pi Linux SBCs paired with FPGA coprocessors.
- Hardware offloader RTL ([`asp_imu_auto_dma.sv`](file:///home/tcmichals/ssdData/projects/home/AbstractX/rtl/imu/asp_imu_auto_dma.sv)) handles 100% of SPI clocking and latches hardware nanosecond timestamps on the exact clock cycle `DRDY` fires.

### Target 3: RP2350 Pico 2 (`src/target/pico2_rp2350/`)
- Uses dual-core ARM Cortex-M33 / Hazard3 RISC-V on **Raspberry Pi RP2350**.
- **Core 0**: Manages PIO state machines, SPI sensor polling, and pushes 64B TLPs into shared SRAM SPSC ring buffer.
- **Core 1**: Executes the `inav-abstractx` C++20 flight loop with zero SPI polling stalls!
