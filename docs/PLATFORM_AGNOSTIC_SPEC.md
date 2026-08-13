# Platform-Agnostic Flight Architecture Specification (`tcmichals/inav` - `pcie-clean` branch)

## 1. Overview

The **`inav-abstractx`** flight control architecture decouples flight control algorithms (Attitude, EKF, PID control, Navigation, MSP) from physical microcontroller peripherals, interrupt vectors, and vendor HAL glue code.

By abstracting all hardware devices into a virtual **32-Bit BAR Address Space** accessed via **64-Byte Transaction Layer Packets (`asp-tlp-64b`)**, the flight control software runs unchanged across:
1. **Linux SBC + FPGA** (Allwinner Cubie A5E / QMTECH Zynq-7020 / Pi Zero Linux)
2. **RP2350 (Pico 2)** (Core 0 PIO TLP Offloader + Core 1 Flight Loop)
3. **STM32 (H7/F7/G4)** (Timer Input Capture + MDMA Circular TLP Ring)
4. **Desktop SITL Simulator** (Software-in-the-Loop native test target)

---

## 2. Virtual BAR Address Layout

All peripherals are accessed as memory-mapped 32-bit registers over the virtual PCIe BAR:

| Base Address | Region Name | Description |
| :--- | :--- | :--- |
| `0x40000000` | `PCIE_BAR_SYS` | System ID, WHO_AM_I, Wishbone Gateway, Status Registers |
| `0x40000100` | `PCIE_BAR_IMU` | Hardware Auto-DMA IMU Control, Status & Continuous Sensor Registers |
| `0x40000200` | `PCIE_BAR_ESC` | DShot ESC Motor Command Outflow & Bidirectional DShot Telemetry |
| `0x40000300` | `PCIE_BAR_BARO` | Barometer Control & Telemetry Stream |
| `0x40000400` | `PCIE_BAR_MAG` | Magnetometer Control & Telemetry Stream |
| `0x40000500` | `PCIE_BAR_SERIAL`| UART ESC Serial Tunnel & Telemetry Gateway |

---

## 3. Hardware Offloader Profiles

### Profile A: FPGA / Linux SBC (Cubie A5E / Zynq-7020)
- **RTL Engines**: SystemVerilog hardware cores ([`asp_imu_auto_dma.sv`](file:///home/tcmichals/ssdData/projects/home/AbstractX/rtl/imu/asp_imu_auto_dma.sv), [`asp_top.sv`](file:///home/tcmichals/ssdData/projects/home/AbstractX/rtl/asp_top.sv)).
- **Transport**: Dual-SPI or PCIe DMA character device driver (`/dev/asp_dma`).
- **Timestamping**: 64-bit nanosecond hardware clock latched on the exact FPGA clock cycle of `IMU_INT` assertion.

### Profile B: RP2350 (Pico 2 Dual-Core)
- **Offloader**: Core 0 + PIO state machines poll SPI IMUs, measure timing, and generate DShot.
- **Transport**: Lock-free SPSC TLP ring buffer in RP2350 SRAM shared between Core 0 and Core 1.
- **Flight Loop**: Core 1 runs the `inav-abstractx` flight loop without software SPI clocking stalls.

### Profile C: STM32 (H7 / F7 / G4)
- **Offloader**: Timer Input Capture latches timestamps on `DRDY` GPIO edges; MDMA autonomously fetches 14-byte SPI bursts directly into 64B TLP SRAM rings.
- **Transport**: Direct memory circular TLP ring buffer in STM32 SRAM.
- **Motor Output**: HRTIM / Advanced Timers driven by 64B ESC write TLPs.

---

## 4. Architectural Dataflow

```
+-----------------------------------------------------------------------------------+
|                           iNAV FLIGHT CONTROL LOOP                                |
|    (Attitude EKF3 -> Navigation Engine -> PID Controller -> Motor Dispatch)       |
+-----------------------------------------------------------------------------------+
                                         ▲
                                         │  Lock-Free SPSC 64B TLP Rings
                                         ▼
+-----------------------------------------------------------------------------------+
|                   C++20 ABSTRACTX PARALLEL I/O COROUTINE ENGINE                   |
|       - `co_await pcie_reg_read_async(addr)`                                      |
|       - `co_await tlp_stream_wait(channel)`                                       |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        CANONICAL 64-BYTE PCIe TLP BUS                              |
|   Opcode Types: MemRd (0x01), MemWr (0x02), CplD (0x03), DMA_Stream (0x10)         |
+-----------------------------------------------------------------------------------+
```
