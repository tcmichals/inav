# `rt_offloader` FPGA vs RP2350 PIO Hardware Offloading Architectural Synergy

> [!IMPORTANT]
> **UNIFIED HARDWARE OFFLOADING PATTERN ACROSS FPGA & PICO 2**
> The hardware offloading architecture in `inav-abstractx` mirrors the **`rt_offloader` FPGA project** (Zynq-7020 / Allwinner Cubie A5E).
>
> Both systems offload real-time signal generation (DShot, OneShot, PWM, 4-Way BLHeli Serial, and Auto-SPI IMU bursts) onto dedicated hardware state machines over the **AbstractX PCIe TLP BAR interface** ([`asp_tlp64.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/asp_tlp64.hpp))!

---

## 1. Architectural Parity: `rt_offloader` FPGA vs RP2350 PIO

| Subsystem / Protocol | `rt_offloader` FPGA Engine (Zynq / Cubie) | RP2350 PIO State Machine (Pico 2 W) | Unified Software API |
| :--- | :--- | :--- | :--- |
| **DShot Motor Waveforms** | FPGA Verilog DShot Core (`0x4000`) | PIO0 State Machine 0..3 (`pio_dshot.pio`) | [`dshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/dshot.hpp) |
| **OneShot & PWM ESCs** | FPGA Verilog PWM Generator (`0x4010`) | PIO0 State Machine 0..3 (`pio_pwm.pio`) | [`pwm_esc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/pwm_esc.hpp) |
| **8-Bit BLHeli 4-Way Serial**| FPGA UART Bridge (`0x4020`) | PIO0 1-Wire UART (`pio_blheli_4way.pio`)| [`pio_esc_reloader.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pio_esc_reloader.hpp) |
| **Auto-SPI IMU Reader** | FPGA SPI Master DMA (`0x1000`) | PIO2 Auto-SPI DMA (`pio_imu_reader.hpp`)| [`imu_pcie_driver.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu_pcie_driver.hpp) |
| **RC Serial Receiver** | FPGA UART FIFO (`0x3000`) | PIO1 CRSF Capture (`crsf.hpp`) | [`crsf.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/crsf.hpp) |

---

## 2. Dynamic Hardware Reconfiguration

Just as the **`rt_offloader` FPGA** dynamically reconfigures control registers over PCIe TLP memory writes (`TLP64` MemWrite to BAR offset `0x4000`), the **RP2350 Pico 2 Target Adapter** reloads PIO0 state machine microcode in $< 1\ \mu\text{s}$.

The C++20 flight loop code (`run_flight_loop()`) sees **ZERO difference** between running on a high-end Linux SBC + `rt_offloader` FPGA or a $5 RP2350 Pico 2 W!
