# Heterogeneous Hardware Targets & AbstractX Architecture Specification

This document specifies the four target hardware platforms supported by `inav-abstractx` and details how the **AbstractX C++20 API** eliminates legacy busy-waiting loops, blocking sleeps, and thread preemption jitter across all four architectures.

---

## 1. Architectural Mandate: AbstractX API & Zero-Loop Coroutine Model

Traditional embedded flight controllers (such as legacy INAV and Betaflight) rely heavily on synchronous busy-waiting loops and sequential polling:
```c
// LEGACY ANTI-PATTERN (Synchronous Polling & Blind Delays)
while (!sensor_ready()) { delay_ms(1); } // CPU blocked, drops 8x 8kHz flight loop cycles
spi_transfer(buf, len);                  // Synchronous CPU blocking transfer
```

The `inav-abstractx` architecture strictly eliminates all blocking loops across all platforms:
```cpp
// ABSTRACTX ASYNCHRONOUS COROUTINE PATTERN
// 1. Transaction Layer Packet (TLP) request dispatched to lock-free SPSC queue
// 2. Co-await yields CPU execution immediately (<0.05 us overhead)
// 3. Hardware transaction races concurrently against a hardware watchdog timer
auto res = co_await (tlp_channel.async_transaction(req) || sleep_us(TIMEOUT_US));
if (!res.has_value()) {
    // Handle hardware timeout gracefully without stalling flight core
}
```

```
+───────────────────────────────────────────────────────────────────────────────────────────+
| TOP LEVEL: Chip Drivers & Flight Core (C++20 Stackless Coroutines)                        |
|                                                                                           |
|  - Drivers (ICM-42688P, BMP280, QMC5883L, MS4525DO, GPS, CRSF, Optical Flow)              |
|  - Dispatches Outbound Tlp64 requests via TlpChannel                                      |
|  - Suspends execution non-blockingly: co_await async_transaction() or co_await sleep_ms()  |
|  - ZERO knowledge of physical SPI pins, I2C controllers, or hardware registers           |
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
                                              │
                   [Outbound TLP SPSC Ring]   │   [Inbound TLP SPSC Ring]
                   (Fixed 64B Tlp64 Requests) │   (Fixed 64B Tlp64 Completions)
                                              │
+─────────────────────────────────────────────▼─────────────────────────────────────────────+
| BOTTOM HALF: Hardware I/O Scheduler & Platform-Specific Transport Engine                  |
+───────────────────────────────────────────────────────────────────────────────────────────+
  ├── 1. Allwinner Cubie A5E: PREEMPT_RT Linux host + E906 RISC-V co-processor via RPMsg
  ├── 2. RP2350 Pico 2 W: Dual Cortex-M33 + PIO hardware state machines (DShot/CRSF/SPI)
  ├── 3. QMTECH Zynq-7020: Dual Cortex-A9 + FPGA Artix-7 single-cycle 64B parallel DMA
  └── 4. ESP32-P4: Dual-core 400 MHz RISC-V + MIPI-CSI optical flow + Wi-Fi 6 companion
```

---

## 2. Target 1: Allwinner Cubie A5E + Companion FPGA (Heterogeneous Tri-Compute Architecture)

### Hardware Specs & Compute Partitioning
- **Host Application Processor (Linux Host)**: Allwinner A523/A527 Octa-Core ARM Cortex-A55 (up to 1.8 GHz) running Linux `PREEMPT_RT`.
  - Runs INAV C++20 flight engine (`inav_linux_sbc`), navigation state machines, attitude fusion, blackbox logging, and MSP server over TCP 5760.
- **Real-Time Co-Processor (T-Head XuanTie E906 RISC-V @ 200–400 MHz)**:
  - Exclusive hardware ownership over sensor buses:
    * **SPI0 DMA**: 8 kHz ICM-42688-P IMU auto-burst.
    * **UART3**: 115200 baud, 10 Hz u-blox NEO-M8N GPS streaming with circular DMA.
    * **TWI1 (I2C)**: 100–200 Hz QMC5883L / HMC5883L 3-axis digital compass.
  - Packs readings into 64-byte `Tlp64` completion frames streamed over VirtIO RPMsg / Shared SRAM (`0x40000000`).
- **Companion FPGA**:
  - Offloads physical, microsecond-critical actuator and pulse-width timing signals via single-cycle 64-byte parallel PCIe/AXI TLP DMA:
    * **ESC Controller**: DShot150/300/600/1200 motor pulse synthesis with bidirectional telemetry RPM feedback.
    * **High-Speed Serial**: Hardware UART buffering & CRSF RC receiver frame parsing.
    * **NeoPixel LED Driver**: Strict $\pm 150\text{ ns}$ WS2812 timing synthesis.
    * **PWM Capture / Decode**: Multi-channel hardware timer input capture for legacy RC receivers.

### Execution Model & Loop Elimination
- **Linux Host Engine**: Pins real-time coroutine worker threads to isolated CPU cores (`pthread_setaffinity_np` on Core 3) with `SCHED_FIFO` priority 99 and page memory locking (`mlockall`).
- **epoll Event Reactor**: Eliminates busy loops on serial, RPMsg, and socket descriptors; incoming sensor TLPs wake coroutines via edge-triggered POSIX epoll.
- **Zero Inter-Processor Contention**: Cortex-A55 never touches physical sensor registers; XuanTie E906 never manages actuators; Companion FPGA never runs floating-point math. All inter-domain coordination happens exclusively through 64-byte `Tlp64` packets over lock-free rings.

---

## 3. Target 2: Raspberry Pi Pico 2 / Pico 2 W (RP2350)

### Hardware Specs
- **Processor**: Dual ARM Cortex-M33 (150 MHz, DSP + FPU) or dual Hazard3 RISC-V.
- **SRAM**: 520 KB SRAM partitioned into 10 banks.
- **Actuator & Bus Offloader**: 3 Programmable I/O (PIO) blocks (12 state machines total).
- **Wireless (Pico 2 W)**: Infineon CYW43439 Wi-Fi 4 / BT 5.2 via dedicated SPI.

### Execution Model & Loop Elimination
- **Core Partitioning**:
  - **Core 0**: Asynchronous Bottom-Half I/O dispatcher, telemetry streaming, flash parameter storage, and Wi-Fi MSP server.
  - **Core 1**: Real-time 8 kHz flight control loop (IMU filtering, EKF position estimation, Betaflight rate PID).
- **Triple-PIO Hardware Offloading**:
  - **PIO0 (DShot150..1200)**: Autonomous motor pulse train synthesis without CPU intervention.
  - **PIO1 (CRSF RC RX)**: Half-duplex bidirectional CRSF telemetry decoding with DMA circular buffer.
  - **PIO2 (Auto-SPI IMU Burst)**: Autonomous periodic 14-byte IMU burst read triggered directly by DRDY interrupt line into `spsc_tlp_ring`.
- **Zero Polling Loops**: Drivers suspend via C++20 `co_await tlp_channel.async_transaction()` or `co_await sleep_ms()`.

---

## 4. Target 3: QMTECH Zynq-7020 (XC7Z020 FPGA + Dual ARM Cortex-A9)

### Hardware Specs
- **Processing System (PS)**: Dual-Core ARM Cortex-A9 (667–800 MHz) with NEON SIMD and double-precision VFP.
- **Programmable Logic (PL)**: Xilinx 7-Series FPGA fabric (85,000 logic cells, 53,200 LUTs, 106,400 flip-flops, 4.9 Mb Block RAM).
- **Memory**: 512 MB – 1 GB DDR3 SDRAM.

### Execution Model & Loop Elimination
- **Single-Cycle 64-Byte Parallel TLP DMA**: Outbound TLP requests are written directly into memory-mapped AXI-HP DMA mailboxes. The FPGA fabric ingests the 64-byte (512-bit) parallel packet in a single clock cycle ($100\text{ MHz} \rightarrow 10\text{ ns}$).
- **Hardware Sensor Voting & Offload**: In multi-IMU redundant configurations, the FPGA fabric computes variance-weighted IMU sample averages and CRC checksums in hardware logic, presenting ready TLP frames to the SPSC ring with 0 CPU overhead.
- **Zero Polling Loops**: The ARM Cortex-A9 flight engine awaits completion tokens without polling AXI status registers.

---

## 5. Target 4: ESP32-P4 (Dual RISC-V + MIPI-CSI Camera Flow)

### Hardware Specs
- **High-Performance (HP) Cores**: Dual-Core 32-bit RISC-V (up to 400 MHz) with single-precision FPU and AI DSP extensions.
- **Low-Power (LP) Core**: Single-Core RISC-V (40 MHz) for standby and power monitoring.
- **Internal Memory**: 768 KB L2 HP SRAM + 32 KB LP SRAM + 16/32 MB PSRAM.
- **Vision Subsystem**: 2-lane MIPI-CSI camera interface with integrated Image Signal Processor (ISP).
- **Connectivity**: Companion Wi-Fi 6 (802.11ax) and Bluetooth 5.

### Execution Model & Loop Elimination
- **Optical Flow Vision Pipeline**: Dedicated hardware ISP captures 120 fps downward-facing grayscale frames. The optical flow Lucas-Kanade velocity delta is calculated and formatted as an AbstractX TLP sensor frame.
- **Zero Priority Inversion**: Suspending the vision task to await the next frame exposure does not hold up the 8 kHz IMU / PID flight engine. Both tasks yield cooperatively on their respective coroutines.
- **Wireless Telemetry**: Wi-Fi 6 streaming transmits live 64-byte CTF telemetry frames at >50 Mbps without blocking sensor buses.

---

## 6. Architecture Comparison Matrix

| Feature / Metric | Legacy C Flight Stack | Preemptive RTOS (Threads) | AbstractX C++20 Architecture |
| :--- | :--- | :--- | :--- |
| **Driver Control Flow** | Busy-wait `while(!ready)` loops | Blocking OS thread sleep | Non-blocking `co_await` coroutines |
| **Multi-Sensor Init** | Sequential sum: $\sum T_{\text{init}}$ | Individual thread creation | Concurrent `when_all(&&)`: $\max(T_{\text{init}})$ |
| **Timeout Protection** | Software counters in main loop | Thread watchdog timer | Concurrent racing `when_any(\|\|)` |
| **Hardware Coupling** | Direct MCU register writes | OS HAL device drivers | Abstracted 64-byte PCIe TLP packets |
| **Dynamic Memory** | Static global buffers | Thread stack allocations ($2\text{--}8\text{ KB}$) | Static pool recycling (0 dynamic heap bytes) |
| **Switch Latency** | Direct function call | $2.0\text{--}10.0\,\mu\text{s}$ (Context switch) | $<0.05\,\mu\text{s}$ (Frame pointer swap) |
