# `inav-abstractx` Flight Control Engine

[![C++20 Standard](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE.md)

---

## 1. Overview

`inav-abstractx` is a high-performance C++20 flight controller that unifies **INAV 3D autonomous navigation** with **Betaflight high-speed acro stick dynamics**. Powered by the **AbstractX C++20 API**, it completely eliminates legacy busy-waiting loops, blocking sleeps, and sequential register polling in favor of non-blocking stackless coroutines and 64-byte PCIe Transaction Layer Packets (`Tlp64`).

The system is deployed uniformly across four heterogeneous hardware platforms:
- **Allwinner Cubie A5E**: Linux `PREEMPT_RT` octa-core mission computer + RISC-V E906 co-processor via RPMsg.
- **Raspberry Pi Pico 2 / Pico 2 W**: RP2350 dual Cortex-M33 real-time flight core with triple PIO hardware offloading (DShot, CRSF, Auto-SPI IMU burst).
- **QMTECH Zynq-7020 (XC7Z020)**: Dual ARM Cortex-A9 + Xilinx Artix-7 FPGA fabric with single-cycle 64-byte parallel PCIe/AXI DMA streaming.
- **ESP32-P4-WIFI6 (Kit A / Slim)**: Dual-core 400 MHz RISC-V companion with 2-lane MIPI-CSI 120 fps optical flow camera and Wi-Fi 6 telemetry.
- **Linux Desktop SITL**: 6-DOF multicopter software-in-the-loop simulation with INAV Configurator TCP server (port 5760).

> [!NOTE]
> Upstream INAV's original documentation, setup instructions, and feature guide are preserved in [`docs/inav.md`](docs/inav.md).

---

## 2. Architecture & AbstractX API Mandate

### Zero Busy-Waiting Loops (`while/for` Register Elimination)
All legacy synchronous loops (`while (!ready) poll()`, `delay_ms()`) are strictly eliminated. All sensor drivers and flight engines operate exclusively via non-blocking C++20 coroutines:
- **`co_await tlp_channel.async_transaction(req)`**: Dispatches 64-byte PCIe TLPs over lock-free SPSC rings and suspends the coroutine ($<0.05\,\mu\text{s}$ swap) until hardware completion.
- **`when_any(hardware_task, timeout_task)` (`||`)**: Watchdog racing prevents bus lockups and achieves minimum latency upon DMA return.
- **`when_all(sensor1_init, sensor2_init)` (`&&`)**: Multi-sensor boot initialization executes in parallel, scaling total boot latency to $\max(\text{delays})$ instead of $\sum(\text{delays})$.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
│ TOP LEVEL: Sensor Drivers & Flight Core (C++20 Coroutines)                                │
│                                                                                           │
│  - Drivers: ICM-42688-P, BMP280, QMC5883L, MS4525DO, GPS (UBLOX), CRSF / ELRS             │
│  - Generates Transaction Layer Packets (TLP) via TlpChannel                               │
│  - Suspends execution via co_await on hardware I/O and conversion delays                  │
│  - Decoupled from physical register maps and MCU HAL implementations                     │
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
                                              │
                   [Outbound TLP SPSC Ring]   │   [Inbound TLP SPSC Ring]
                                              │
+─────────────────────────────────────────────▼─────────────────────────────────────────────+
│ BOTTOM HALF: Hardware I/O Scheduler & Transport Layer                                     │
│                                                                                           │
│  - Consumes TLP requests from the outbound ring buffer                                    │
│  - Routes requests by BAR address to target hardware backend:                             │
│      ├── Cubie A5E Linux SBC (Allwinner A523 Octa-Core Cortex-A55 PREEMPT_RT)             │
│      ├── Tang Nano 9K FPGA (Gowin GW1NR-9 64-byte TLP parallel hardware bridge)          │
│      ├── RP2350 PIO State Machines & DMA (PioImuReader, DShot PIO, CRSF PIO)               │
│      ├── ESP32-P4 Dual-Core RISC-V DMA, MIPI-CSI ISP & Wi-Fi 6 Companion Transport       │
│      ├── Microcontroller SPI / I2C / UART HAL                                             │
│      └── SITL Simulator / Test Injection Harness                                         │
│  - Pushes timestamped responses to inbound ring buffer to resume coroutines               │
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 3. Technical Rationale: Execution Models & TLP Sizing

### 3.1 Execution Models for Asynchronous Sensor Delays

Flight controllers must coordinate physical sensor settling and conversion delays without blocking real-time control loops:
- **MS5611 Barometer**: 9.04 ms delta-sigma ADC integration time.
- **ICM-42688-P IMU**: 2 ms internal RC oscillator lock and gyro power-up settling.
- **BMP280 Barometer**: 4.5 ms IIR filter conversion latency.
- **BMI088 IMU**: 50 ms accelerometer power-up delay.
- **SPI DMA / I2C Bus Transfers**: 50–500 µs asynchronous transfer durations.

#### Architectural Paradigms

1. **Preemptive Multithreading (RTOS Architecture)**:
   - Used in RTOS-based flight controllers (such as NuttX or ChibiOS).
   - Provides POSIX compliance, modular driver threading, and process isolation across dedicated tasks.
   - Requires per-thread stack memory allocation and thread synchronization primitives (mutexes, semaphores).

2. **Cooperative Periodic Schedulers (C Single-Stack Architecture)**:
   - Used in traditional embedded flight controllers (such as upstream INAV and Betaflight).
   - Highly lightweight single-stack design executing periodic callback tasks via timer checks (`micros()`) and explicit state enums.

3. **Stackless C++20 Coroutines (`inav-abstractx` Architecture)**:
   - Combines linear, sequential code structure (`co_await sleep_ms(10)` / `co_await tlp_channel.send_recv()`) with non-blocking cooperative scheduling.
   - The compiler generates suspension state records into pre-allocated static memory pools (`CoroutineStaticPool`), ensuring **zero dynamic heap allocation** (`malloc`, `free`, `new`, `delete`) in the real-time flight path.
   - Provides sub-microsecond suspension overhead ($<0.05\text{ µs}$) on a single execution stack.
   - Employs parallel combinators:
     - `when_any(io_task, timeout_task)` (`||`): Races hardware transactions against hardware watchdog timers for immediate return upon DMA completion.
     - `when_all(init_imu, init_baro, init_mag)` (`&&`): Executes multi-sensor initialization concurrently during boot.

#### Architectural Trade-Off Matrix

| Design Dimension | Preemptive RTOS (Threads) | Cooperative Task Scheduler (C) | Stackless Coroutines (C++20) |
| :--- | :--- | :--- | :--- |
| **Execution Paradigm** | Preemptive OS threads | Cooperative tick-based task array | Cooperative stackless coroutines |
| **Stack Memory Model** | Dedicated stack per thread ($2\text{–}8\text{ KB}$ each) | Single execution stack ($\approx 4\text{–}8\text{ KB}$) | Single execution stack + static coroutine pool |
| **Inter-Task Coordination** | Mutexes, semaphores, queues | Shared structs with rate guards | Lock-free SPSC TLP queues |
| **Context Switch Mechanism** | OS kernel register save/restore | Function call dispatch | Coroutine frame pointer swap |
| **Driver Control Flow** | Blocking OS calls or thread loops | Multi-step state machine enums | Sequential expressions with `co_await` |
| **Dynamic Memory Model** | Runtime thread/queue allocations | Static global memory | Static memory pools (0 runtime heap bytes) |

---

### 3.2 Parallel Boot Initialization (`when_all`)

By initiating sensor startup sequences concurrently, total cold-boot hardware initialization time scales to the longest individual sensor settling delay:

$$\text{Sequential Startup:} \quad T_{\text{boot}} = \sum_{i=1}^N T_{\text{init}, i} \approx 1.4\text{–}1.5\text{ s}$$

$$\text{Concurrent Startup (`when_all`):} \quad T_{\text{boot}} = \max_{i=1}^N(T_{\text{init}, i}) \approx 1.0\text{–}1.1\text{ s}$$

---

### 3.3 Why Variable-Length TLP on MCU vs. Fixed 64-Byte on Tang 9K FPGA?

The Transaction Layer Packet (TLP) abstraction decouples high-level flight dynamics from physical bus hardware (SPI, I2C, UART, PIO, PCIe, AXI DMA). However, memory constraints and bus architectures differ across hardware targets:

#### Microcontroller Domain (RP2350 Pico 2 / Pico 2 W, ESP32-P4 Slim)
- **SRAM Constraints**: Microcontrollers have tightly constrained internal memory (520 KB SRAM on RP2350, 768 KB L2 SRAM on ESP32-P4).
- **Payload Characteristics**: Most sensor register transactions transfer 1 to 6 bytes (e.g., 1-byte register writes or 6-byte IMU gyro/accel burst reads).
- **SPSC Ring Efficiency**: Enforcing fixed 64-byte allocations for every ring-buffer transaction would waste SRAM and increase CPU memory-copy overhead across internal buses. Therefore, software internal ring buffers use compact, variable-length TLP payloads (header + $N$-byte payload).

#### Hardware FPGA / Tang 9K Domain (Gowin GW1NR-9)
- **Wide Parallel Datapaths**: The Tang Nano 9K FPGA uses synchronous parallel internal buses and Block RAM FIFOs (26 B-SRAM blocks, 468 Kbits total, 64 Mbit PSRAM).
- **Deterministic Ingestion**: Fixed 64-byte alignment enables single-cycle hardware DMA ingestion and fixed-pipeline packet parsing without complex variable-length byte-realignment state machines or multi-cycle unaligned shifts.
- **Boundary Translation**: When crossing into or out of the Tang 9K FPGA or Cubie A5E PCIe/DMA boundaries, the software transport layer pads outbound packets to 64 bytes (512 bits) and strips padding from inbound completions.

---

## 4. Subsystem Implementation & Upstream Parity

Algorithms are ported from upstream C sources in [`external/inav/src/main/`](external/inav/src/main/) and [`external/betaflight/src/main/`](external/betaflight/src/main/):

| Subsystem | Upstream Reference | C++20 Implementation | Description |
| :--- | :--- | :--- | :--- |
| **Mahony AHRS** | `inav/src/main/flight/imu.c` | [`src/flight/attitude.hpp`](src/flight/attitude.hpp) | 11-stage Mahony orientation filter with centrifugal acceleration compensation ($\vec{a}_{\text{cent}} = \vec{v} \times \vec{\omega}$), gravity vector weighting, and magnetometer/GPS fusion. |
| **Dynamic Gyro Notch** | `inav/src/main/flight/gyroanalyse.c` | [`src/flight/gyro_analyse.hpp`](src/flight/gyro_analyse.hpp) | 4-stage FFT analysis pipeline using Hanning windowing and parabolic peak interpolation for center frequency tracking. |
| **Dynamic Gyro LPF** | `inav/src/main/flight/dynamic_lpf.c` | [`src/flight/dynamic_lpf.hpp`](src/flight/dynamic_lpf.hpp) | 1st/2nd-order dynamic low-pass filter scaled to throttle and motor RPM. |
| **Rate PID & Acro** | `betaflight/src/main/flight/pid.c` | [`src/flight/pid.hpp`](src/flight/pid.hpp) | Betaflight Rate PID loop with Feedforward 2.0, Anti-Gravity quadratic boost, D-Min scaling, and PT1/PT2/Biquad filtering. |
| **Inertial Position Estimator**| `inav/src/main/navigation/navigation_pos_estimator.c` | [`src/flight/pos_estimator.hpp`](src/flight/pos_estimator.hpp) | 2nd-order complementary position estimator fusing accelerometer, barometer, and GPS data with continuous accelerometer bias tracking. |
| **3D Navigation Suite** | `inav/src/main/navigation/navigation.c` | [`src/flight/navigation.hpp`](src/flight/navigation.hpp) | Waypoint loitering, S-curve kinematic velocity profiling ($v = \min(v_{\text{max}}, \sqrt{2ad})$), and 3-phase Return-to-Home (RTH). |
| **AutoTune & EZ-Tune** | `inav/src/main/flight/pid_autotune.c`<br>`ez_tune.c` | [`src/flight/autotune.hpp`](src/flight/autotune.hpp)<br>[`src/flight/ez_tune.hpp`](src/flight/ez_tune.hpp) | Åström-Hägglund relay limit-cycle tuning and 3-slider parameter synthesis. |
| **DShot RPM Filtering** | `inav/src/main/flight/rpm_filter.c` | [`src/flight/rpm_filter.hpp`](src/flight/rpm_filter.hpp) | Cascaded biquad notch filters tracked to motor telemetry RPM harmonics. |
| **Flight Mixer** | `inav/src/main/flight/mixer.c` | [`src/flight/mixer.hpp`](src/flight/mixer.hpp) | Multi-rotor and fixed-wing motor/servo output mixing, airmode logic, and torque limiting. |
| **Failsafe System** | `inav/src/main/flight/failsafe.c` | [`src/flight/failsafe.hpp`](src/flight/failsafe.hpp) | 2-stage failsafe engine handling signal loss detection, level descent, and emergency landing/RTH procedures. |
| **Blackbox CTF Logging** | `inav/src/main/blackbox/blackbox.c` | [`src/logging/blackbox_ctf.hpp`](src/logging/blackbox_ctf.hpp) | Common Trace Format binary event logging and high-rate flight data streaming. |
| **MSP Serial Protocol** | `inav/src/main/io/msp.c` | [`src/msp/msp_protocol.cpp`](src/msp/msp_protocol.cpp) | MSP v1 and MSP v2 serialization compatible with INAV Configurator (TCP port 5760). |

---

## 5. Supported Target Platforms

| Target Platform | Processor / Architecture | Peripherals & Interfaces | Role |
| :--- | :--- | :--- | :--- |
| **Allwinner Cubie A5E** | Octa-Core Cortex-A55 Linux `PREEMPT_RT` + RISC-V E906 (400 MHz) | RPMsg / VirtIO async channels, isolated CPU Core 3 (`SCHED_FIFO 99`), epoll peripheral manager. | High-level autonomous mission computer & coprocessor bridge |
| **RP2350 Pico 2 / Pico 2 W** | Dual ARM Cortex-M33 (150 MHz) | PIO0 DShot150..1200, PIO1 CRSF RX, PIO2 Auto-SPI IMU burst DMA, CYW43439 Wi-Fi (MSP TCP 5760). | Real-time flight control loop (8 kHz) & PIO actuator engine |
| **QMTECH Zynq-7020 (XC7Z020)** | Dual ARM Cortex-A9 (667–800 MHz) + Artix-7 FPGA (85K logic cells) | Single-cycle 64-byte parallel AXI-HP DMA mailboxes, multi-IMU hardware voting, zero-CPU bus offload. | High-throughput FPGA DMA bridge & sensor coprocessor |
| **ESP32-P4-WIFI6 (Kit A / Slim)** | Dual-Core RISC-V HP (400 MHz) + Single-Core LP (40 MHz) | 2-lane MIPI-CSI (120 fps optical flow ISP), high-speed SPI/DMA IMU, 802.11ax Wi-Fi 6 (>50 Mbps CTF stream). | High-performance vision pipeline & wireless telemetry companion |
| **Linux Desktop SITL** | x86_64 / ARM64 Linux Host | 6-DOF multicopter dynamics simulation, TCP 5760 MSP Configurator interface, UDP telemetry stream. | Software-in-the-Loop simulation & testing |

---

## 6. Verification & Quality Assurance

The validation pipeline verifies numerical accuracy, timing constraints, and system integration:

1. **Unit Test Suite** (`build/run_unit_tests`):
   - CppUTest test groups validating mathematical operations, filter attenuation, state transitions, and state machines.

2. **Submodule Differential Tests** (`build/submodule_differential_test`):
   - Compares C++20 output against upstream C functions in `external/inav` and `external/betaflight`.

3. **Scheduler Benchmark** (`build/scheduler_benchmark`):
   - Measures loop execution latency, jitter, throughput, and validates zero dynamic heap allocation in the flight loop.

4. **Full-Stack Mission Parity Test** (`build/full_stack_parity_test`):
   - Runs simulated 60-second autonomous missions comparing full state trajectories between C and C++20 implementations.

5. **Hardware Diagnostic Harness** (`build/pico2_hw_test`):
   - Hardware register validation, loopback tests, and peripheral verification.

---

## 7. Build Instructions & Usage

### Prerequisites
- GCC / Clang with C++20 support
- CMake 3.20+
- Ninja or Make
- Python 3

### Building & Running on Linux Host (SITL & Cubie A5E)

```bash
# 1. Configure and build all host targets (SITL, test suites, benchmarks, SBC binary)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 2. Run automated validation pipeline
python3 tools/run_all_validations.py

# 3. Run individual test binaries
./build/run_unit_tests
./build/submodule_differential_test
./build/scheduler_benchmark
./build/full_stack_parity_test

# 4. Run SITL simulator
./build/inav_abstractx_sitl

# 5. Run Cubie A5E SBC flight executable
./build/inav_linux_sbc
```

### Building RP2350 Pico 2 / Pico 2 W Firmware

```bash
# Configure and build Pico 2 W firmware (.uf2 / .elf)
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release
cmake --build build_pico2w -j$(nproc)
```

### Building ESP32-P4-WIFI6 Firmware

```bash
# Configure and build ESP32-P4 firmware via ESP-IDF / CMake
cmake -B build_esp32p4 -DESP_PLATFORM=1 -DIDF_TARGET=esp32p4 -DCMAKE_BUILD_TYPE=Release
cmake --build build_esp32p4 -j$(nproc)
```

### Diagnostic & Bench Utilities

```bash
# Flight controller bench diagnostic dashboard
python3 tools/gui_fc_bench_dashboard.py

# Interactive pre-flight sensor calibration wizard
python3 tools/guided_calibration_and_setup.py

# Blackbox CTF log decoder & CSV converter
python3 tools/ctf_to_blackbox.py
```

---

## 8. Documentation Index

- [`docs/inav.md`](docs/inav.md): Upstream INAV original documentation, setup instructions, and feature guide.
- [`docs/TARGET_HARDWARE_PLATFORMS.md`](docs/TARGET_HARDWARE_PLATFORMS.md): Detailed hardware architecture and zero-loop AbstractX execution model across Cubie A5E, Pico 2 W, QMTECH Zynq-7020, and ESP32-P4.
- [`docs/INAV_PORTING_MASTER_PLAN.md`](docs/INAV_PORTING_MASTER_PLAN.md): Persistent subsystem porting checklist and upstream mapping tracker.
- [`docs/ENGINEERING_LOG.md`](docs/ENGINEERING_LOG.md): Architectural decisions, hardware facts, and engineering change chronicle.
- [`CLAUDE.md`](CLAUDE.md): Multi-agent pairing guide, build/test commands, and safety rules for Claude Code.
- [`.agents/skills/`](.agents/skills/): Antigravity / agy skills (`inav-parity-validation`, `engineering-log`).
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): System architecture, coroutine execution model, TLP bus protocol, and memory layouts.
- [`docs/HARDWARE_DRIVERS.md`](docs/HARDWARE_DRIVERS.md): Peripheral driver catalog and coroutine lifecycle.
- [`docs/TARGET_PICO2.md`](docs/TARGET_PICO2.md): RP2350 board layout, PIO allocation, and pin assignments.
- [`docs/PICO2W_WIRING_AND_SETUP.md`](docs/PICO2W_WIRING_AND_SETUP.md): Wiring diagram, hardware setup, and flashing guide for Pico 2 W.
- [`docs/BUILD_GUIDE.md`](docs/BUILD_GUIDE.md): Cross-compilation toolchain and SDK setup.
- [`docs/BLACKBOX_LOGGING.md`](docs/BLACKBOX_LOGGING.md): CTF binary logging specification and extraction.
- [`docs/CONFIGURATOR_AND_MSP.md`](docs/CONFIGURATOR_AND_MSP.md): INAV Configurator TCP/UART setup and MSP protocol integration.
- [`docs/FLIGHT_ESTIMATION_AND_CONTROL.md`](docs/FLIGHT_ESTIMATION_AND_CONTROL.md): Mathematical formulations for estimators and controllers.
- [`docs/FIRST_FLIGHT_MAIDEN_CHECKLIST.md`](docs/FIRST_FLIGHT_MAIDEN_CHECKLIST.md): Maiden flight procedures and safety checklist.
- [`docs/CALIBRATION_SETUP_AND_PREFLIGHT_GUIDE.md`](docs/CALIBRATION_SETUP_AND_PREFLIGHT_GUIDE.md): Sensor calibration and pre-flight field checklist.
- [`docs/SITL_SIMULATOR.md`](docs/SITL_SIMULATOR.md): 6-DOF Software-in-the-Loop simulator configuration.
- [`docs/TESTING_AND_VALIDATION.md`](docs/TESTING_AND_VALIDATION.md): Test suite architecture and differential validation harness.
- [`LICENSE.md`](LICENSE.md): GNU General Public License v3.0 text and upstream attribution.



