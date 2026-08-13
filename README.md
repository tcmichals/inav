# `inav-abstractx` Flight Control Engine (`pcie-clean` branch)

`inav-abstractx` is a C++20 port of the iNav autonomous navigation suite combined with Betaflight flight dynamics, implemented over the **AbstractX PCIe TLP Hardware Abstraction Layer**. It supports configuration via the **iNav Configurator** protocol over MSP.

---

## Quickstart

```bash
# Run Master Validation Pipeline (Builds executables & runs all Python/C++ test suites)
python3 tools/run_all_validations.py

# Build and run SITL Flight Engine on Linux
mkdir -p build && cd build
cmake .. && cmake --build .
./inav_abstractx_sitl

# Build and run Automated 9-Suite Unit Test
cmake --build . --target run_unit_tests && ./run_unit_tests

# Build and run On-Device Hardware Test Harness
cmake --build . --target pico2_hw_test && ./pico2_hw_test

# Run Offline Logic Analyzer CSV/VCD Signal Verification (Saleae / Kingst)
python3 tools/validate_logic_trace.py trace.csv

# Run Differential Parity Test Suite vs Reference Implementation
python3 tools/compare_inav_parity.py
```

To configure, open **iNav Configurator**, select **TCP** connection to `127.0.0.1:5760`, and click **Connect**.

---

## System Architecture

1. **Autonomous Navigation**: 3D Return-To-Home (RTH), Waypoints, Position Hold, and Safehomes.
2. **Flight Dynamics**: 3-axis PID controller with PT1/PT2 D-term filtering, Feedforward, and DShot motor output generation.
3. **TCP vs UDP Protocol Transport**: Protocol specification ([`docs/TELEMETRY_TRANSPORT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/TELEMETRY_TRANSPORT_SPEC.md)) detailing TCP Port 5760 for Configurator MSP tuning and UDP Port 19000 for zero-latency live telemetry streaming.
4. **Linux Disk I/O & TCP Background Threads**: Thread architecture specification ([`docs/LINUX_BACKGROUND_THREADS_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_BACKGROUND_THREADS_SPEC.md)).
5. **Linux `PREEMPT_RT` Enabled Hard Real-Time Target**: Real-time specification ([`docs/LINUX_PREEMPT_RT_SCHEDULER_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_PREEMPT_RT_SCHEDULER_SPEC.md)).
6. **Linux 1.0 GHz Single-Core Unified Execution**: Architecture specification ([`docs/LINUX_REALTIME_HARDENING_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_REALTIME_HARDENING_SPEC.md)).
7. **Linux Dedicated Thread-Per-Bus Peripheral Manager**: Architecture specification ([`docs/LINUX_PER_BUS_THREAD_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_PER_BUS_THREAD_SPEC.md)).
8. **Linux Native `epoll` Asynchronous Event Reactor**: Multi-peripheral event reactor specification ([`docs/LINUX_EPOLL_EVENT_REACTOR_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_EPOLL_EVENT_REACTOR_SPEC.md)).
9. **Allwinner Cubie A5E RISC-V + FPGA Synergy**: Hardware offloading specification ([`docs/A5E_RISCV_FPGA_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/A5E_RISCV_FPGA_SPEC.md)).
10. **Linux Asynchronous Non-Blocking I/O Dispatch**: Parallel hardware dispatch specification ([`docs/LINUX_ASYNC_IO_DISPATCH_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_ASYNC_IO_DISPATCH_SPEC.md)).
11. **Coroutine Telemetry & `co_await` Flow**: End-to-end telemetry streaming flow specification ([`docs/COROUTINE_TELEMETRY_FLOW_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/COROUTINE_TELEMETRY_FLOW_SPEC.md)).
12. **Master Validation Pipeline (`tools/run_all_validations.py`)**: Unified Python runner ([`docs/PYTHON_VALIDATION_WORKFLOW.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PYTHON_VALIDATION_WORKFLOW.md)).
13. **Offline Logic Analyzer Signal Testing**: Verification guide ([`docs/LOGIC_ANALYZER_TESTING_GUIDE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LOGIC_ANALYZER_TESTING_GUIDE.md)).
14. **Hardcore Hardware Validation Guidelines**: Failure mode testing checklist ([`docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md)).
15. **Hardware Validation Target (`pico2_hw_test`)**: Dedicated validation binary running on Pico 2 W hardware and Linux.
16. **Hardware Abstraction**: AbstractX PCIe TLP BAR memory interface (`asp_tlp64.hpp`).
17. **RP2350 Dual-Core Offloading**: Core 0 manages PIO state machines, Core 1 runs 8 kHz C++20 coroutine flight loop.
18. **Dynamic Motor Pin Multiplexing**: PIO0 microcode hot-swapping on GPIO pins 2..5.
19. **Zero Dynamic Allocation**: C++20 coroutines execute within static promise pools (`CoroutineStaticPool`).

---

## Documentation Index

- [`docs/TELEMETRY_TRANSPORT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/TELEMETRY_TRANSPORT_SPEC.md): TCP vs UDP network telemetry transport specification.
- [`docs/LINUX_BACKGROUND_THREADS_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_BACKGROUND_THREADS_SPEC.md): Linux disk I/O & TCP telemetry background thread specification.
- [`docs/LINUX_PREEMPT_RT_SCHEDULER_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_PREEMPT_RT_SCHEDULER_SPEC.md): Linux `PREEMPT_RT` real-time kernel enabled specification.
- [`docs/LINUX_REALTIME_HARDENING_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_REALTIME_HARDENING_SPEC.md): Linux 1.0 GHz single-core unified real-time execution specification.
- [`docs/LINUX_PER_BUS_THREAD_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_PER_BUS_THREAD_SPEC.md): Linux dedicated thread-per-bus peripheral architecture specification.
- [`docs/LINUX_EPOLL_EVENT_REACTOR_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_EPOLL_EVENT_REACTOR_SPEC.md): Linux native `epoll` asynchronous multi-peripheral event reactor specification.
- [`docs/A5E_RISCV_FPGA_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/A5E_RISCV_FPGA_SPEC.md): Allwinner Cubie A5E RISC-V co-processor & FPGA offloader specification.
- [`docs/LINUX_ASYNC_IO_DISPATCH_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_ASYNC_IO_DISPATCH_SPEC.md): Linux non-blocking SPI/I2C/UART asynchronous hardware dispatch specification.
- [`docs/COROUTINE_TELEMETRY_FLOW_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/COROUTINE_TELEMETRY_FLOW_SPEC.md): Lock-free SPSC TLP ring buffer & `co_await` telemetry streaming flow specification.
- [`docs/PYTHON_VALIDATION_WORKFLOW.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PYTHON_VALIDATION_WORKFLOW.md): Automated Python validation pipeline & file output workflow guide.
- [`docs/LOGIC_ANALYZER_TESTING_GUIDE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LOGIC_ANALYZER_TESTING_GUIDE.md): Offline Saleae 8-pin & Kingst logic analyzer signal verification guide.
- [`docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md): Comprehensive failure mode and hardware stress testing guidelines.
- [`docs/HARDWARE_VALIDATION_PROCEDURES.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/HARDWARE_VALIDATION_PROCEDURES.md): Step-by-step physical hardware validation guidelines.
- [`docs/PICO2_HARDWARE_TEST_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2_HARDWARE_TEST_SPEC.md): On-Device hardware validation specification (`pico2_hw_test`).
- [`docs/ENGINEERING_LOG.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/ENGINEERING_LOG.md): Chronological engineering log recording design decisions, changes, and system verifications.
- [`docs/PICO2_ARCHITECTURE_ANALYSIS.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2_ARCHITECTURE_ANALYSIS.md): RP2350 target architecture and design analysis.
- [`docs/RT_OFFLOADER_FPGA_SYNERGY.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/RT_OFFLOADER_FPGA_SYNERGY.md): Overview of hardware offloading parity between `rt_offloader` FPGA and RP2350 PIO state machines.
- [`docs/PICO2_PIO_DYNAMIC_RELOAD_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2_PIO_DYNAMIC_RELOAD_SPEC.md): Dynamic PIO microcode reloading specification for DShot, OneShot, PWM, and BLHeli serial passthrough.
- [`docs/PICO2_BARE_METAL_VS_RTOS.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2_BARE_METAL_VS_RTOS.md): Technical comparison of RP2350 bare-metal dual-core execution vs preemptive RTOS.
- [`docs/LINUX_REALTIME_HARDENING_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_REALTIME_HARDENING_SPEC.md): POSIX real-time thread hardening specification for Linux (`mlockall`, `SCHED_FIFO`, CPU affinity).
- [`docs/RTOS_VS_COROUTINES_ANALYSIS.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/RTOS_VS_COROUTINES_ANALYSIS.md): Real-time execution model analysis comparing preemptive RTOS and C++20 coroutines.
- [`docs/LEGACY_VS_NEW_DIFFERENTIAL_TESTING.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LEGACY_VS_NEW_DIFFERENTIAL_TESTING.md): Differential parity testing specification (`tools/compare_inav_parity.py`).
- [`docs/UNIT_TEST_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/UNIT_TEST_SPEC.md): Unit test suite specification (`./run_unit_tests`).
- [`docs/PICO2W_PINOUT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2W_PINOUT_SPEC.md): GPIO pin allocation table for RP2350 Pico 2 W.
- [`docs/PORTABLE_MATH_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PORTABLE_MATH_SPEC.md): Portable C++20 math specification.
- [`docs/BUILD_GUIDE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/BUILD_GUIDE.md): CMake build instructions for Linux SITL, RP2350 Pico 2, and Linux SBC + FPGA targets.
- [`docs/FULL_PROJECT_REVIEW_AUDIT.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/FULL_PROJECT_REVIEW_AUDIT.md): Engineering audit covering concurrency, memory model, and build verification.
- [`docs/SOURCE_LINE_AUDIT.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/SOURCE_LINE_AUDIT.md): Source file line count and module responsibility audit.
