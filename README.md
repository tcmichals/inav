# `inav-abstractx` Flight Control Engine (`main` branch)

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
3. **Master Linux Quad-Core Flight System Specification**: Master system specification ([`docs/LINUX_SYSTEM_ARCHITECTURE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_SYSTEM_ARCHITECTURE.md)) detailing CPU Core 3 (Flight Control), Core 2 (Hardware I/O Reactor), Core 1 (RPMsg RISC-V/FPGA Transport), and Core 0 (Linux OS, TCP 5760, UDP 19000, Disk Logger).
4. **RP2350 Pico 2 W Wi-Fi & USB Configurator Target**: Target specification ([`docs/PICO2W_PINOUT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2W_PINOUT_SPEC.md)) detailing CYW43439 Wi-Fi AP (`INAV_PICO2W` IP `192.168.4.1:5760`) and USB CDC virtual COM port.
5. **iNav Configurator TCP Connector (Port 5760)**: Setup guide ([`docs/INAV_CONFIGURATOR_SETUP.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/INAV_CONFIGURATOR_SETUP.md)).
6. **TCP vs UDP Protocol Transport**: Protocol specification ([`docs/TELEMETRY_TRANSPORT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/TELEMETRY_TRANSPORT_SPEC.md)).
7. **Coroutine Telemetry & `co_await` Flow**: End-to-end telemetry streaming flow specification ([`docs/COROUTINE_TELEMETRY_FLOW_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/COROUTINE_TELEMETRY_FLOW_SPEC.md)).
8. **Master Validation Pipeline (`tools/run_all_validations.py`)**: Unified Python runner ([`docs/PYTHON_VALIDATION_WORKFLOW.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PYTHON_VALIDATION_WORKFLOW.md)).
9. **Offline Logic Analyzer Signal Testing**: Verification guide ([`docs/LOGIC_ANALYZER_TESTING_GUIDE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LOGIC_ANALYZER_TESTING_GUIDE.md)).
10. **Hardcore Hardware Validation Guidelines**: Failure mode testing checklist ([`docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/HARDCORE_HARDWARE_TESTING_GUIDELINES.md)).
11. **Hardware Validation Target (`pico2_hw_test`)**: Dedicated validation binary running on Pico 2 W hardware and Linux.
12. **Hardware Abstraction**: AbstractX PCIe TLP BAR memory interface (`asp_tlp64.hpp`).
13. **RP2350 Dual-Core Offloading**: Core 0 manages PIO state machines, Core 1 runs 8 kHz C++20 coroutine flight loop.
14. **Dynamic Motor Pin Multiplexing**: PIO0 microcode hot-swapping on GPIO pins 2..5.
15. **Zero Dynamic Allocation**: C++20 coroutines execute within static promise pools (`CoroutineStaticPool`).

---

## Documentation Index

- [`docs/LINUX_SYSTEM_ARCHITECTURE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LINUX_SYSTEM_ARCHITECTURE.md): Master Linux quad-core flight system architecture & threading specification.
- [`docs/INAV_CONFIGURATOR_SETUP.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/INAV_CONFIGURATOR_SETUP.md): iNav Configurator GUI setup & MSP command integration guide.
- [`docs/TELEMETRY_TRANSPORT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/TELEMETRY_TRANSPORT_SPEC.md): TCP vs UDP network telemetry transport specification.
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
- [`docs/RTOS_VS_COROUTINES_ANALYSIS.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/RTOS_VS_COROUTINES_ANALYSIS.md): Real-time execution model analysis comparing preemptive RTOS and C++20 coroutines.
- [`docs/LEGACY_VS_NEW_DIFFERENTIAL_TESTING.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/LEGACY_VS_NEW_DIFFERENTIAL_TESTING.md): Differential parity testing specification (`tools/compare_inav_parity.py`).
- [`docs/UNIT_TEST_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/UNIT_TEST_SPEC.md): Unit test suite specification (`./run_unit_tests`).
- [`docs/PICO2W_PINOUT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PICO2W_PINOUT_SPEC.md): GPIO pin allocation table for RP2350 Pico 2 W.
- [`docs/PORTABLE_MATH_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PORTABLE_MATH_SPEC.md): Portable C++20 math specification.
- [`docs/BUILD_GUIDE.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/BUILD_GUIDE.md): CMake build instructions for Linux SITL, RP2350 Pico 2, and Linux SBC + FPGA targets.
- [`docs/FULL_PROJECT_REVIEW_AUDIT.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/FULL_PROJECT_REVIEW_AUDIT.md): Engineering audit covering concurrency, memory model, and build verification.
- [`docs/SOURCE_LINE_AUDIT.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/SOURCE_LINE_AUDIT.md): Source file line count and module responsibility audit.
