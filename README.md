# `inav-abstractx` Flight Control Engine (`pcie-clean` branch)

> **The Ultimate Fusion**: Merging **iNav Autonomous Navigation** with **Betaflight Modern Flight Dynamics** over the **AbstractX PCIe TLP Hardware Abstraction Layer**, using **iNav Configurator** for configuration and tuning.

---

## Vision

1. **iNav Autonomous Navigation**: RTH (Return-To-Home), 3D Waypoints, Position Hold, Safehomes, and Mission Planning.
2. **Betaflight-Grade Dynamics**: Modern C++20 PID controller with dynamic PT1/PT2 D-term filtering, Feedforward, and bidirectional DShot RPM filtering.
3. **iNav Configurator GUI Compatibility**: Uses MultiWii Serial Protocol (MSP v1/v2) to connect seamlessly to the official **iNav Configurator** for PID tuning, sensor plotting, and mission setup.
4. **AbstractX PCIe TLP Bus**: Hardware devices present a virtual 32-bit BAR address map (`PCIE_BAR_SYS`, `PCIE_BAR_IMU`, `PCIE_BAR_ESC`), running identically across **Linux SBC + FPGA** (Cubie A5E / Zynq-7020), **RP2350 Pico 2 PIO**, **STM32 Timer/MDMA**, and **Desktop SITL**.

---

## Core Specifications & Documentation

- [`docs/BETTER_THAN_BETAFLIGHT_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/BETTER_THAN_BETAFLIGHT_SPEC.md): Comprehensive architectural breakdown of the iNav + Betaflight fusion.
- [`docs/AI_DESIGN_GUARDRAILS.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/AI_DESIGN_GUARDRAILS.md): Mandatory design rules for AI and human developers enforcing zero dynamic allocation (`malloc`/`free`) and C++20 coroutine constraints.
- [`docs/PLATFORM_AGNOSTIC_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/PLATFORM_AGNOSTIC_SPEC.md): Virtual BAR address map & hardware offloader profiles.
- [`docs/COROUTINES_ZERO_ALLOC_SPEC.md`](file:///home/tcmichals/ssdData/projects/home/inav/docs/COROUTINES_ZERO_ALLOC_SPEC.md): Zero-heap C++20 coroutine specification.
