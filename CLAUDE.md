# CLAUDE.md — Developer & AI Pairing Guide for `inav-abstractx`

This document defines repository standards, architecture, commands, and rules for Claude Code and AI assistants working in this repository.

---

## 1. Safety-Critical Avionics & Coding Rules

1. **Git Commit Freeze**: **Do NOT execute `git commit` or `git push`** unless explicitly instructed by the user. Keep all work live in the local working tree.
2. **Direct Yes/No**: Start technical answers with a direct, unambiguous "Yes" or "No".
3. **Raw Facts & Data**: Provide pure technical facts, code links, exact numbers, hardware specs, memory numbers, and timing metrics.
4. **Professional, Respectful & Humble Tone**: Strictly neutral, constructive, and respectful. Avoid hype or derogatory phrasing when describing flight stacks.
5. **Mandatory Engineering Log Updates**: All architectural decisions, bug fixes, upstream parity ports, and hardware platform changes MUST be logged in [`docs/ENGINEERING_LOG.md`](docs/ENGINEERING_LOG.md) following the standardized chronicle format defined in [`.agents/skills/engineering-log/SKILL.md`](.agents/skills/engineering-log/SKILL.md).
6. **Single TODO.md Master Roadmap Mandate**: Maintain strictly ONE authoritative `TODO.md` in the project root (`TODO.md`). Never create fragmented, secondary, or per-feature todo files. All milestones, active hardware integration tasks, multi-repo links, and roadmap items MUST reside exclusively within `TODO.md`.
6. **MISRA C++ Standard Adherence (MISRA C++:2008 / MISRA C++:2023)**:
   - **Zero Dynamic Allocation**: No `malloc`, `free`, `new`, or `delete` in flight execution paths.
   - **Fixed-Width Types**: Mandatory explicit `<cstdint>` types (`uint8_t`, `int16_t`, `uint32_t`, `uint64_t`, `float`, `double`). No naked `int`, `short`, `long`, or `char`.
   - **Explicit Casting**: Numeric conversions must use `static_cast<T>()`. No implicit narrowing conversions (`-Wconversion -Werror=vla`).
   - **Bounds Safety**: No raw unbounded pointer arithmetic. Use `std::span` and compile-time `std::array`.
   - **Const-Correctness & Noexcept**: All non-modifying functions and methods must be `const noexcept`.
6. **NASA/JPL "Power of 10" & DO-178C Principles**:
   - **Bounded Execution (WCET) & Zero Recursion**: Strictly no recursion. All loops must have a fixed, compile-time upper iteration bound.
   - **Check Return Values (`[[nodiscard]]`)**: All functions returning status or validation results must be annotated with `[[nodiscard]]`.
   - **Defensive Boundary Validation**: external sensor and RC inputs must be sanitized and range-checked before consumption.

---

## 2. Core Architecture: Decoupled Top-Level Coroutines & Bottom-Half TLP Scheduler

- **AbstractX API Mandate (Zero Busy-Waiting Loops)**:
  - All legacy busy-waiting loops (`while(!ready) wait()`, `delay_ms()`, register polling loops) are completely eliminated.
  - Everything runs via non-blocking C++20 coroutines yielding on `co_await tlp_channel.async_transaction()`, concurrent racing via `when_any(||)`, and multi-channel joins via `when_all(&&)`.
  - Enforced across all four hardware targets:
    * **Allwinner Cubie A5E**: PREEMPT_RT Linux host + RISC-V E906 co-processor via RPMsg / VirtIO.
    * **RP2350 Pico 2 W**: Dual Cortex-M33 + PIO hardware offloading (DShot, CRSF, IMU burst).
    * **QMTECH Zynq / FPGA**: Single-cycle 64-byte parallel PCIe/AXI TLP DMA streaming directly to SPSC rings.
    * **ESP32-P4**: MIPI-CSI optical flow vision pipeline + dual-core RISC-V coroutine reactor.
- **Queue Naming & Concurrency Invariant Mandate**:
  - Never share a single `SpscRing` among multiple producers (Core 0, PIO DMA, ISRs). A single SPSC ring is valid strictly for 1:1 point-to-point links.
  - Multi-producer contexts MUST be explicitly named and implemented as `MpscEventQueue` (`MpscTlpRing`) or `SpscChannelArray` (dedicated per-producer SPSC channels).
- **AbstractX Single-Source-of-Truth Mandate (Zero Code Duplication)**:
  - Never duplicate platform abstractions, reactor event loops, hardware abstraction layers (HAL), inter-core bridges, socket transports, or low-level peripheral drivers inside `tcmichals/inav`.
  - All such platform primitives belong exclusively in `AbstractX` (`external/AbstractX`).
  - If an architectural component, transport awaiter, bridge protocol, or driver capability is needed by INAV, it must be implemented or extended directly in `AbstractX` and consumed by INAV via header/library inclusion. INAV's scope is strictly flight control dynamics, navigation state machines, filtering mathematics, failsafe, and avionics safety.
- **Top Half (C++20 Stackless Coroutines)**:
  - Flight engine, sensors, and telemetry are non-blocking coroutines yielding via `co_await sleep_ms()` or `co_await tlp_channel.async_transaction()`.
  - Zero knowledge of physical SPI/I2C/GPIO pins.
- **Virtual Bus (AbstractX 64-Byte TLP)**:
  - Software uses flexible TLP packet lengths internally, padded to 64 bytes (512 bits) when crossing into hardware/FPGA boundaries.
- **Bottom Half (Hardware I/O Dispatcher)**:
  - Dispatches TLP requests across RP2350 PIO state machines, DMA SPI/I2C/UART masters, or Linux SBC / SITL simulator.

---

## 3. Algorithmic Fidelity & Submodule References

The project merges **INAV 3D Navigation & State Estimation** with **Betaflight Acro Dynamics & Filters**:
- **Upstream INAV Submodule**: [`external/inav/src/main/`](external/inav/src/main/)
- **Upstream Betaflight Submodule**: [`external/betaflight/src/main/`](external/betaflight/src/main/) (or local clean clone at `../betaflight`)
- **Master Checklist**: [`docs/INAV_PORTING_MASTER_PLAN.md`](docs/INAV_PORTING_MASTER_PLAN.md)
- **Zero Toy Stubs**: Every filter stage, anti-windup clamp, and state transition from the reference C sources must be faithfully implemented.

---

## 4. Build & Test Commands

```bash
# Configure and compile host Linux / SITL / Unit Test suite
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Run Submodule Differential Parity Tests (100% bit-exact against C sources)
./submodule_differential_test

# Run All 20 Unit Test Suites
./run_unit_tests

# Run Full-Stack 60-Second Flight Mission Parity
./full_stack_parity_test

# Run Coroutine Scheduler Benchmark
./scheduler_benchmark
```

---

## 5. Mandatory 4-Tier Review & SITL Simulation Protocol

Before declaring any feature or port complete, execute the 4-tier verification funnel:
1. **Tier 1 (Static Analysis & MISRA Guards)**: 0 dynamic allocations (`malloc`/`free`/`new`/`delete`), `<cstdint>`, bounded loops.
2. **Tier 2 (Differential Parity)**: `./submodule_differential_test` passes 100% bit-exact against C sources.
3. **Tier 3 (Unit Tests & Benchmark)**: `./run_unit_tests` (20 suites) and `./scheduler_benchmark` (100,000 cycles with 0 heap bytes).
4. **Tier 4 (6-DOF SITL Simulation)**: `./full_stack_parity_test` (60,000 ticks across 5 flight phases with 0.000000 attitude error) and live TCP 5760 MSP test via `python3 tools/test_sitl_integration.py`. Follow [`.agents/skills/sitl-mission-review/SKILL.md`](.agents/skills/sitl-mission-review/SKILL.md).

For RP2350 Pico 2 bare-metal builds:
```bash
mkdir -p build_pico2w && cd build_pico2w
cmake -DPICO_BOARD=pico2_w -DBUILD_PICO2=ON ..
make -j$(nproc) inav_abstractx_pico2w
```
