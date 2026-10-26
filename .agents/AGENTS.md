# Workspace Rules for `tcmichals/inav`

## Communication & Documentation Style Guidelines
1. **Direct Yes/No**: Start technical answers with a direct, unambiguous "Yes" or "No".
2. **Raw Facts & Data**: Provide pure technical facts, code links, exact numbers, hardware specs, memory numbers, and timing metrics. Truthful with facts at all times.
3. **Critical Thinking & Trade-Offs**: Include objective evaluation of hardware constraints, trade-offs, and edge cases without hype or marketing spin.
4. **Professional, Respectful & Humble Tone**: Strictly neutral, constructive, and respectful. Never use arrogant, condescending, dismissive, or derogatory phrasing when describing or comparing systems, architectures, or other open-source projects (ArduPilot, PX4, INAV, Betaflight). State trade-offs objectively without being an asshole.
5. **Git Commit Freeze**: **Do NOT execute `git commit` or `git push`** until explicitly instructed by the user. Keep all work live in the local working tree.
6. **Mandatory Engineering Log Updates**: All architectural decisions, bug fixes, upstream parity ports, and hardware platform changes MUST be logged in [`docs/ENGINEERING_LOG.md`](../docs/ENGINEERING_LOG.md) following the standardized chronicle format defined in [`.agents/skills/engineering-log/SKILL.md`](skills/engineering-log/SKILL.md).
7. **Single TODO.md Master Roadmap Mandate**: Maintain strictly ONE authoritative `TODO.md` in the project root (`TODO.md`). Never create fragmented, secondary, or per-feature todo documents (e.g. `todo.txt`, `TODO_PHASE2.md`). All milestones, active hardware integration tasks, multi-repo links, and roadmap items MUST reside exclusively within `TODO.md`.
8. **Strict AbstractX Isolation Mandate (Zero Unapproved AbstractX Git Actions)**: Never execute git commits, pushes, or code modifications inside `AbstractX` (`external/AbstractX` or `/home/tcmichals/projects/AbstractX`) unless explicitly requested and authorized by the user. All commits and working tree operations MUST be strictly scoped to `tcmichals/inav`.

## Algorithmic Fidelity & Porting Rules
1. **Master Porting Plan**: Always reference, update, and follow the master checklist in [`docs/INAV_PORTING_MASTER_PLAN.md`](../docs/INAV_PORTING_MASTER_PLAN.md).
2. **Reference-First Parity**: When implementing or auditing flight dynamics, sensor filters, or navigation, cross-reference the exact upstream C source files in:
   - Upstream INAV submodule: `external/inav/src/main/` (or `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/`)
   - Upstream Betaflight submodule: `external/betaflight/src/main/` (or `/home/tcmichals/ssdData/projects/home/flightcode/betaflight/src/main/`)
3. **Zero Toy Stubs**: Never implement textbook approximations (e.g., raw unfiltered PID derivative, fixed-gain EKF, or empty switch statements). Every filter stage, anti-windup clamp, transition curve, and state machine from the reference C files must be faithfully implemented.
4. **Evidence-Based Completion**: Never claim a feature is "100% Complete" in documentation unless all mathematical formulas, constants, and filter stages from the reference C files have been ported and validated with unit tests.
5. **Sensor Initialization & Data Gathering Parity**: Sensor drivers and data ingestion pipelines must maintain complete parity with upstream INAV and Betaflight:
   - **Hardware Register Initialization**: Configure full Anti-Aliasing Filters (AAF), DLPF bandwidth, ODR, and full-scale ranges matching upstream.
   - **Zero-Motion Calibration**: Implement gyro bias calibration at boot ($N = 1000$ samples, variance check $\sigma < \text{threshold}$ to reject quad movement during arming).
   - **Board Alignment Transformations**: Support full 8-direction standard sensor rotations (`CW0_DEG`, `CW90_DEG`, `CW180_DEG`, `CW270_DEG`, flip orientations) and custom rotation matrices.
   - **Temperature Drift Compensation**: Implement gyro zero-bias temperature compensation ($K_T \cdot \Delta T$).
   - **Barometer Ground Pressure Calibration**: Capture ground pressure $P_0$ on arming for relative altitude calculation ($h = 44330 \cdot (1 - (P/P_0)^{1/5.255})$).
   - **Magnetometer Calibration**: Apply hard-iron offset $(b_x, b_y, b_z)$ and soft-iron scale matrix $(s_x, s_y, s_z)$ compensation.
   - **Battery, Pitot & Rangefinder**: Implement ADC voltage/current scaling, mAh capacity integration, differential dynamic pressure airspeed calculation ($v = \sqrt{2\Delta P/\rho}$), and LiDAR/sonar tilt compensation ($h = d \cdot \cos\theta \cos\phi$).


## Architectural & Design Rules
1. **Unified 64-Byte (512-bit) TLP Standard & Zynq BRAM Ingestion**: All hardware targets (Linux SBC, microcontrollers, and FPGA) standardize strictly on the unified 64-byte (512-bit) `asp_tlp64_t` packet format. This achieves exact 64-byte cache-line alignment on ARM Cortex-A55/A9 cores (~80 ns copy), fits cleanly into 512-byte VirtIO RPMsg buffers (8 TLPs/slot) without ring fragmentation, and maps directly into QMTECH Zynq-7020 dual 8 KiB PL BRAM FIFOs (`asp_axi_lite_bridge.sv` via `M_AXI_GP0` / `/dev/uio0`) for single-cycle 512-bit crossbar switching.
2. **Concurrent Asynchronous Waits & Zero Sequential Blind Sleeps (`when_any` / `when_all`)**: Never implement legacy sequential blind delays (e.g. `start_conversion()` followed by a blind `sleep(10ms)` followed by a blocking `read()`). All asynchronous hardware drivers (SPI DMA, I2C, UART, Barometer, Magnetometer, GPS, RC Receiver) MUST use concurrent parallel combinators:
   - **`when_any(hardware_io_task, timeout_task)` (`||`)**: Every hardware transaction must race concurrently against an explicit safety watchdog timer (`sleep_ms` / `sleep_us`) to prevent bus lockups and achieve minimum latency (immediate return upon DMA completion).
   - **`when_all(sensor1_task, sensor2_task)` (`&&`)**: Multi-sensor synchronization (e.g. Dual-IMU redundancy, EKF multi-sensor epochs) must execute concurrently across independent hardware channels and join without sequential CPU stalling.
3. **Sensor Physical Delays & Non-Blocking Coroutine Pends (`co_await sleep_ms / async_io`)**:
   - **Sensors Have Inherent Delays, But the System Never Pends On One Thing**: When a sensor requires physical settling time, reset duration, or ADC conversion latency (e.g. MS5611 9.04ms delta-sigma integration, ICM-42688-P 2ms oscillator lock, BMP280 filter delay, BMI088 50ms power-up), the delay is scoped strictly to that specific sensor's coroutine. The driver MUST yield as a C++20 coroutine (`co_await sleep_ms()` / `co_await sleep_us()`).
   - **Zero Priority Inversion & Full I/O Concurrency**: Suspending a specific sensor task to satisfy its physical delay must NEVER block the CPU or hold up independent I/O channels. While one sensor is pending its delay, all other subsystems (8kHz IMU sampling, PID loop, DShot motor output, CRSF RC receiver, GPS UART, Telemetry, Blackbox logging, and other sensor inits) MUST continue executing concurrently without interruption.
   - **Parallel Boot Initialization (`when_all`)**: All subsystem initialization sequences (`async_init`) must execute concurrently via `when_all` so total boot latency equals $\max(\text{sensor delays})$ rather than $\sum(\text{sensor delays})$.
4. **AbstractX API Mandate — Zero Polling Loops (`while/for` Register Elimination)**:
   - All legacy busy-waiting loops (e.g. `while(!ready) wait()`, `delay_ms()`, sequential register spins, and synchronous task queues) are strictly eliminated.
   - The system exclusively uses the **AbstractX C++20 API**: non-blocking stackless coroutines yielding via `co_await tlp_channel.async_transaction()`, lock-free 64-byte PCIe TLP frames (`Tlp64`), concurrent watchdog racing via `when_any(||)`, and multi-channel joins via `when_all(&&)`.
   - This architectural model is enforced uniformly across all four heterogeneous target platforms:
     * **Allwinner Cubie A5E**: PREEMPT_RT Linux host + RISC-V E906 co-processor via RPMsg / VirtIO.
     * **RP2350 Pico 2 W**: Dual Cortex-M33 + PIO hardware offloading (DShot, CRSF, IMU burst).
     * **QMTECH Zynq / FPGA**: Single-cycle 64-byte parallel PCIe/AXI TLP DMA streaming directly to SPSC rings.
     * **ESP32-P4**: MIPI-CSI optical flow vision pipeline + dual-core RISC-V coroutine reactor.
5. **Queue Naming & Concurrency Invariant Mandate (Zero Misleading 'SPSC' on Multi-Producer Contexts)**:
   - A single `SpscRing` is mathematically and architecturally valid **ONLY** when there is strictly **ONE** writer and **ONE** reader (point-to-point).
   - Never share a single `SpscRing` among multiple producers (e.g. Core 0 I/O processor, PIO DMA, and Core 1 coroutines/ISRs). Doing so causes silent race conditions on `head_`, leading to torn writes and corrupted frames.
   - **Mandatory Naming & Implementation Standards**:
     * **Point-to-Point (1 Producer : 1 Consumer)**: Name explicitly as `SpscTlpChannel` or `DedicatedSpscRing`.
     * **Multi-Producer (N Producers : 1 Consumer)**: Name explicitly as `MpscEventQueue` (`MpscTlpRing`) using atomic ticket reservation, interrupt locks, or hardware spinlocks (`push_from_isr`).
     * **Array of Dedicated Channels**: Name as `TlpChannelMultiplexer` or `SpscChannelArray<N>` where each producer has a private index `[0..N-1]` drained round-robin by the consumer with zero contention.
6. **AbstractX Single-Source-of-Truth Mandate (Zero Code Duplication)**:
   - Never duplicate platform abstractions, reactor event loops, hardware abstraction layers (HAL), inter-core bridges, socket transports, or low-level peripheral drivers inside `tcmichals/inav`.
   - All such platform primitives belong exclusively in `AbstractX` (`external/AbstractX`).
   - If an architectural component, transport awaiter, bridge protocol, or driver capability is needed by INAV, it must be implemented or extended directly in `AbstractX` and consumed by INAV via header/library inclusion. INAV's scope is strictly flight control dynamics, navigation state machines, filtering mathematics, failsafe, and avionics safety.


## Testing & Quality Assurance Rules
1. **CppUTest Standard**: Every ported flight control module (filtering, Betaflight Rate PID, Mahony AHRS, INAV Position Estimator, AutoTune, EZ-Tune, Failsafe, Navigation) MUST have comprehensive CppUTest test groups (`TEST_GROUP`, `TEST`, `DOUBLES_EQUAL`, `CHECK_TRUE`) validating mathematical accuracy, filter attenuation, state transitions, and edge cases.
2. **Mandatory 4-Tier Review & SITL Simulation Protocol**:
   Before claiming any algorithmic port, sensor driver, or architectural change is complete, it must be verified through the 4-tier verification funnel defined in [`.agents/skills/sitl-mission-review/SKILL.md`](skills/sitl-mission-review/SKILL.md):
   - **Tier 1: Static Analysis & MISRA Safety Guards**: Zero dynamic allocation in flight path, NASA/JPL Power of 10 bounds.
   - **Tier 2: Submodule Differential Parity**: Run `submodule_differential_test` verifying 100% bit-exact parity against `external/inav` and `external/betaflight`.
   - **Tier 3: Unit Tests & Scheduler Benchmark**: Run `run_unit_tests` (all 20 suites) and `scheduler_benchmark` (100,000 flight loop cycles, 0 heap bytes).
   - **Tier 4: Closed-Loop 6-DOF SITL Simulation**: Execute `full_stack_parity_test` (60,000 ticks across 5 flight phases with 0.000000 attitude error) and live TCP 5760 MSP handshake via `test_sitl_integration.py`.

## Safety-Critical & MISRA C++ Coding Rules
1. **MISRA C++ Standard Adherence (MISRA C++:2008 / MISRA C++:2023)**:
   - **Zero Dynamic Allocation**: No `malloc`, `free`, `new`, or `delete` in the flight execution path.
   - **Fixed-Width Types**: Mandatory use of explicit `<cstdint>` types (`uint8_t`, `int16_t`, `uint32_t`, `uint64_t`, `float`, `double`). No naked `int`, `short`, `long`, or `char`.
   - **Explicit Type Casting**: All numeric conversions must use explicit `static_cast<T>()`. No implicit narrowing conversions (`-Wconversion -Werror=vla`).
   - **Bounds Safety**: No raw unbounded pointer arithmetic. Use `std::span` and compile-time `std::array` with explicit size constraints.
   - **Const-Correctness & Noexcept**: All non-modifying functions and methods must be declared `const noexcept`.
2. **NASA/JPL "Power of 10" & DO-178C Avionics Principles**:
   - **Bounded Execution (WCET) & Zero Recursion**: Strictly no recursion. All loops must have a fixed, compile-time upper iteration bound ($N \le \text{MAX\_ITER}$) to ensure deterministic Worst-Case Execution Time.
   - **Check All Return Values (`[[nodiscard]]`)**: All functions returning status, validation results, or error codes must be annotated with `[[nodiscard]]` and checked by the caller.
   - **Defensive Boundary Validation**: All external sensor and RC inputs must be sanitized, range-checked, and glitch-gated (e.g. RC 1000–2000 µs clamping, GPS HDOP gating, Baro spike filters) before mathematical consumption.
   - **Fail-Safe Defaults**: All state machines, arming flags, and actuator setpoints must initialize and recover to a deterministic, safe state (disarmed / level descent) upon sensor loss or communication timeout.




