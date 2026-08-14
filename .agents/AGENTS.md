# Workspace Rules for `tcmichals/inav`

## Communication & Documentation Style Guidelines
1. **Direct Yes/No**: Start technical answers with a direct, unambiguous "Yes" or "No".
2. **Raw Facts & Data**: Provide pure technical facts, code links, exact numbers, hardware specs, memory numbers, and timing metrics.
3. **Critical Thinking & Trade-Offs**: Include objective evaluation of hardware constraints, trade-offs, and edge cases.
4. **Professional & Respectful Tone**: Keep all documentation (.md) and responses strictly neutral, constructive, and respectful. Avoid arrogant, dismissive, or opinionated phrasing.
5. **Git Commit Freeze**: **Do NOT execute `git commit` or `git push`** until explicitly instructed by the user. Keep all work live in the local working tree.

## Algorithmic Fidelity & Porting Rules
1. **Master Porting Plan**: Always reference, update, and follow the master checklist in [`docs/INAV_PORTING_MASTER_PLAN.md`](../docs/INAV_PORTING_MASTER_PLAN.md).
2. **Reference-First Parity**: When implementing or auditing flight dynamics, sensor filters, or navigation, cross-reference the exact upstream C source files in:
   - Upstream INAV submodule: `external/inav/src/main/` (or `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/`)
   - Upstream Betaflight submodule: `external/betaflight/src/main/` (or `/home/tcmichals/ssdData/projects/home/flightcode/betaflight/src/main/`)
3. **Zero Toy Stubs**: Never implement textbook approximations (e.g., raw unfiltered PID derivative, fixed-gain EKF, or empty switch statements). Every filter stage, anti-windup clamp, transition curve, and state machine from the reference C files must be faithfully implemented.
4. **Evidence-Based Completion**: Never claim a feature is "100% Complete" in documentation unless all mathematical formulas, constants, and filter stages from the reference C files have been ported and validated with unit tests.


## Architectural & Design Rules
1. **Flexible Software TLP Sizing & FPGA 64-Byte Padding**: In software (Linux and microcontrollers), TLP packet lengths can be whatever size they naturally need to be (sized payloads with a length header field to conserve SRAM and minimize memory copy overhead). When crossing into or out of an FPGA or PCIe hardware domain, software pads (or unpads) the packet to exactly **64 bytes (512 bits)** to meet the FPGA's single-cycle parallel bus and DMA requirements.
2. **Concurrent Asynchronous Waits & Zero Sequential Blind Sleeps (`when_any` / `when_all`)**: Never implement legacy sequential blind delays (e.g. `start_conversion()` followed by a blind `sleep(10ms)` followed by a blocking `read()`). All asynchronous hardware drivers (SPI DMA, I2C, UART, Barometer, Magnetometer, GPS, RC Receiver) MUST use concurrent parallel combinators:
   - **`when_any(hardware_io_task, timeout_task)` (`||`)**: Every hardware transaction must race concurrently against an explicit safety watchdog timer (`sleep_ms` / `sleep_us`) to prevent bus lockups and achieve minimum latency (immediate return upon DMA completion).
   - **`when_all(sensor1_task, sensor2_task)` (`&&`)**: Multi-sensor synchronization (e.g. Dual-IMU redundancy, EKF multi-sensor epochs) must execute concurrently across independent hardware channels and join without sequential CPU stalling.


## Testing & Quality Assurance Rules
1. **CppUTest Standard**: Every ported flight control module (filtering, Betaflight Rate PID, Mahony AHRS, INAV Position Estimator, AutoTune, EZ-Tune, Failsafe, Navigation) MUST have comprehensive CppUTest test groups (`TEST_GROUP`, `TEST`, `DOUBLES_EQUAL`, `CHECK_TRUE`) validating mathematical accuracy, filter attenuation, state transitions, and edge cases.

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




