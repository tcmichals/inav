# Master Implementation & Porting Roadmap: `inav-abstractx`
## Permanent Engineering Plan & Phase Progress Tracker

> [!IMPORTANT]
> **PERMANENT REPOSITORY ROADMAP**
> This document tracks the implementation progress, mathematical requirements, upstream C source mappings, and status of all flight control modules in **`inav-abstractx`**.
> It serves as the single source of truth for the project roadmap across all development sessions.

---

## 1. Executive Summary & Core Project Requirements

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                    INAV-ABSTRACTX SYNTHESIS                               |
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
| WHAT WE LIFT FROM UPSTREAM INAV             | WHAT WE LIFT FROM UPSTREAM BETAFLIGHT       |
+─────────────────────────────────────────────┼─────────────────────────────────────────────+
| • 3D Autonomous Navigation (RTH/Waypoints)  | • Feedforward 2.0 (Jitter-free stick accel) |
| • Inertial-Complementary Position Estimator | • Anti-Gravity (Throttle-punchout boost)    |
| • Multicopter Braking & Deceleration Curves | • D-Min Dynamic D-Gain (Cooler motors)      |
| • Mahony AHRS with Centrifugal Correction   | • Cascaded PT1/PT2/PT3 & Biquad Filters     |
| • AutoTune (Relay Limit-Cycle Engine)       | • 1st/2nd-Order Gyro Kalman Noise Filter    |
| • EZ-Tune High-Level Parameter Synthesizer  | • RPM Harmonic & Dynamic Notch Filters      |
| • iNav Configurator MSP v1/v2 TCP Server    | • VBat Voltage Sag Compensation             |
+─────────────────────────────────────────────┴─────────────────────────────────────────────+
```

### Core Architectural Guardrails:
1. **High-Speed Multi-Rate Loop (Up to 16 kHz)**:
   * **16 kHz (62.5 µs)**: Gyro Kalman, PT2/PT3 Low-pass, Rate PID, PIO DShot600/1200.
   * **1 kHz (1.0 ms)**: Accelerometer integration, Mahony AHRS attitude, Angle mode, Blackbox logging.
   * **100 Hz (10 ms)**: Barometer altitude integration, continuous $b_{a,z}$ bias tracking, VBat comp.
   * **10 Hz (100 ms)**: GPS position/velocity fusion & 3D waypoint trajectory planning.
2. **C++20 Stackless Coroutines**: Zero-overhead (< 7 ns dispatch), zero per-thread stack overhead, zero dynamic heap allocations (`CoroutineStaticPool<4096>`).
3. **Flexible TLP Software Sizing & FPGA 64-Byte Padding**: Sized packets on microcontrollers/Linux (saving >90% SRAM), padded to 64 bytes at the FPGA/PCIe boundary.
4. **Cross-Platform Target Parity**: 100% shared C++20 flight logic across RP2350 Pico 2, Linux Desktop SITL, and Linux Quad-Core SBC + FPGA.
5. **Mandatory CppUTest Standard**: Every flight module and algorithmic stage MUST have dedicated CppUTest test groups validating formulas, filter responses, and edge cases.
6. **MISRA C++ Coding Standard (MISRA C++:2008 / MISRA C++:2023)**:
   * Strict 0 bytes dynamic heap allocation in the flight loop.
   * Fixed-width integer types (`<cstdint>`) with zero naked primitives.
   * Explicit `static_cast<T>()` conversions with `-Wconversion -Werror=vla` enforcement.
   * Bounds-safe `std::span` and `std::array` instead of raw pointer arithmetic.
   * Strict `const noexcept` enforcement on all state accessors.
7. **NASA/JPL "Power of 10" & DO-178C Avionics Principles**:
   * Bounded loop execution (WCET guarantee) with zero recursion.
   * Check all return values (`[[nodiscard]]`).
   * Defensive input sanitization and glitch gating across all external sensor streams.
   * Fail-safe defaults: actuators and state machines fail to disarmed / level descent.



---

## 2. 5-Phase Implementation Master Tracker

```
+───────────────────────────────────────────────────────────────────────────────────────────+
| PHASE 1: SENSOR FILTERING & PRODUCTION PID DYNAMICS (Betaflight Core)       [COMPLETE ✅] |
+───────────────────────────────────────────────────────────────────────────────────────────+
| PHASE 2: STATE ESTIMATION & AHRS (INAV Core)                                [COMPLETE ✅] |
+───────────────────────────────────────────────────────────────────────────────────────────+
| PHASE 3: FLIGHT TUNING & PRESET ENGINES (INAV Core)                         [COMPLETE ✅] |
+───────────────────────────────────────────────────────────────────────────────────────────+
| PHASE 4: AUTONOMOUS 3D NAVIGATION & FAILSAFE (INAV Core)                    [COMPLETE ✅] |
+───────────────────────────────────────────────────────────────────────────────────────────+
| PHASE 5: MULTI-RATE ENGINE INTEGRATION & DIFFERENTIAL TESTING               [COMPLETE ✅] |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 3. Phase-by-Phase Task & Module Specifications

### Phase 1: Sensor Filtering & Production PID Dynamics
* **Goal**: Establish rock-solid noise suppression and locked-in Betaflight acro rate control.
* **Upstream References**: `betaflight/src/main/flight/feedforward.c`, `anti_gravity.c`, `pid.c`, `filter.c`, `kalman.c`, `inav/src/main/flight/dterm_filter.c`.

| Module File | Component Tasks & Algorithms | Status |
| :--- | :--- | :--- |
| [`src/flight/filter.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/filter.hpp) | PT1, PT2, PT3, Biquad Direct Form II Transposed (LPF & Notch), SlewLimiter | ✅ **Complete** |
| [`src/flight/kalman.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/kalman.hpp) | 1D & 3D Gyro Kalman state-space noise filter with dynamic covariance updates | ✅ **Complete** |
| [`src/flight/pid.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/pid.hpp) | **Feedforward 2.0**: Setpoint derivative acceleration + jitter suppression factor (`ff_jitter_factor`).<br>**Anti-Gravity**: Dynamic I-term boost on throttle derivative $\frac{d(\text{throttle})}{dt}$.<br>**D-Min Dynamic D-Gain**: Scaled between $D_{\text{min}}$ (cruise) and $D_{\text{max}}$ (transients).<br>**Cascaded D-Term Filtering**: Dual PT1/PT2/Biquad filtering on $\Delta \text{gyro}$.<br>**TPA**: Throttle PID Attenuation above breakpoint.<br>**I-Term Rotation**: 3D body vector rotation $\vec{I}_k = R(\Delta \vec{\omega}) \vec{I}_{k-1}$.<br>**Angle / Horizon Mode**: Outer-loop attitude stabilization controller. | ✅ **Complete** |

---

### Phase 2: State Estimation & AHRS
* **Goal**: High-accuracy attitude determination and drift-free multi-sensor 3D position estimation.
* **Upstream References**: `inav/src/main/flight/imu.c`, `inav/src/main/navigation/navigation_pos_estimator.c`.

| Module File | Component Tasks & Algorithms | Status |
| :--- | :--- | :--- |
| [`src/flight/attitude.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/attitude.hpp) | **Mahony AHRS**: Quaternion integration, centrifugal acceleration correction ($\vec{a}_{\text{cent}} = \vec{\omega} \times \vec{v}$), gravity vector normalization, magnetic declination rotation. | ✅ **Complete** |
| [`src/flight/pos_estimator.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/pos_estimator.hpp) | **INAV Inertial Position Estimator**: 2nd-order position integration ($\Delta \vec{p} = \vec{v} \Delta t + \frac{1}{2} \vec{a} \Delta t^2$), dynamic weights ($w_z, w_{xy}$), continuous Accel Z-bias ($b_{a,z}$) & XY-bias ($b_{a,x}, b_{a,y}$) tracking, GPS glitch detection & gating. | ✅ **Complete** |

---

### Phase 3: Flight Tuning & Preset Engines
* **Goal**: Automated in-flight parameter tuning and simplified slider-based flight configuration.
* **Upstream References**: `inav/src/main/flight/pid_autotune.c`, `inav/src/main/flight/ez_tune.c`.

| Module File | Component Tasks & Algorithms | Status |
| :--- | :--- | :--- |
| [`src/flight/autotune.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/autotune.hpp) | **AutoTune Relay Engine**: Åström-Hägglund relay perturbation injection ($\pm \Delta \text{rate}$), zero-crossing peak detector, oscillation period ($T_u$) measurement, modified Ziegler-Nichols gain derivation. | ✅ **Complete** |
| [`src/flight/ez_tune.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/ez_tune.hpp) | **EZ-Tune Synthesizer**: Synthesizes $K_p, K_i, K_d, K_{ff}$ and filter cutoff frequencies from high-level Response, Damping, and Tracking parameters. | ✅ **Complete** |

---

### Phase 4: Autonomous 3D Navigation & Failsafe
* **Goal**: Full autonomous mission execution, smooth multicopter braking, and multi-stage failsafe recovery.
* **Upstream References**: `inav/src/main/navigation/navigation.c`, `navigation_multicopter.c`, `inav/src/main/flight/failsafe.c`.

| Module File | Component Tasks & Algorithms | Status |
| :--- | :--- | :--- |
| [`src/flight/navigation.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/navigation.hpp) | **3D Navigation Engine**: Multicopter S-curve deceleration profiles, velocity feedforward, full 3D RTH state machine (Climb, Turn, Cruise, Descent, Land), Safehome dynamic selection. | ✅ **Complete** |
| [`src/flight/failsafe.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/failsafe.hpp) | **Two-Stage Failsafe**: Stage 1 guard timer for transient signal loss, Stage 2 procedure execution (RTH or Auto-Land). | ✅ **Complete** |

---

### Phase 5: Multi-Rate Engine Integration & Differential Testing
* **Goal**: End-to-end multi-rate loop execution up to 16 kHz and automated differential testing against upstream logs.

| Module File | Component Tasks & Algorithms | Status |
| :--- | :--- | :--- |
| [`src/flight/flight_engine_template.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/flight_engine_template.hpp) | Multi-rate decimated task scheduler (16kHz Rate PID $\to$ 1kHz Attitude $\to$ 100Hz Baro $\to$ 10Hz GPS Nav). | ✅ **Complete** |
| [`test/test_main.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/test/test_main.cpp) | 14 Automated CppUTest / C++20 Unit Test Suites covering 100% of flight math, filters, AHRS, Nav, and Failsafe. | ✅ **Complete** |
| [`tools/run_all_validations.py`](file:///home/tcmichals/ssdData/projects/home/inav/tools/run_all_validations.py) | Master automated validation pipeline verifying build targets, 14 test suites, hardware diagnostics, and mathematical parity. | ✅ **Complete** |

---

## 4. Verification & Validation Commands

```bash
# 1. Build Host Executables & Run All Unit Test Suites:
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
./build/run_unit_tests -v -c

# 2. Build Bare-Metal RP2350 Pico 2 W Firmware:
cmake -B build_pico2w -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release
cmake --build build_pico2w -j$(nproc)

# 3. Run Automated Python Master Validation Pipeline:
python3 tools/run_all_validations.py
```

