# Three-Way Feature & Algorithmic Parity Audit Matrix
## Upstream INAV vs. Upstream Betaflight vs. `inav-abstractx`

> [!IMPORTANT]
> **EXHAUSTIVE ALGORITHMIC PARITY AUDIT**
> This matrix provides a file-by-file and algorithm-by-algorithm gap analysis cross-referencing:
> 1. **Upstream INAV**: `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/`
> 2. **Upstream Betaflight**: `/home/tcmichals/ssdData/projects/home/flightcode/betaflight/src/main/`
> 3. **Current C++20 Codebase**: `/home/tcmichals/ssdData/projects/home/inav/`

---

## 1. Flight Dynamics & PID Control

| Feature / Module | Upstream Reference | Algorithms & Mathematical Requirements | Current Implementation | Parity Status |
| :--- | :--- | :--- | :--- | :--- |
| **PID Controller** | INAV `pid.c` / BF `pid.c` | Cascaded rate PID, TPA gain scaling, anti-windup clamping, angle mode outer loop | [`src/flight/pid.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/pid.hpp) | ⚠️ **Partial** (Lacks TPA, I-term rotation) |
| **Feedforward 2.0** | BF `feedforward.c` | Setpoint derivative acceleration $\frac{d(\text{setpoint})}{dt}$, jitter attenuation, transition smoothing | Missing | ❌ **Need Port** |
| **Anti-Gravity** | BF `anti_gravity.c` | Dynamic I-term gain boost driven by throttle derivative $\frac{d(\text{throttle})}{dt}$ | Missing | ❌ **Need Port** |
| **D-Min Dynamic D** | BF `pid.c` | Dynamic D-gain scaling between $D_{\text{min}}$ (cruise) and $D_{\text{max}}$ (transients) | Missing | ❌ **Need Port** |
| **VBat Compensation**| BF `pid.c` | Master PID multiplier scaling inversely with battery sag: $\frac{V_{\text{nom}}}{V_{\text{meas}}}$ | Missing | ❌ **Need Port** |
| **AutoTune Engine** | INAV `pid_autotune.c` | Åström-Hägglund relay limit-cycle oscillation, peak detector, Ziegler-Nichols tuning | Missing | ❌ **Need Port** |
| **EZ-Tune Synthesizer**| INAV `ez_tune.c` | High-level Response/Damping/Tracking synthesis into PID gains & filter cutoffs | Missing | ❌ **Need Port** |

---

## 2. Sensor Filtering & Noise Rejection Pipeline

| Filter Stage | Upstream Reference | Algorithms & Mathematical Requirements | Current Implementation | Parity Status |
| :--- | :--- | :--- | :--- | :--- |
| **Cascaded Low-Pass**| INAV/BF `filter.c` | 1st/2nd/3rd order PT1, PT2, PT3, and Biquad Direct Form II Transposed | [`src/flight/filter.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/filter.hpp) | ✅ **100% Complete** |
| **Gyro Kalman Filter**| INAV/BF `kalman.c` | 1st & 2nd order dynamic state-space covariance noise filter | [`src/flight/kalman.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/kalman.hpp) | ✅ **100% Complete** |
| **RPM Harmonic Notch**| INAV/BF `rpm_filter.c` | 1st, 2nd, 3rd harmonic notch filters per motor via DShot telemetry | Missing | ❌ **Need Port** |
| **Dynamic Peak Notch**| BF `dyn_notch.c` | Peak tracking notch filters adapting to instantaneous frame resonances | Missing | ❌ **Need Port** |
| **D-Term Low-Pass** | INAV `dterm_filter.c` | Dual cascaded low-pass filtering on gyro rate derivative | Missing (In progress) | ⚠️ **Partial** |

---

## 3. State Estimation & Attitude (AHRS)

| Subsystem | Upstream Reference | Algorithms & Mathematical Requirements | Current Implementation | Parity Status |
| :--- | :--- | :--- | :--- | :--- |
| **Attitude Filter (AHRS)**| INAV `imu.c` | Mahony/Madgwick quaternion filter, centrifugal compensation, gravity normalization | [`src/flight/attitude.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/attitude.hpp) | ⚠️ **Partial** (Basic Euler integration) |
| **Position Estimator**| INAV `navigation_pos_estimator.c` | Multi-sensor Accel+Baro+GPS complementary & inertial fusion, glitch recovery | [`src/flight/ekf3.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/ekf3.hpp) | ⚠️ **Partial** (Fixed-gain baseline) |
| **Sensor Calibration**| INAV `acceleration.c` / `compass.c` | 6-point sphere-fitting accelerometer calibration, hard/soft iron mag correction | Missing | ❌ **Need Port** |

---

## 4. Autonomous Navigation & Failsafe

| Subsystem | Upstream Reference | Algorithms & Mathematical Requirements | Current Implementation | Parity Status |
| :--- | :--- | :--- | :--- | :--- |
| **3D Navigation Engine**| INAV `navigation.c` | RTH climb/turn/cruise/land state machine, Position Hold, Safehome selection | [`src/flight/navigation.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/navigation.hpp) | ⚠️ **Partial** (Basic RTH) |
| **Fixed-Wing TECS** | INAV `navigation_fixedwing.c` | Total Energy Control System (pitch/throttle energy balance) | Missing | ❌ **Need Port** |
| **Multicopter Nav** | INAV `navigation_multicopter.c`| Deceleration braking curves, velocity feedforward, jerk limiting | Missing | ❌ **Need Port** |
| **Failsafe System** | INAV `failsafe.c` | Stage 1 guard timers, Stage 2 RTH/Emergency Landing procedure | [`src/flight/failsafe.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/failsafe.hpp) | ⚠️ **Partial** (Simple guard) |

---

## 5. Hardware Abstraction & Infrastructure

| Subsystem | Upstream Architecture | `inav-abstractx` Architecture | Parity Status |
| :--- | :--- | :--- | :--- |
| **Bus Abstraction** | Monolithic HAL & direct register writes | **64-Byte TLP Virtual Bus & SPSC Rings** ([`asp_tlp64.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/asp_tlp64.hpp)) | ✅ **100% Complete** |
| **Hardware Offloading**| Software IRQ bit-banging | **RP2350 Triple-PIO State Machines** (DShot, CRSF, Auto-SPI IMU) | ✅ **100% Complete** |
| **Platform Targets** | Separate fork builds | **SITL, RP2350 Pico 2 Bare-Metal, Linux SBC** unified targets | ✅ **100% Complete** |
| **Configuration** | Linker section `.pg_registry` | **Flat POD Registry & Flash Sector `0x1F0000`** | ✅ **100% Complete** |
| **Blackbox Tracing** | CPU bit-packing to SD Card | **Zero-CPU BareCTF 64B TLP Stream** | ✅ **100% Complete** |
