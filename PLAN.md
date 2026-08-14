# Project Action Plan & Roadmap (`PLAN.md`)

> **Master References**:
> - [INAV Master Porting Plan](docs/INAV_PORTING_MASTER_PLAN.md)
> - [Pico 2 RP2350 Hardware Architecture](docs/TARGET_PICO2.md)
> - [Validation Runner](tools/run_all_validations.py)

---

## 🎯 Current Sprint Objectives

- [x] **Git Submodule Parity Setup**: Integrated `external/inav` & `external/betaflight`.
- [x] **Submodule Differential Test Suite**: Native C++ test runner (`submodule_differential_test`) verifying 7 core math/protocol suites.
- [x] **Flight Engine Decoupling**: Replaced prototype `Ekf3Filter` with complete INAV pipeline (`InavImu`, `InavPosEstimator`, `GyroSpectralAnalyzer`, `DynamicGyroNotchBank`, `PidController`, `NavigationEngine`, `Mixer`).
- [x] **Dynamic Gyro Notch Spectral Analyzer (`src/flight/gyro_analyse.hpp`)**: 64-point FFT with Hanning windowing & parabolic interpolation.
- [x] **Dynamic Gyro LPF (`src/flight/dynamic_lpf.hpp`)**: Throttle-dependent expo cutoff curve.
- [x] **EZ-Tune Macro Engine (`src/flight/ez_tune.hpp`)**: Ported `ezTuneUpdate()` and parameter synthesis.
- [x] **Task vs Coroutine Benchmarking Suite (`test/test_scheduler_benchmark.cpp`)**: Verified 0.36us jitter and 100% bit-exact control outputs.
- [x] **Full-Stack 60s Flight Mission Parity Suite (`test/test_full_stack_parity.cpp`)**: 60,000 continuous flight ticks across all 5 flight phases with 0.000000 attitude & position error.
- [x] **DShot RPM Notch Filter Bank (`src/flight/rpm_filter.hpp`)**: Motor harmonic acoustic noise tracking.
- [ ] **Matrix Smith Predictor (`src/flight/smith_predictor.hpp`)**: Gyro filter phase lead compensation.
- [ ] **Native CRSF / ELRS Protocol Decoder (`src/drivers/rc/crsf.hpp`)**: Bidirectional telemetry & link quality frames.

---

## 📋 Active Task Backlog

### 1. Flight Dynamics & Sensor Filters
- [x] **1.1 IMU / AHRS** (`src/flight/attitude.hpp` $\leftrightarrow$ `external/inav/src/main/flight/imu.c`)
  - Gaussian nearness weighting, dynamic rate ignore, centrifugal compensation, EF horizontal mag projection, decidegrees.
- [x] **1.2 Gyro Spectral Dynamic Notch** (`src/flight/gyro_analyse.hpp` $\leftrightarrow$ `external/inav/src/main/flight/gyroanalyse.c`)
  - 4-step state machine, 64-point FFT, Hanning window, parabolic sub-bin peak interpolation, 25Hz PT1 smoothing.
- [x] **1.3 Dynamic Gyro LPF** (`src/flight/dynamic_lpf.hpp` $\leftrightarrow$ `external/inav/src/main/flight/dynamic_lpf.c`)
  - Throttle-dependent expo cutoff curve & `dynamicLpfGyroTask`.
- [x] **1.4 EZ-Tune Macro Engine** (`src/flight/ez_tune.hpp` $\leftrightarrow$ `external/inav/src/main/flight/ez_tune.c`)
  - 10-parameter synthesis, yaw scale, pitch ratio, Smith delay, Kalman Q.
- [x] **1.5 DShot RPM Filter** (`src/flight/rpm_filter.hpp` $\leftrightarrow$ `external/inav/src/main/flight/rpm_filter.c`)
  - Multi-harmonic biquad notch filter bank driven by DShot RPM telemetry.
- [ ] **1.6 Matrix Smith Predictor** (`src/flight/smith_predictor.hpp` $\leftrightarrow$ `external/inav/src/main/flight/smith_predictor.c`)
  - Phase lead compensation for low-pass filter group delay.


### 2. PID & Control Dynamics
- [x] **2.1 Betaflight Feedforward 2.0 & Anti-Gravity** (`src/flight/pid.hpp`)
  - Derivative acceleration, jitter filter, throttle punchout I-term boost, D-Min dynamic damping.
- [x] **2.2 INAV 3D Body-Frame I-Term Rotation & TPA** (`src/flight/pid.hpp`)
  - $\vec{I}_{\text{new}} = \vec{I} + \vec{\omega} \times \vec{I}$, throttle attenuation curve.
- [ ] **2.3 D-Boost & I-Term Relax** (`src/flight/pid.hpp`)
  - Transient stick acceleration D-boost and setpoint rotation iterm relaxation.

### 3. State Estimation & Navigation
- [x] **3.1 3D Inertial Position Estimator** (`src/flight/pos_estimator.hpp` $\leftrightarrow$ `navigation_pos_estimator.c`)
  - Accel + Baro + GPS innovation fusion, Z-bias tracking, glitch gating.
- [x] **3.2 3D Autonomous Guidance & S-Curve Kinematics** (`src/flight/navigation.hpp`)
  - RTH state machine, S-curve braking $v = \min(v_{\text{max}}, \sqrt{2 a d})$, Safehome selection.
- [ ] **3.3 Wind & RTH Energy Estimators** (`src/flight/wind_estimator.hpp`)
  - Groundspeed vs airspeed vector drift calculation.

### 4. Protocols & Hardware Targets
- [x] **4.1 MSP v1/v2 Protocol Handshake** (`src/msp/msp_protocol.cpp`)
  - Full handshake suite + `MSP2_COMMON_GET_EZ_TUNE` (0x2405) / `MSP2_COMMON_SET_EZ_TUNE` (0x2406).
- [ ] **4.2 Native CRSF / ExpressLRS Receiver** (`src/drivers/rc/crsf.hpp`)
  - 16-channel RC packets + link statistics telemetry frames.
- [x] **4.3 RP2350 Bare-Metal Hardware Harness** (`src/target/pico2_rp2350/`)
  - PIO DShot, SPI IMU, I2C Baro, UART GPS/MSP.
- [x] **4.4 6-DOF SITL Python Simulator & 3D EFIS Dashboard** (`tools/`)

---

## ⚡ Quick Validation Commands

```bash
# 1. Run Complete 7-Step System Validation Pipeline
python3 tools/run_all_validations.py

# 2. Run Native Submodule Differential Test Runner
cd build && ./submodule_differential_test

# 3. Run Comprehensive 14-Suite Unit Tests
cd build && ./run_unit_tests

# 4. Launch 3D Ground Station Dashboard (with autonomous flight demo)
python3 tools/gui_fc_bench_dashboard.py --demo
```
