# INAV & Betaflight C++20 Master Porting & Parity Plan

> **Official Submodule References**:
> - Upstream INAV: [`external/inav/src/main/`](../external/inav/src/main/)
> - Upstream Betaflight: [`external/betaflight/src/main/`](../external/betaflight/src/main/)
> - Validation Suite: `python3 tools/run_all_validations.py`

This document is the persistent, step-by-step master checklist and architectural reference for porting all upstream INAV and Betaflight flight control, sensor fusion, filtering, navigation, and communication systems to high-performance C++20 with 100% bit-exact mathematical parity.

---

## 1. Subsystem Mapping & Progress Tracker

| # | Subsystem | Upstream Reference C Source | C++20 Target Header | Status | Differential Unit Test |
|---|:---|:---|:---|:---:|:---|
| **1.1** | **IMU / AHRS State Estimation** | `external/inav/src/main/flight/imu.c`<br>`external/inav/src/main/flight/imu.h` | [`src/flight/attitude.hpp`](../src/flight/attitude.hpp) | `COMPLETE` | `DIFF 3/7` in `submodule_differential_test` |
| **1.2** | **Dynamic Gyro Notch / FFT** | `external/inav/src/main/flight/gyroanalyse.c`<br>`external/inav/src/main/flight/gyroanalyse.h` | [`src/flight/gyro_analyse.hpp`](../src/flight/gyro_analyse.hpp) | `COMPLETE` | `DIFF 4/7` in `submodule_differential_test` |
| **1.3** | **Dynamic Gyro LPF** | `external/inav/src/main/flight/dynamic_lpf.c`<br>`external/inav/src/main/flight/dynamic_lpf.h` | [`src/flight/dynamic_lpf.hpp`](../src/flight/dynamic_lpf.hpp) | `COMPLETE` | `DIFF 1/7` in `submodule_differential_test` |
| **1.4** | **EZ-Tune Macro Engine** | `external/inav/src/main/flight/ez_tune.c`<br>`external/inav/src/main/flight/ez_tune.h` | [`src/flight/ez_tune.hpp`](../src/flight/ez_tune.hpp) | `COMPLETE` | `DIFF 2/7` in `submodule_differential_test` |
| **1.5** | **Rate PID & Acro Dynamics** | `external/betaflight/src/main/flight/pid.c`<br>`external/inav/src/main/flight/pid.c` | [`src/flight/pid.hpp`](../src/flight/pid.hpp) | `COMPLETE` | `DIFF 5/7` in `submodule_differential_test` |
| **1.6** | **3D Inertial Pos Estimator** | `external/inav/src/main/navigation/navigation_pos_estimator.c` | [`src/flight/pos_estimator.hpp`](../src/flight/pos_estimator.hpp) | `COMPLETE` | `TEST 12/14` in `run_unit_tests` |
| **1.7** | **3D Navigation & S-Curve** | `external/inav/src/main/navigation/navigation.c`<br>`external/inav/src/main/navigation/sqrt_controller.c` | [`src/flight/navigation.hpp`](../src/flight/navigation.hpp) | `COMPLETE` | `DIFF 6/7` in `submodule_differential_test` |
| **1.8** | **AutoTune Relay Feedback** | `external/inav/src/main/flight/pid_autotune.c` | [`src/flight/autotune.hpp`](../src/flight/autotune.hpp) | `COMPLETE` | `TEST 13/14` in `run_unit_tests` |
| **1.9** | **Airframe Motor/Servo Mixer** | `external/inav/src/main/flight/mixer.c`<br>`external/betaflight/src/main/flight/mixer.c` | [`src/flight/mixer.hpp`](../src/flight/mixer.hpp) | `COMPLETE` | `TEST 8/14` in `run_unit_tests` |
| **1.10** | **2-Stage Failsafe Engine** | `external/inav/src/main/flight/failsafe.c` | [`src/flight/failsafe.hpp`](../src/flight/failsafe.hpp) | `COMPLETE` | `TEST 14/14` in `run_unit_tests` |
| **1.11** | **MSP v1/v2 Serialization** | `external/inav/src/main/io/msp.c`<br>`external/inav/src/main/msp/msp_protocol.h` | [`src/msp/msp_protocol.cpp`](../src/msp/msp_protocol.cpp) | `COMPLETE` | `DIFF 7/7` in `submodule_differential_test` |
| **1.12** | **DShot RPM Notch Filtering** | `external/inav/src/main/flight/rpm_filter.c` | [`src/flight/rpm_filter.hpp`](../src/flight/rpm_filter.hpp) | `COMPLETE` | `DIFF 8/8` in `submodule_differential_test` |
| **1.13** | **Matrix Smith Predictor** | `external/inav/src/main/flight/smith_predictor.c` | [`src/flight/smith_predictor.hpp`](../src/flight/smith_predictor.hpp) | `COMPLETE` | `TEST 16/18` in `run_unit_tests` |
| **1.14** | **Wind & RTH Estimators** | `external/inav/src/main/flight/wind_estimator.c`<br>`external/inav/src/main/flight/rth_estimator.c` | [`src/flight/wind_estimator.hpp`](../src/flight/wind_estimator.hpp) | `COMPLETE` | `TEST 17/19` in `run_unit_tests` |
| **1.15** | **Sensor Alignment & 3D Rotation** | `external/inav/src/main/sensors/boardalignment.c` | [`src/sensors/sensor_alignment.hpp`](../src/sensors/sensor_alignment.hpp) | `COMPLETE` | `TEST 19/19` in `run_unit_tests` |
| **1.16** | **Sensor Calibration & Temp Drift** | `external/inav/src/main/sensors/gyro.c`<br>`external/inav/src/main/sensors/compass.c` | [`src/sensors/sensor_calibration.hpp`](../src/sensors/sensor_calibration.hpp) | `COMPLETE` | `TEST 19/19` in `run_unit_tests` |
| **1.17** | **MS4525DO Digital Pitot Airspeed** | `external/inav/src/main/sensors/pitotmeter.c`<br>`external/inav/src/main/drivers/pitotmeter/pitotmeter_ms4525.c` | [`src/drivers/pitot/ms4525do.hpp`](../src/drivers/pitot/ms4525do.hpp) | `COMPLETE` | `TEST 19/19` in `run_unit_tests` |
| **1.18** | **Battery ADC & Energy Monitor** | `external/inav/src/main/sensors/battery.c`<br>`external/betaflight/src/main/sensors/battery.c` | [`src/sensors/battery_monitor.hpp`](../src/sensors/battery_monitor.hpp) | `COMPLETE` | `TEST 19/19` in `run_unit_tests` |
| **1.19** | **Sensor Auto-Detection & Displays** | `external/inav/src/main/drivers/display/` | [`src/drivers/sensor_detector.hpp`](../src/drivers/sensor_detector.hpp)<br>[`oled_ssd1306.hpp`](../src/drivers/display/oled_ssd1306.hpp)<br>[`osd_max7456.hpp`](../src/drivers/display/osd_max7456.hpp) | `COMPLETE` | `TEST 18/19` in `run_unit_tests` |
| **1.20** | **Cubie A5E Linux SBC Transport** | `external/inav/src/main/target/` | [`src/target/linux_common/linux_fpga_transport.hpp`](../src/target/linux_common/linux_fpga_transport.hpp) | `COMPLETE` | `TEST 19/19` in `run_unit_tests` |
| **1.21** | **ESP32-P4 MIPI-CSI Camera Optical Flow** | `external/inav/src/main/sensors/opflow.c` | [`src/sensors/optical_flow.hpp`](../src/sensors/optical_flow.hpp) | `PLANNED` | Visual-inertial velocity tracking ($\Delta x, \Delta y$ @ 120 fps) |
| **1.22** | **Tang Nano 9K FPGA 64B TLP Bridge** | `external/inav/src/main/target/` | [`src/target/linux_common/tang9k_fpga_transport.hpp`](../src/target/linux_common/tang9k_fpga_transport.hpp) | `PLANNED` | Gowin GW1NR-9 64-byte parallel TLP bus bridge |

---

## 2. Step-by-Step Task Breakdown

### Phase 1: Core Flight & Dynamics (Current Focus)
- [x] **Audit & replace top-level `Ekf3Filter` with true INAV pipeline** (`src/flight/flight_engine_template.hpp`).
- [x] **Add official git submodules** (`external/inav` & `external/betaflight`).
- [x] **Implement Native C++ Submodule Differential Test Target** (`test/test_submodule_differential.cpp`).
- [x] **Port 11-Stage INAV AHRS IMU** (`src/flight/attitude.hpp`):
  * Gaussian Accel Nearness Weighting ($e^{-(a-1.0)^2 / (2 \cdot 0.20^2)}$).
  * Dynamic Angular Rate Invalidation (`acc_ignore_rate`, `acc_ignore_slope`).
  * 3Hz PT1 cascades on Gyro BF, Accel BF, Heading EF, and GPS speed.
  * Centrifugal Acceleration Correction ($\vec{a}_{\text{cent}} = \vec{v}_{\text{gps}} \times \vec{\omega}$).
  * Earth-Frame Horizontal Magnetometer Projection ($v_z = 0$).
  * GPS COG & Acceleration Yaw Fusion with multirotor tilt angle attenuation.
  * Sinc Delta Quaternion Integration ($\Delta q = [\cos |\theta|, \frac{\sin |\theta|}{|\theta|} \theta]$).
  * Decidegree Euler Angles and Small Angle Arming Check (`r_mat_[2][2] > cos(25 deg)`).
- [x] **Port 4-Stage FFT Dynamic Gyro Notch Analyzer** (`src/flight/gyro_analyse.hpp`):
  * 64-sample circular buffer with $2\times$ downsampling.
  * 4-Stage State Machine (`StepWindowing`, `StepRfft`, `StepMagnitudeAndPeaks`, `StepUpdateFilters`).
  * Hanning windowing ($0.5 - 0.5 \cos(2\pi i / 63)$).
  * Parabolic sub-bin interpolation (`computeParabolaMean`: $\frac{y_0 - y_2}{2(y_0 - 2y_1 + y_2)}$).
  * 25Hz PT1 frequency smoothing filter.
  * 1:1 C symbol wrappers: `gyroDataAnalysePush`, `gyroDataAnalyse`, `gyroDataAnalyseStateInit`.
- [x] **Port Dynamic Gyro LPF** (`src/flight/dynamic_lpf.hpp`):
  * Throttle-dependent expo cutoff curve `curve = throttle * (1 - throttle) * (expo/10) + throttle`.
  * 1:1 C symbol wrapper: `dynamicLpfGyroTask(throttle_us)`.
- [x] **Port EZ-Tune Macro Preset Engine** (`src/flight/ez_tune.hpp`):
  * `ezTuneSettings_t` struct with 10 exact parameters.
  * `ezTuneUpdate()` formulas: D-term LPF cutoff, Smith predictor delay, Kalman Q scaling, Roll/Pitch/Yaw gain matrix, rates, and expo.
  * Wired into `FlightEngineTemplate` startup and MSP `Cmd::EzTuneSet` (0x2406).
- [x] **Port DShot RPM Filter Engine** (`src/flight/rpm_filter.hpp`):
  * Frequency tracking per motor RPM harmonic ($f = \frac{\text{RPM}}{60} \times h$).
  * Cascaded biquad notch filters per motor harmonic on Roll, Pitch, and Yaw with 150Hz PT1 smoothing.
  * Validated bit-exact in submodule differential test suite `[DIFF 8/8]`.
- [x] **Port Matrix Smith Predictor** (`src/flight/smith_predictor.hpp`):
  * Phase lead compensation for low-pass filter group delay.
- [x] **Port Wind & RTH Estimator** (`src/flight/wind_estimator.hpp`):
  * Earth-frame horizontal wind velocity estimation from airspeed / groundspeed delta.

---

### Phase 2: Protocols, Handshakes & Ground Station Support
- [x] **MSP v1 & MSP v2 Handshake Processing** (`src/msp/msp_protocol.cpp`):
  * Support `MSP_FC_VARIANT`, `MSP_API_VERSION`, `MSP_BOARD_INFO`, `MSP_RAW_IMU`, `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_RC`, `MSP_PID`, `MSP2_COMMON_GET_EZ_TUNE`, `MSP2_COMMON_SET_EZ_TUNE`.
- [x] **CRSF / ELRS Native Protocol Decoder** (`src/drivers/rc/crsf.hpp`):
  * Direct bidirectional CRSF telemetry frames with link stats.

---

### Phase 3: Hardware Diagnostics & SITL Emulation
- [x] **Live 6-DOF SITL Simulation** (`src/target/sitl/sitl_main.cpp`).
- [x] **3D Hardware Dashboard & EFIS PFD** (`tools/gui_fc_bench_dashboard.py`).
- [x] **Decoupled Top-Level TLP Drivers & Bottom-Half PCIe Scheduler** (`src/drivers/bus/tlp_channel.hpp`, `src/target/common/pcie_tlp_scheduler.hpp`).
- [x] **Pure Asynchronous Driver Standard**:
  * Strict elimination of legacy synchronous `init()` wrappers and blocking `delay_ms()` calls across all sensor drivers.
  * All chip drivers operate 100% as non-blocking C++20 coroutines (`async_init()`, `sample_loop()`).
  * Bottom-half layer handles asynchronous PCIe TLP packet dispatching/reception (`Tlp64` / `TlpChannel`) with zero thread blocking.

---

### Phase 4: Heterogeneous Hardware Target Pipeline & TLP Sizing

- [x] **RP2350 (Pico 2 / Pico 2 W) Real-Time Actuator Node**:
  * Dual ARM Cortex-M33 / RISC-V Hazard3 @ 150 MHz with 520 KB SRAM.
  * Triple-PIO state machines executing jitter-free DShot600, SPI IMU DMA, and CRSF framing.
  * Standalone failsafe level descent engine.
- [ ] **ESP32-P4-WIFI6 (Kit A / Slim) Vision & Wireless Node**:
  * Dual-core RISC-V HP CPU @ 400 MHz + Single-core RISC-V LP CPU @ 40 MHz with 768 KB L2 SRAM + 32 MB PSRAM.
  * 2-lane MIPI-CSI camera interface with hardware ISP processing 120 fps optical flow velocity vectors ($\Delta x, \Delta y$).
  * Wi-Fi 6 (802.11ax) companion transport delivering >50 Mbps sustained throughput for 1 kHz binary CTF Blackbox streaming and MSP Configurator over TCP 5760.
- [ ] **Cubie A5E SBC + Tang Nano 9K FPGA Node**:
  * Allwinner A523 Octa-Core ARM Cortex-A55 Linux `PREEMPT_RT` mission computer.
  * Gowin GW1NR-9 FPGA (8640 LUT4, 26 Block RAMs) 64-byte TLP hardware routing bridge.
- [x] **Sized Software TLP vs. 64-Byte FPGA Bus Protocol**:
  * Variable-length compact TLP payloads over SPSC rings on microcontrollers (RP2350, ESP32-P4) to conserve internal SRAM footprint.
  * Automatic 64-byte (512-bit) alignment padding ([TlpWire64](../include/asp_tlp64.hpp)) crossing into Tang 9K FPGA / DMA hardware boundaries for single-cycle DMA ingestion.

---

## 3. How to Run Validations

### Run Master 7-Step Automated Pipeline
```bash
python3 tools/run_all_validations.py
```


### Run Native Submodule Differential Test Suite
```bash
cd build && ./submodule_differential_test
```

### Run Python Differential Math Parity Test
```bash
python3 tools/compare_inav_parity.py
```

### Run Comprehensive 20-Suite Unit Tests
```bash
cd build && ./run_unit_tests
```

### Run Linux Board Hardware Test Suite
```bash
python3 tools/test_board_hardware.py --device sitl
```

