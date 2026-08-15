# Project Action Plan & Roadmap (`PLAN.md`)

> **Master References**:
> - [INAV Master Porting Plan](docs/INAV_PORTING_MASTER_PLAN.md)
> - [Pico 2 RP2350 Hardware Architecture](docs/TARGET_PICO2.md)
> - [Hardware Driver Spec](docs/HARDWARE_DRIVERS.md)
> - [Validation Runner](tools/run_all_validations.py)

---

## 🎯 Architectural Model: Decoupled Top-Level Drivers & Bottom-Half PCIe TLP Scheduler

The system is strictly divided into **Top-Level Coroutine Drivers** and a **Bottom-Half PCIe TLP Protocol Processor & Scheduler** communicating over lock-free SPSC rings.

### ⚠️ Pure Asynchronous Architecture Mandate:
1. **Zero Synchronous / Blocking Methods**: All chip drivers MUST be 100% asynchronous C++20 coroutines (`async_init()`, `sample_loop()`). Legacy synchronous `init()` wrappers and blocking delays (`delay_ms()`) are strictly eliminated.
2. **Bottom-Half TLP Dispatching**: The bottom half handles hardware DMA, interrupts, and dispatches 64-byte PCIe TLP packets (`Tlp64`) across `TlpChannel` and `SpscTlpRing`.
3. **Top-Half Pure Transform**: The top half consumes completed TLP frames and parses them (`parse_tlp`) into engineering units without blocking or hardware coupling.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
│ TOP LEVEL: Chip Drivers & Flight Core (C++20 Coroutines)                                  │
│                                                                                           │
│  - Drivers (ICM-42688P, BMP280, QMC5883L, MS4525DO, GPS, CRSF, etc.)                     │
│  - Generates Outbound Tlp64 requests (MemRead, MemWrite, Config, DmaStream) via TlpChannel│
│  - Calls co_await tlp_channel.async_transaction(req) or co_await sleep_ms(sensor_delay)  │
│  - Consumes Inbound Tlp64 completions and parses raw payloads into float units (<0.1 µs)  │
│  - ZERO knowledge of physical SPI pins, I2C controllers, or hardware registers           │
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
                                              │
                   [Outbound TLP SPSC Ring]   │   [Inbound TLP SPSC Ring]
                   (Fixed 64B Tlp64 Requests) │   (Fixed 64B Tlp64 Completions)
                                              │
+─────────────────────────────────────────────▼─────────────────────────────────────────────+
│ BOTTOM HALF: PCIe TLP Protocol Processor & Hardware I/O Scheduler                         │
│                                                                                           │
│  - Pops Outbound Tlp64 requests from the queue                                            │
│  - Decodes Virtual BAR Address (bar::ImuBase, bar::BaroBase, bar::MagBase, etc.)          │
│  - Dispatches & Schedules transactions to the appropriate hardware backend:               │
│      ├── FPGA PCIe / AXI DMA Mailbox (Hardware offload)                                   │
│      ├── RP2350 PIO State Machines (PioImuReader, DShot PIO, CRSF PIO)                    │
│      ├── Native Hardware DMA SPI / I2C / UART Master                                      │
│      └── SITL Simulator / Test Byte Injector                                              │
│  - Packages hardware results + 64-bit nanosecond timestamp into Inbound Tlp64 completion  │
│  - Pushes Inbound Tlp64 into ring and wakes up the pending Top-Level coroutine            │
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 🎯 Current Sprint Objectives

- [x] **Git Submodule Parity Setup**: Integrated `external/inav` & `external/betaflight`.
- [x] **Submodule Differential Test Suite**: Native C++ test runner (`submodule_differential_test`) verifying 8 core math/protocol suites (100% bit-exact).
- [x] **Flight Engine Decoupling**: Complete INAV pipeline (`InavImu`, `InavPosEstimator`, `GyroSpectralAnalyzer`, `DynamicGyroNotchBank`, `PidController`, `NavigationEngine`, `Mixer`).
- [x] **Dynamic Gyro Notch Spectral Analyzer** (`src/flight/gyro_analyse.hpp`): 64-point FFT, Hanning windowing, parabolic interpolation.
- [x] **Dynamic Gyro LPF** (`src/flight/dynamic_lpf.hpp`): Throttle-dependent expo cutoff curve.
- [x] **EZ-Tune Macro Engine** (`src/flight/ez_tune.hpp`): Ported `ezTuneUpdate()` and parameter synthesis.
- [x] **Task vs Coroutine Benchmarking Suite**: 0.36 µs jitter and 100% bit-exact control outputs over 100,000 cycles.
- [x] **Full-Stack 60s Flight Mission Parity Suite**: 60,000 continuous flight ticks across 5 flight phases with 0.000000 attitude & position error.
- [x] **DShot RPM Notch Filter Bank** (`src/flight/rpm_filter.hpp`): Motor harmonic acoustic noise tracking.
- [x] **Bus Concept Layer** (`src/drivers/bus/bus_concepts.hpp`): `IsSpiBus` / `IsI2cBus` C++20 concepts with `Pico2SpiBus`, `Pico2I2cBus`, `FakeSpiBus`, `FakeI2cBus`.
- [x] **Full IMU Drivers (Bottom-Half + Top-Half)**:
  - `ICM-42688P` (`src/drivers/imu/icm42688p.hpp`): WHO_AM_I, soft reset, Bank 0..2 AAF filters, INT1 DRDY, 14B burst.
  - `BMI088` (`src/drivers/imu/bmi088.hpp`): Dual-die SPI, Accel & Gyro soft resets, ±24g / ±2000 dps config, 14B burst.
  - `MPU-6000/6500` (`src/drivers/imu/mpu6000.hpp`): WHO_AM_I, PLL Z-gyro clock, ±16g / ±2000 dps, DLPF, 14B burst.
- [x] **Full Barometer Drivers (Bottom-Half + Top-Half)**:
  - `BMP280` (`src/drivers/baro/bmp280.hpp`): WHO_AM_I, 24B calib readout, forced mode, exact Bosch 64-bit integer compensation.
  - `DPS310` (`src/drivers/baro/dps310.hpp`): WHO_AM_I, 18B calib readout, 16x continuous mode, exact Infineon 2nd-order polynomial compensation.
  - `MS5611` (`src/drivers/baro/ms5611.hpp`): 8-word PROM, CRC4 verify, 2-phase non-blocking D1/D2 conversion, exact MEAS 64-bit math.
- [x] **Full Magnetometer Drivers (Bottom-Half + Top-Half)**:
  - `QMC5883L` (`src/drivers/mag/qmc5883l.hpp`): WHO_AM_I, SET/RESET period, 200 Hz continuous mode, 6B burst, 3000 LSB/Gauss.
  - `IST8310` (`src/drivers/mag/ist8310.hpp`): WHO_AM_I, 16x average, pulse duration config, 200 Hz continuous mode, 0.3 µT/LSB.
- [x] **POSIX Zero-Dependency TCP Transport** (`src/msp/posix_tcp_transport.hpp`): Native POSIX socket transport for Linux SITL / SBC without requiring external Boost packages.
- [x] **SensorConfig in MasterConfig**: Flash-persisted chip selection, SPI/I2C pin assignments, ODR/FS per sensor.
- [x] **RP2350 Pico 2 W Target Harness** (`src/target/pico2_rp2350/`): Peripheral init and dual-core flight engine boot sequence.

---

## 📋 Active Task Backlog

### 1. Flight Dynamics & Sensor Filters
- [x] **1.1 IMU / AHRS** (`src/flight/attitude.hpp`)
- [x] **1.2 Gyro Spectral Dynamic Notch** (`src/flight/gyro_analyse.hpp`)
- [x] **1.3 Dynamic Gyro LPF** (`src/flight/dynamic_lpf.hpp`)
- [x] **1.4 EZ-Tune Macro Engine** (`src/flight/ez_tune.hpp`)
- [x] **1.5 DShot RPM Filter** (`src/flight/rpm_filter.hpp`)
- [x] **1.6 Matrix Smith Predictor** (`src/flight/smith_predictor.hpp`)
  - Phase lead compensation for low-pass filter group delay.

### 2. PID & Control Dynamics
- [x] **2.1 Betaflight Feedforward 2.0 & Anti-Gravity** (`src/flight/pid.hpp`)
- [x] **2.2 INAV 3D Body-Frame I-Term Rotation & TPA** (`src/flight/pid.hpp`)
- [x] **2.3 D-Boost & I-Term Relax** (`src/flight/pid.hpp`)
  - Transient stick acceleration D-boost and setpoint rotation iterm relaxation.

### 3. State Estimation & Navigation
- [x] **3.1 3D Inertial Position Estimator** (`src/flight/pos_estimator.hpp`)
- [x] **3.2 3D Autonomous Guidance & S-Curve Kinematics** (`src/flight/navigation.hpp`)
- [x] **3.3 Wind & RTH Energy Estimators** (`src/flight/wind_estimator.hpp`)
  - 2D horizontal wind vector estimation and return-to-home battery horizon calculation.

### 4. Protocols & Actuator Drivers
- [x] **4.1 MSP v1/v2 Protocol Handshake** (`src/msp/msp_protocol.cpp`)
- [x] **4.2 DShot Digital Actuator Driver** (`src/drivers/esc/dshot.hpp`, `src/drivers/esc_dshot_driver.hpp`)
### 5. Decoupled TLP Drivers, CppUTest Suites & Hardware Board Test Harnesses
- [x] **5.1 Pure TLP Top-Level Drivers**:
  - Implemented `Icm42688pTlpDriver`, `Bmp280TlpDriver`, `Qmc5883lTlpDriver`, `Ms4525doTlpDriver`.
  - Drivers emit Outbound `Tlp64` requests (`MemRead`, `MemWrite`) via `TlpChannel`.
  - Sensor settling times use non-blocking `co_await sleep_ms()`.
- [x] **5.2 Bottom-Half PCIe TLP Scheduler Engine** (`src/target/common/pcie_tlp_scheduler.hpp`):
  - Centralized dispatcher dequeuing Outbound TLPs and routing to FPGA PCIe, RP2350 PIO, native SPI/I2C/UART DMA, or SITL mock.
  - Generates Inbound `Tlp64` completion packets with nanosecond timestamps.
- [x] **5.3 CppUTest / Unit Test Suite (20/20 Test Suites Passing)**:
  - Comprehensive unit test suite covering pure TLP drivers, SPSC rings, and scheduler routing (`test_decoupled_tlp_drivers_and_scheduler`).
- [x] **5.4 Linux SBC Board Test Harness (C++ & Python)**:
  - C++ `inav_linux_sbc` binary interfacing with PCIe/UIO/spidev/i2c-dev to validate live TLP ring streaming.
  - Python `tools/test_board_hardware.py` validating register roundtrips, throughput, and packet integrity over Linux hardware.
- [x] **5.5 RP2350 Pico 2 W Hardware Test Harness (`src/target/pico2_rp2350/pico2_hw_test.cpp`)**:
  - **Live IMU Acquisition Test**: WHO_AM_I probe, PIO state machine DMA burst reader, and verifying continuous 8 kHz accelerometer/gyro sample streaming.
  - **Parallel Multi-Driver Init Test (`when_all` / `&&`)**: Concurrently booting IMU, Baro, Mag, Pitot, and GPS, measuring hardware boot time to prove $T_{\text{boot}} = \max(\text{delays})$ on real silicon.
  - **Watchdog Timeout Race Test (`when_any` / `||`)**: Verifying unplugged I2C/SPI sensors time out safely without stalling the core.
  - **USB CDC / Serial Test Runner**: Clean diagnostic report output over USB serial (`/dev/ttyACM0`) with pass/fail metrics and live sensor readings.

---

## ⚡ Quick Validation Commands

```bash
# 1. Run Complete System Validation Pipeline
python3 tools/run_all_validations.py

# 2. Run Comprehensive Unit Tests
cd build && ./run_unit_tests

# 3. Run Native Submodule Differential Test Runner
cd build && ./submodule_differential_test

# 4. Run Task Scheduler vs Coroutine Benchmark
cd build && ./scheduler_benchmark

# 5. Run Full-Stack 60-Second Autonomous Mission Parity Test
cd build && ./full_stack_parity_test

# 6. Run Linux Board Hardware Test Suite (Python & C++)
python3 tools/test_board_hardware.py --device /dev/uio0
```
