# AbstractX Flight Engine Engineering Log

This engineering log records design decisions, hardware trade-offs, architectural changes, and system verifications across the `tcmichals/inav` repository (`pcie-clean` branch).

---

## Log Entries

### [2026-08-13] Hardware Offloading & Platform Parity
- **Decision**: Adopted AbstractX PCIe TLP 64-byte frame abstraction ([`asp_tlp64.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/asp_tlp64.hpp)) across all target platforms.
- **Rationale**: Provides unified memory-mapped BAR register access across Linux SITL, RP2350 Pico 2 W, and Linux SBC + FPGA (`rt_offloader`).
- **Files Modified**: `src/drivers/imu_pcie_driver.hpp`, `src/drivers/esc_dshot_driver.hpp`, `src/target/target_interface.hpp`.

---

### [2026-08-13] Zero-Dynamic Allocation & ETL Policy
- **Decision**: Banned dynamic heap memory (`malloc`, `free`, `new`) during flight execution. Banned `<iostream>` and `printf` in core flight headers.
- **Rationale**: Eliminates heap fragmentation and memory leaks. Reduces MCU binary size under 200 KB.
- **Files Modified**: `src/abstractx_engine/coroutine_task.hpp`, `src/config/config_registry.hpp`, `docs/ETL_EMBEDDED_TEMPLATE_SPEC.md`.

---

### [2026-08-13] RP2350 Pico 2 W GPIO Pinout & Triple-PIO Offloading
- **Decision**: Assigned 26 GPIO pins on RP2350 Pico 2 W. Offloaded DShot motor pulses (PIO0), CRSF RC serial RX (PIO1), and Auto-SPI IMU burst DMA (PIO2) to PIO state machines.
- **Rationale**: Offloads physical signal timing from CPU cores, dedicating Core 1 exclusively to 8 kHz EKF3 and PID math.
- **Files Modified**: `src/target/pico2_rp2350/pico2_target.cpp`, `src/target/pico2_rp2350/pio_imu_reader.hpp`, `docs/PICO2W_PINOUT_SPEC.md`.

---

### [2026-08-13] Dynamic Motor Pin Signal Multiplexing
- **Decision**: Implemented runtime PIO microcode hot-swapping ([`pio_esc_reloader.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pio_esc_reloader.hpp)) on physical motor pins (`GPIO 2..5`).
- **Rationale**: Allows the same physical wires to switch between 1-way DShot flight outputs and half-duplex 19200 baud 1-wire serial UART for 8-bit BLHeli ESC flashing directly from BLHeli Suite.
- **Files Modified**: `src/target/pico2_rp2350/pio_esc_reloader.hpp`, `docs/PICO2_PIO_DYNAMIC_RELOAD_SPEC.md`.

---

### [2026-08-13] Linux Real-Time POSIX Hardening
- **Decision**: Implemented POSIX thread hardening ([`linux_rt_hardener.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_rt_hardener.hpp)) for Linux targets.
- **Rationale**: Pins flight loop thread to an isolated CPU core (`pthread_setaffinity_np`), sets `SCHED_FIFO` priority 99, locks physical RAM (`mlockall`), and pre-faults 64 KB stack memory.
- **Files Modified**: `src/target/sitl/linux_rt_hardener.hpp`, `src/main.cpp`, `docs/LINUX_REALTIME_HARDENING_SPEC.md`.

---

### [2026-08-13] Automated Testing & Parity Verification
- **Decision**: Added an 8-suite C++20 unit test runner (`./run_unit_tests`) and a Python differential math parity test (`python3 tools/compare_inav_parity.py`).
- **Rationale**: Verifies coroutines, fixed-capacity containers, EKF3 fusion, PID dynamics, 3D navigation, mixers, and driver parsing with 100% pass rate.
- **Files Modified**: `test/test_main.cpp`, `tools/compare_inav_parity.py`, `docs/UNIT_TEST_SPEC.md`, `docs/LEGACY_VS_NEW_DIFFERENTIAL_TESTING.md`.

---

### [2026-08-13] Documentation Tone Standardization
- **Decision**: Created workspace communication rules ([`.agents/AGENTS.md`](file:///home/tcmichals/ssdData/projects/home/inav/.agents/AGENTS.md)) requiring direct Yes/No answers, pure technical facts, critical trade-offs, and neutral, respectful phrasing.
- **Rationale**: Ensures documentation remains objective, un-opinionated, and machine-parsable for developers and AI tools.
- **Files Modified**: `.agents/AGENTS.md`, `docs/PICO2_ARCHITECTURE_ANALYSIS.md`, `README.md`.

---

### [2026-08-15] Pure Asynchronous Driver Architecture & Synchronous Removal
- **Decision**: Formally eliminated all legacy synchronous `bool init()` methods and blocking bus delays (`delay_ms()`) across all sensor drivers (`DPS310`, `QMC5883L`, `IST8310`, `MS4525DO`, `ICM42688P`, `BMP280`, `MS5611`, `BMI088`, `MPU6000`).
- **Rationale**: 
  1. The Bottom-Half layer is dedicated strictly to asynchronous PCIe TLP packet dispatching/receiving (`Tlp64` over `TlpChannel` and `SpscTlpRing`).
  2. Retaining synchronous blocking wrappers violates the non-blocking C++20 coroutine execution model, causes thread stalls, and prevents concurrent multi-sensor parallel initialization (`when_all`).
  3. All sensor initialization and sampling pipelines now operate 100% via non-blocking C++20 coroutines (`async_init()` and `sample_loop()`).
- **Files Modified**: `PLAN.md`, `docs/INAV_PORTING_MASTER_PLAN.md`, `docs/ENGINEERING_LOG.md`, `docs/HARDWARE_DRIVERS.md`, `src/drivers/baro/dps310.hpp`, `src/drivers/mag/qmc5883l.hpp`, `src/drivers/mag/ist8310.hpp`, `src/drivers/pitot/ms4525do.hpp`, `src/target/pico2_rp2350/pico2_main.cpp`, `test/test_main.cpp`.

---

### [2026-08-15] Sensor Physical Timeouts: Non-Blocking `co_await` vs. Thread Pends
- **Decision**: Established the core design rule: **"Sensors have limits and functional timeouts, so we await instead of pending."**
- **Rationale**:
  1. **Zero Thread Pends / Zero Core Stalling**: Physical sensor conversion latencies (e.g. MS5611 9.04ms $\Delta\Sigma$ integration, DPS310 15ms continuous mode, ICM-42688-P 2ms oscillator lock) must never block the CPU. In a synchronous/RTOS pending model, waiting $10\,\text{ms}$ drops 80 consecutive $8\,\text{kHz}$ PID cycles ($125\,\mu\text{s}$ period), causing flight instability.
  2. **Watchdog Racing (`when_any` / `||`)**: All hardware transactions race against a hardware watchdog timer (`bus.async_read() || sleep_us(TIMEOUT)`). If a sensor locks up or disconnects, the watchdog triggers safely without stalling the core.
  3. **Zero Jitter & Multi-Rate Concurrency**: Suspending a sensor task via `co_await` takes $<0.1\,\mu\text{s}$, allowing the 8 kHz IMU sampling, dynamic gyro notch filtering, and DShot ESC outputs to proceed with deterministic Worst-Case Execution Time (WCET).
- **Files Modified**: `docs/HARDWARE_DRIVERS.md`, `docs/ENGINEERING_LOG.md`, `docs/ARCHITECTURE.md`.

---

### [2026-08-15] Sensor Data Integrity & Hardware Anti-Overdriving Guards
- **Decision**: Implemented hardware-accurate timing guards across all driver coroutines to prevent sensor overdriving while parallelizing CPU execution.
- **Rationale**:
  1. **Integrity Preservation**: Polling physical sensors faster than their analog settling time (e.g. MS5611 9.04ms $\Delta\Sigma$, QMC5883L 200 Hz ODR) causes register corruption, stale reads, bus clock stretching, and thermal self-heating noise.
  2. **Asynchronous Parallel Efficiency**: Rather than serializing timing guards with blocking busy-waits, `co_await` yields the CPU, allowing the processor to parallelize independent bus I/O (SPI DMA, I2C, UART) and execute background tasks within the 72% idle headroom ($90.5\,\mu\text{s}$) of the 8 kHz IMU tick.
- **Files Modified**: `docs/HARDWARE_DRIVERS.md`, `docs/ENGINEERING_LOG.md`, `docs/ARCHITECTURE.md`, `README.md`.



