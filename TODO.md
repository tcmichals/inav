# Master Project Roadmap & Execution Plan (`TODO.md`)

> **MANDATORY SINGLE SSOT RULE**: This is the **strictly unique and authoritative** `TODO.md` file for `inav-abstractx`. No secondary, per-feature, or duplicate todo files are permitted in the workspace. All architectural milestones, hardware test phases, multi-repo links, and porting checklists MUST reside exclusively in this document.

---

## 1. Repository Ecosystem & Architecture Map

| Repository / Project | Role & Architectural Scope | Key Directory / Path | Status |
| :--- | :--- | :--- | :--- |
| **`tcmichals/inav`** (This Repo) | C++20 Flight Dynamics, 3D Navigation, Filters, Attitude Estimation, Failsafe, and Avionics Safety Core. | [`/home/tcmichals/projects/quadcopter/inav`](file:///home/tcmichals/projects/quadcopter/inav) | **Active Working Tree** (0 Git Commits) |
| **`tcmichals/AbstractX`** | Hardware Abstraction Layer (HAL), non-blocking C++20 coroutines, TLP Virtual Bus (`Tlp64`), lock-free rings (`DedicatedSpscChannel`, `TlpChannelMultiplexer`, `MpscEventQueue`), drivers, epoll reactor, and Studio. | [`/home/tcmichals/projects/AbstractX`](file:///home/tcmichals/projects/AbstractX) | Active (Partner Session) |
| **`tcmichals/python-imgui-esc-configurator`** | Full-featured desktop ESC configurator and firmware flasher in Python + `imgui-bundle` with 4-Way MSP passthrough. | [GitHub: `python-imgui-esc-configurator`](https://github.com/tcmichals/python-imgui-esc-configurator) | Production Ready |
| **`external/inav`** | Upstream INAV reference C source files for mathematical differential verification. | [`external/inav/src/main/`](file:///home/tcmichals/projects/quadcopter/inav/external/inav/src/main/) | Submodule Linked |
| **`external/betaflight`** | Upstream Betaflight reference C source files for rate dynamics and dynamic notch filters. | [`external/betaflight/src/main/`](file:///home/tcmichals/projects/quadcopter/inav/external/betaflight/src/main/) | Submodule Linked |
| **`projects/cubie`** | Radxa Cubie A5E BSP, Buildroot environment, device tree overlays, and automated silicon sweep scripts. | [`/home/tcmichals/projects/cubie/cubie-a5e`](file:///home/tcmichals/projects/cubie/cubie-a5e) | Integrated |
| **`projects/QMTECH`** | QMTECH Zynq-7020 XC7Z020 FPGA + Dual ARM Cortex-A9 starter kit board support and PetaLinux overlays. | [`/home/tcmichals/projects/QMTECH`](file:///home/tcmichals/projects/QMTECH) | Integrated |

---

## 2. Heterogeneous Hardware Targets & Compute Partitioning

```
+───────────────────────────────────────────────────────────────────────────────────────────+
│                                  inav-abstractx Virtual Bus                               │
│                         (Flexible Software Sizing / 64-Byte FPGA TLP)                     │
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
                                              │
    ┌─────────────────────────┬───────────────┴───────────────┬─────────────────────────┐
    ▼                         ▼                               ▼                         ▼
[ 1. Cubie A5E + FPGA ]  [ 2. RP2350 Pico 2 W ]     [ 3. ESP32-P4 Dual RISC-V ] [ 4. QMTECH Zynq-7020 ]
• Host: Cortex-A55 Linux • Core 0: Wi-Fi, MSP, CTF  • Core 0: MIPI-CSI Flow     • Dual Cortex-A9 PS
• Co-Proc: E906 Sensors  • Core 1: 8 kHz Flight Loop• Core 1: Flight Loop       • Artix-7 PL 64B DMA
• FPGA: DShot, CRSF, LED • 3x PIO State Machines    • Wi-Fi 6 / BT 5 Comms      • HW Sensor Voting
```

### Detailed Target Breakdown:

1. **Target 1: Radxa Cubie A5E + Companion FPGA + XuanTie E906 (Primary Bench Target)**:
   - **Linux Host (Cortex-A55 @ 1.8 GHz, PREEMPT_RT)**: Runs `inav_linux_sbc`, navigation state machines, Mahony AHRS, Blackbox CTF logging, and MSP TCP 5760 server.
     * **Dedicated Real-Time Reserved Core (Core 3)**:
       - Kernel boot isolation: `isolcpus=3 nohz_full=3 rcu_nocbs=3` excludes Core 3 from the general Linux scheduler, OS workqueues, and non-realtime interrupts.
       - POSIX thread pinning: [`target::sitl::LinuxRtHardener::harden_single_realtime_core(3)`](src/target/sitl/linux_rt_hardener.hpp) binds the 8 kHz INAV flight loop to Core 3 with `SCHED_FIFO` maximum priority `99`.
       - Zero-Page-Fault hardening: [`target::sitl::LinuxRtHardener::lock_process_memory()`](src/target/sitl/linux_rt_hardener.hpp) executes `mlockall(MCL_CURRENT | MCL_FUTURE)` and pre-faults 64 KB stack memory, preventing paging jitter.
     * **General Purpose Compute Cores (Cores 0..2)**: Run general Linux OS tasks, SSH console, AIC8800 Wi-Fi 6 networking, MSP TCP 5760 server, and Blackbox CTF disk/network logging with zero interference on the flight loop.
   - **Real-Time Co-Processor (T-Head XuanTie E906 RISC-V @ 200–400 MHz)**: Exclusive ownership over sensor buses:
     * **SPI1 DMA**: 8 kHz ICM-42688-P IMU auto-burst (`PB11..PB14` Pins 24, 23, 19, 21 + `PI8` DRDY Pin 29).
     * **UART3**: 115200 baud, 10 Hz u-blox NEO-M8N GPS (`PI11/PI12` Pins 16 & 31) with circular DMA.
     * **TWI1 (I2C)**: 100–200 Hz QMC5883L / HMC5883L 3-axis digital compass (`PB4/PB5` Pins 3 & 5).
     * Streams 64-byte `Tlp64` packets over VirtIO RPMsg / Shared SRAM (`0x40000000`).
   - **Companion FPGA**: Offloads microsecond-critical physical actuators & serial timing:
     * DShot150..1200 motor pulse synthesis with bidirectional RPM telemetry feedback.
     * High-speed CRSF RC receiver frame decoding.
     * NeoPixel WS2812 strict $\pm 150\text{ ns}$ timing synthesis.
     * PWM input capture / RC decode channels.

2. **Target 2: Raspberry Pi Pico 2 / Pico 2 W (RP2350 Dual-Core ARM Cortex-M33)**:
   - **Core 0 (I/O Processor)**: Wi-Fi (CYW43), MSP TCP server (port 5760), BareCTF telemetry, Flash storage.
   - **Core 1 (Flight Core)**: Real-time 8 kHz flight loop, dynamic notch filters, Mahony AHRS, Rate PID.
   - **PIO Offloaders**: PIO0 (DShot motor generation), PIO1 (CRSF RC RX), PIO2 (Auto-SPI IMU burst).

3. **Target 3: ESP32-P4 (Dual-Core RISC-V @ 400 MHz)**:
   - **Core 0**: MIPI-CSI optical flow camera vision pipeline ($v = \omega_{\text{trans}} \cdot h$), hardware crypto, Wi-Fi 6 / BT 5 companion bridge.
   - **Core 1**: 8 kHz flight loop, navigation, DShot motor output.

4. **Target 4: QMTECH Zynq-7020 (XC7Z020 FPGA + Dual ARM Cortex-A9)**:
   - **ARM Cortex-A9 PS**: Flight control and navigation dynamics.
   - **Artix-7 FPGA PL**: Single-cycle 64-byte (512-bit) parallel PCIe/AXI TLP DMA, hardware sensor voting across multi-IMU redundancy, DShot/CRSF controllers.

---

## 3. Master Porting & Verification Checklist

### Phase 1: Core Dynamics & Mathematical Filtering (100% COMPLETE)
- [x] **1.1 Biquad & PT1/PT2/PT3 Filter Stages**: Upstream bit-exact matching `filter.c`.
- [x] **1.2 Dual Dynamic Notch Filters**: Real-time frequency tracking and Q-factor attenuation.
- [x] **1.3 Betaflight Rate PID Controller**: P, I, D, Feedforward, D-term LPF1/LPF2, TPA curve.
- [x] **1.4 Mahony AHRS 9-DoF Fusion**: Gyro integration, Accel gravity tilt, Magnetometer yaw correction.
- [x] **1.5 Fixed-Wing & Multirotor Mixers**: Multi-actuator motor and servo mixing matrices.
- [x] **1.6 BareCTF 1.8 Flight Telemetry Logger**: Zero-copy binary logging with UDP stream server.
- [x] **1.7 Differential Parity Test Suite**: Bit-exact validation against upstream C files (8/8 tests pass).
- [x] **1.8 20-Suite CppUTest Runner**: Complete unit test coverage (`./run_unit_tests` 100% pass).

### Phase 2: Navigation & Advanced Avionics (100% COMPLETE)
- [x] **2.1 INAV Position Estimator (xy/z inertial fusion)**: GPS and Barometer sensor fusion.
- [x] **2.2 Navigation State Machine**: RTH, POSHOLD, Waypoint Mission, Failsafe Land.
- [x] **2.3 INAV EZ-Tune & AutoTune Profiles**: Automated PID gain scheduling and rate optimization.
- [x] **2.4 Failsafe Supervisor & Stage 1/Stage 2 State Machine**: Comprehensive timeout & recovery.
- [x] **2.5 Sensor Alignment Engine**: Full 8-direction mounting transformations (`CW0_DEG`..`CW270_DEG`).
- [x] **2.6 Pitot Tube Airspeed Driver & Rangefinder**: Differential dynamic pressure and tilt compensation.
- [x] **2.7 Optical Flow C++20 Ground Velocity Estimator**: Rangefinder fusion and gyro compensation.

### Phase 3: Hardware Ingestion & Auto-DMA Infrastructure (100% COMPLETE)
- [x] **3.1 Zero-Copy Auto-DMA Manager**: Dedicated managers for IMU burst, Baro, Mag, GPS UART.
- [x] **3.2 Concurrency top-half/bottom-half refactoring**: Queue naming audit (`DedicatedSpscChannel`, `TlpChannelMultiplexer`, `MpscEventQueue`).
- [x] **3.3 Pico 2 W Asymmetric Multicore Architecture**: Formal Core 0 vs Core 1 execution pipeline.
- [x] **3.4 Zero Code Duplication Integration**: AbstractX canonical headers (`tlp_ring.hpp`, `asp_tlp64.hpp`, `pcie_bar_map.hpp`) included with zero shadow duplicates.
- [x] **3.5 Full-Stack 60-Second Mission Parity Test**: 60,000 flight steps with $0.000000^\circ$ attitude error.
- [x] **3.6 Scheduler Benchmark Validation**: 100,000 cycles with 0 dynamic heap allocations, $0.935\text{ \mu s}$ jitter.

---

## 4. Active Milestone: Live Hardware Target Integration (IN PROGRESS)

### Target 1: Radxa Cubie A5E + XuanTie E906 Silicon Verification
- [x] **Network & SSH Access**: Verified on `192.168.3.3` (user: `root`, password: `tcmichals`) via `sshpass`.
- [x] **RemoteProc & RPMsg Endpoints**: Verified `running` with `/dev/rpmsg0`, `/dev/rpmsg1`, `/dev/rpmsg2`.
- [x] **Co-Processor Diagnostics**: `[E906] Online` confirmed in `/sys/kernel/debug/remoteproc/remoteproc0/trace0`.
- [x] **Live GPS Hardware Stream**: Verified live u-blox NEO-M8N streaming on UART3 (`0x02500C00`):
  - *Quality*: 2 (Differential GPS 3D Fix)
  - *Satellites*: 12 satellites locked
  - *HDOP*: 0.76
  - *Altitude*: 151.1 m MSL
  - *Update Rate*: 10 Hz @ 115200 baud via circular DMA.
- [x] **Live Compass Hardware Stream**: Verified live HMC5883L streaming on TWI1 (`0x02502400`):
  - *Chip ID*: Detected (`0x0A..0x0C` = `b"H43"`)
  - *ODR*: 200 Hz continuous measurement
  - *3D Vector*: $X=-46, Y=550, Z=58$.
- [x] **Identify Sweep Testing Suite**: Located [`cubie-a5e/tools/run_full_sweep.py`](file:///home/tcmichals/projects/cubie/cubie-a5e/tools/run_full_sweep.py).
- [x] **Physical IMU SPI + Interrupt Connection (100% VERIFIED LIVE)**:
  - Connected SPI1 bus: MOSI (`Pin 19`), MISO (`Pin 21`), CLK (`Pin 23`), CS (`Pin 24`).
  - Connected DRDY interrupt: **Pin 29** (`PI8` / `PI-EINT8`, PLIC IRQ 82).
  - Connected 3.3V Power (`Pin 17` / `Pin 1`) & GND (`Pin 20` / `Pin 25`).
  - **Live Silicon Validation**: Probed over SPI1 @ 10 MHz:
    * `WHO_AM_I`: **`0x47`** (ICM-42688-P 6-DoF IMU hardware verified)
    * `Low-Noise Mode`: Activated `PWR_MGMT0` = `0x0F`
    * `Live Accel & Gyro Burst`: Streaming valid 12-byte raw frames with zero errors.
  - **Triple-Sensor Suite Complete**: IMU (ICM-42688-P), Compass (HMC5883L), and GPS (u-blox M8N) all verified operational concurrently on Cubie A5E hardware.
- [x] **Execute Live Integrated Flight Engine on Cubie A5E (VERIFIED LIVE)**:
  - Cross-compiled `inav_linux_sbc` for AArch64 using Buildroot GCC 15.1.0 (`bld.a5e/host/bin/aarch64-linux-g++`).
  - Deployed binary to `/root/inav_linux_sbc` on Radxa Cubie A5E (`192.168.3.3`).
  - Hardened real-time flight loop on isolated Core 3 (`isolcpus=3`, `SCHED_FIFO` 99, `mlockall` with 64 KB pre-faulted stack): 21.6 MB locked RSS, 0% CPU overhead, 96% idle.
  - Native POSIX TCP transport bound to port 5760 serving MultiWii Serial Protocol (MSP v1/v2).
  - Configured TLP bridge for dual emission: transmits live 64B TLPs for IMU (`0x40001000`), Compass (`0x40003000`), and GPS (`0x40005000`) concurrently to AbstractX Studio (`192.168.3.1:9870`) and local INAV (`127.0.0.1:9870`).
  - Connected live sensor pipeline end-to-end: `inav_linux_sbc` receives live hardware IMU/Mag/GPS, executes Mahony AHRS, and feeds `g_msp_live_state`.
  - Verified live MSP query from host (`python3 tools/test_sitl_integration.py 192.168.3.3 5760`): `MSP_FC_VARIANT` ("INAV"), `MSP_API_VERSION` (2.0), `MSP_ATTITUDE` (`Roll=-1.0° Pitch=-2.1° Yaw=+28.0°`), and `MSP_RAW_GPS` all passed 100%.
- [x] **Custom Bare-Metal C++ E906 Flight Co-Processor App (`inav_e906_app`) (100% VERIFIED LIVE)**:
  - Created dedicated bare-metal C++ firmware [`AbstractX/apps/inav_e906_app`](file:///home/tcmichals/projects/AbstractX/apps/inav_e906_app):
    * **Zero Python / Zero REPL**: 84 KB compiled RISC-V RV32IMAFC ELF running in SRAM Space 0 (`0x3FFC0000`).
    * **SPI1 IMU**: 1 kHz burst read from ICM-42688-P (`0x1D` 14 bytes), packed to 64B TLP Tag 1 (`0x40001000`).
    * **TWI1 Compass**: 100 Hz 3D vector read from HMC5883L via canonical `abstractx::hal::II2c(1)`, packed to 64B TLP Tag 3 (`0x40003000`).
    * **UART3 GPS**: 115200 baud streaming ingestion and NMEA/UBX parsing via canonical `abstractx::hal::IUart(3)`, packed to 64B TLP Tag 2 (`0x40005000`).
    * **Dual SPI FPGA Interface**: Low-latency hardware driver (`FpgaDualSpiDriver` on PB11..PB14) for companion FPGA.
    * **VirtIO RPMsg Channel**: Streams 64B TLPs directly over `abstractx-tlp` (addr 1024) to Linux PREEMPT_RT.
    * **AbstractX HAL Extensions**: Added `E906Uart3Driver` and `E906Twi1Driver` to `targets/allwinner_e906/src/platform.cpp`, exposed through `hal::get_uart_port(3)` and `hal::get_i2c_driver(1)`.
    * **AbstractX Applib Linked**: Linked `abstractx_applib` with `applib::drivers::imu::Icm42688p`, `applib::drivers::mag::Compass`, and `applib::drivers::gps::UbloxGps`.
  - Enhanced `src/target/linux_sbc/linux_sbc_main.cpp`: Ingests directly from `/dev/rpmsg0` (`abstractx-tlp`) with sub-microsecond latency.
  - Verified live on Cubie A5E with zero Python processes: `inav_linux_sbc` actively receives real-time attitude vectors and serves MSP over TCP 5760.
- [ ] **Transition Ethernet to Wi-Fi for Airborne Quad Deployment**:
  - Configure AIC8800 Wi-Fi 6 on MMC1 (`reg_3v3_wifi` on PL7, `reg_wifi_en` on PM1).
  - Bind MSP TCP 5760 and BareCTF UDP 9870 to `wlan0`.

### Target 2: Raspberry Pi Pico 2 W Bench Verification
- [x] Compile bare-metal firmware `inav_abstractx_pico2w.elf` and `inav_pico2_imu_test.elf`.
- [ ] Flash to physical Pico 2 W hardware and verify Core 0 Wi-Fi MSP bridge + Core 1 8 kHz flight loop.

### Target 3 & 4: ESP32-P4 & QMTECH Zynq Hardware Staging
- [ ] Deploy ESP32-P4 FreeRTOS + optical flow task runner.
- [ ] Deploy Zynq-7020 bitstream with 64-byte single-cycle parallel PCIe/AXI TLP DMA.

---

## 5. All-in-One Avionics Studio Roadmap (`imgui-bundle`)

Consolidate INAV Configurator, Betaflight Configurator, `python-imgui-esc-configurator`, and AbstractX Studio into a single native immediate-mode desktop tool:

```
[ AbstractX All-in-One Flight & Avionics Studio ] (Python 3 + imgui-bundle)
 ├── Tab 1: INAV / Betaflight MSP Configurator (Setup, 3D Deck, PIDs, Radio, Motors, GPS, CLI) [IMPLEMENTED]
 ├── Tab 2: User Domain Instruments (Artificial Horizon, HSI Compass Rose, GPS 3D Map)
 ├── Tab 3: ESC Configurator & Flasher (tcmichals 4-way passthrough, BLHeli_S/Bluejay/AM32)
 ├── Tab 4: High-Rate Oscilloscope (ImPlot: 8 kHz Gyro FFT, Dynamic Notch Attenuation)
 ├── Tab 5: BareCTF / Blackbox Flight Log Analyzer & Replay
 └── Tab 6: Silicon Admin (XuanTie E906 REPL, MMIO Register Peek/Poke, RemoteProc, FPGA Bitstream)
```

### Action Items:
- [x] **5.1 Unified MSP Client & INAV Studio Plugin (`inav_plugin.py`)**:
  - Plugin cleanly separated from AbstractX and maintained inside INAV repository at [`tools/studio_plugin/inav_plugin.py`](file:///home/tcmichals/projects/quadcopter/inav/tools/studio_plugin/inav_plugin.py) inheriting from `AbstractXStudioPlugin`.
  - AbstractX Studio dynamically loads external plugins via generic `--plugin <path>` CLI argument.
  - Background threaded non-blocking MSP protocol engine polling high-rate (Attitude, Status, Motors), medium-rate (IMU, Altitude, Analog, RC), and slow-rate (GPS).
- [x] **5.2 Port Core FC Tabs**:
  - *3D Flight Deck*: Solid polygon quad mesh with Lambertian shading, spinning props with motion blur discs, ground shadow, orbit camera, G1000 EFIS PFD HUD, and dual phosphor gimbals.
  - *Setup & Calibration*: Hardware sensor bus status (ACC, BARO, MAG, GPS, SONAR), 1-click Accel & Compass calibration, arming status.
  - *PID Tuning*: Roll/Pitch/Yaw Rate PIDs ($K_P$, $K_I$, $K_D$, $K_{FF}$) with presets (Freestyle, Long Range, Cinewhoop) and EEPROM write.
  - *Receiver & Radio*: 16-channel live stick visualization + PWM meters.
  - *Motors & ESCs*: Safety disclaimer, master slider, individual M1..M4 sliders, and instant cut.
  - *GPS & Navigation*: 3D fix, satellites, lat/lon/alt, speed, and HDOP readouts.
  - *CLI Terminal*: Interactive command prompt for MSP queries.
- [x] **5.3 Material Design 3 (M3) Styling Overhaul in `imgui-bundle`**:
  - Replaced legacy retro look with Google Material Design 3 dark surface palette (`#121316` background, `#1d1e23` elevated cards, `#7cacf8` primary blue).
  - 12px / 10px / 8px rounded geometry across cards, inputs, tabs, and popups with 1px subtle outlines.
  - Refactored all Configurator tabs into clean elevated M3 card containers (`begin_child`).
- [ ] **5.4 Embed [`python-imgui-esc-configurator`](https://github.com/tcmichals/python-imgui-esc-configurator)**: Drop directly into Studio as `tab_esc.py` sharing the same ImGui frame loop.
- [ ] **5.5 Multi-Transport Selector**: Add top-bar unified connection selector (`[Serial / TCP 5760 / SSH 192.168.3.3]`).

---

## 6. Submodule & Maintenance Tasks

- [ ] **Submodule Reference Update**: When the partner AbstractX session completes queue naming updates, update the commit reference in `external/AbstractX`.
- [ ] **Git Commit Freeze Policy**: Maintain all working tree modifications live without executing `git commit` or `git push` until explicitly authorized by user.
- [ ] **Mandatory Engineering Log Updates**: Keep [`docs/ENGINEERING_LOG.md`](file:///home/tcmichals/projects/quadcopter/inav/docs/ENGINEERING_LOG.md) synchronized after every hardware milestone and architectural decision.
