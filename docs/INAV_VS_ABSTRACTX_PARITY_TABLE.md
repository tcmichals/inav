# Comprehensive Comparison Table: Legacy iNav vs Legacy Betaflight vs `inav-abstractx`

> [!IMPORTANT]
> **THREE-WAY PARITY & ARCHITECTURE COMPARISON**
> This table provides a 3-way side-by-side comparison of **Hardware Abstraction**, **Motor/Servo Mixers**, **Flight Controllers & PID Math**, **Euler/Quaternion Math**, and **State Estimators** across **Legacy iNav**, **Legacy Betaflight**, and the new **`inav-abstractx` C++20 Port**.

---

## 3-Way Architecture & Math Comparison Table

| Feature Domain | Legacy iNav (Legacy C) | Legacy Betaflight (Legacy C) | New `inav-abstractx` Port (C++20) | Advantage of New Port |
| :--- | :--- | :--- | :--- | :--- |
| **Autonomous Navigation** | Full RTH, Waypoints, Position Hold | Basic RTH (Rescue Mode only) | **iNav Autonomous Navigation Engine** ([`navigation.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/navigation.hpp)) | Full autonomous 3D navigation & RTH |
| **Flight PID Dynamics** | Basic PID loop | Advanced Betaflight PID (Feedforward, RPM notch) | **Betaflight-Grade C++20 PID Controller** ([`pid.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/pid.hpp)) | Acro/FPV dynamics + iNav navigation |
| **Hardware Driver Layer** | Monolithic `target.c` & STM32 HAL | Monolithic `target.c` & STM32 HAL | **Modular per-device files** ([`icm42688p.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/icm42688p.hpp), [`bmp280.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/bmp280.hpp)) | Clean git tracking; zero HAL bloat |
| **Bus Access Model** | Direct SPI/I2C bit-banging & IRQs | Direct SPI/I2C bit-banging & IRQs | **AbstractX Virtual BAR Map** (`PCIE_BAR_IMU_BASE`, `PCIE_BAR_ESC_BASE`) | 100% platform-agnostic (SITL, Pico 2, FPGA) |
| **Airframe Motor Mixer** | Fixed C array (`MAX_MOTORS=16`) | Fixed C array (`MAX_MOTORS=8`) | **C++20 Template Mixer** ([`mixer.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/mixer.hpp): `Mixer<N>`) | Zero RAM waste; unrolled compile-time loop math |
| **Euler / Attitude Math** | Legacy C Euler integration | Legacy C Euler integration | **C++20 Attitude Filter** ([`attitude.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/attitude.hpp)) & Quaternions | High-precision vector math without trig stalls |
| **Sensor Fusion & EKF** | Complementary filter & 10-state EKF | Basic complementary filter | **C++20 15-State EKF3 Filter** ([`ekf3.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/ekf3.hpp)) | Uses 64-bit nanosecond hardware timestamps ($<20\text{ ns}$) |
| **Timestamping Source** | Software IRQ latency ($20\text{--}50\ \mu\text{s}$) | Software IRQ latency ($20\text{--}50\ \mu\text{s}$) | **64-bit Hardware Clock** latched at `DRDY` edge | Zero velocity derivative noise during EKF state updates |
| **Target Build Setup** | Preprocessor `#ifdef USE_ACC` macro spaghetti | Preprocessor `#ifdef USE_ACC` macro spaghetti | **C++20 Concepts** ([`target_interface.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/target_interface.hpp)) & `if constexpr` | 100% active code in IDE; hard compile errors |
| **Memory Allocation** | Dynamic C heap allocation (`malloc`/`free`) | Dynamic C heap allocation (`malloc`/`free`) | **Strict Zero Heap** (`0 bytes` dynamic allocation) | Deterministic flight timing; no heap fragmentation |
| **Config Persistence** | Linker section hacks (`.pg_registry`) | Linker section hacks (`.pg_registry`) | **C++20 Flat POD Registry** ([`config_registry.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/config_registry.hpp)) | Loads/saves `config.bin` directly; zero linker hacks |
| **Configurator GUI** | iNav Configurator (MSP v1/v2) | Betaflight Configurator (MSP v1/v2) | **Configurable MSP TCP Server** ([`msp_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/msp_server.hpp): TCP `5760`) | Connects live to **iNav Configurator** over TCP or serial |
| **CLI Command Line** | Legacy C text scanner | Legacy C text scanner | **C++20 CLI Engine** ([`cli_engine.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/cli_engine.hpp)) | Full `dump`, `status`, `save`, `defaults` support |
| **Blackbox Tracing** | Custom `.BBL` varint bit-packing on CPU | Custom `.BBL` varint bit-packing on CPU | **BareCTF 64B TLPs** + Python converter ([`ctf_to_blackbox.py`](file:///home/tcmichals/ssdData/projects/home/inav/tools/ctf_to_blackbox.py)) | Zero CPU overhead in flight loop; open CTF trace format |
