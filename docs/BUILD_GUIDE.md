# Comprehensive Build Guide: Linux SITL, RP2350 Pico 2 & Linux SBC Targets

> [!IMPORTANT]
> **BUILD SYSTEM SPECIFICATION**
> In **`inav-abstractx`**, CMake 3.20+ is used as the unified build system across all targets.
>
> Below are step-by-step instructions to build for **Desktop Linux SITL**, **RP2350 Pico 2**, and **Linux SBC + FPGA** platforms.

---

## 1. Building Desktop Linux SITL / HIL Simulator (`inav_abstractx_sitl`)

### Prerequisites (Ubuntu / Debian / Fedora)
```bash
sudo apt update && sudo apt install -y build-essential cmake g++ git
```

### Build Commands
```bash
# 1. Navigate to inav workspace
cd /home/tcmichals/ssdData/projects/home/inav

# 2. Configure CMake build directory
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 3. Build SITL binary
cmake --build build -j$(nproc)

# 4. Launch SITL Flight Engine
./build/inav_abstractx_sitl
```

Upon launching, the binary will listen on **MSP TCP Port 5760** for **iNav Configurator** connections!

---

## 2. Building for RP2350 Pico 2 (Raspberry Pi Pico 2)

### Prerequisites (ARM / RISC-V Toolchain)
```bash
# Install ARM Cortex-M toolchain
sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi

# Clone Pico SDK (if not already installed)
git clone --recursive https://github.com/raspberrypi/pico-sdk.git ~/.pico-sdk
export PICO_SDK_PATH=~/.pico-sdk
```

### Build Commands (Pico 2 ARM Cortex-M33)
```bash
cd /home/tcmichals/ssdData/projects/home/inav

# 1. Configure CMake with Pico 2 toolchain
cmake -B build_pico2 \
    -DPICO_BOARD=pico2 \
    -DCMAKE_TOOLCHAIN_FILE=$PICO_SDK_PATH/cmake/preload/toolchains/util/find_compiler.cmake \
    -DPICO_SDK_PATH=$PICO_SDK_PATH

# 2. Build Pico 2 UF2 firmware binary
cmake --build build_pico2 -j$(nproc)
```

The output `inav_abstractx_pico2.uf2` file can be dropped directly onto the Pico 2 USB mass-storage drive!

---

## 3. Building for Linux SBC + FPGA (Cubie A5E / Zynq-7020)

### Cross-Compilation Setup
```bash
cd /home/tcmichals/ssdData/projects/home/inav

# Build using native GCC on Linux SBC (e.g. Cubie A5E ARM64)
cmake -B build_sbc -DCMAKE_BUILD_TYPE=Release
cmake --build build_sbc -j$(nproc)
```

---

## 4. CMake Options & Customization Flags

| CMake Option | Default | Description |
| :--- | :--- | :--- |
| `-DCMAKE_BUILD_TYPE` | `Debug` / `Release` | Sets optimization level (`-O3` for Release) |
| `-DABSTRACTX_PATH` | `../AbstractX` | Path to canonical AbstractX headers |
| `-DENABLE_MSP_SERVER` | `ON` | Enables Configurable MSP TCP/Serial Server |
| `-DENABLE_CTF_LOGGING`| `ON` | Enables BareCTF binary trace logging |
