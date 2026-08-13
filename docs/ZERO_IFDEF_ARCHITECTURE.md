# Zero `#ifdef` Architecture Specification: Eliminating Macro Spaghetti

> [!IMPORTANT]
> **MANDATORY ARCHITECTURAL DIRECTIVE**
> Legacy Betaflight and iNav codebases are littered with thousands of preprocessor macros (`#ifdef USE_ACC`, `#ifdef STM32F4`, `#ifdef MOTOR_COUNT_8`, `#ifdef USE_DSHOT`).
>
> In **`inav-abstractx`**, preprocessor `#ifdef` macros are **STRICTLY BANNED** in core flight software, drivers, and navigation modules. We replace preprocessor spaghetti with **C++20 Concepts**, **`if constexpr` compile-time evaluation**, **Virtual BAR address mapping**, and **strongly typed policy classes**.

---

## 1. Why Legacy `#ifdef` Code is Inferior

| Problem | Legacy Betaflight / iNav (`#ifdef`) | `inav-abstractx` (C++20 Solution) |
| :--- | :--- | :--- |
| **Readability** | Nesting of up to 8 levels of `#ifdef`/`#endif` | Clean, readable standard C++ code |
| **IDE Support** | IDE auto-completion greyed out; syntax checking fails | 100% active code syntax highlighted & checked by IDE |
| **Compilation Drift** | Silent build failures when macro names mismatch | Hard compile-time errors with clear diagnostic messages |
| **Testability** | Only 1 target configuration can be built per compile | Multiple targets & configurations testable in a single unit test |

---

## 2. The 4 Modern C++20 Patterns Replacing `#ifdef`

### Pattern 1: C++20 Concepts (`concepts::IsPlatform`, `concepts::IsSensor`)
Instead of `#ifdef STM32F4`:
```cpp
// src/target/target_interface.hpp
template <typename T>
concept IsPlatform = requires(T platform, uint32_t addr, uint32_t val) {
    { platform.reg_read32(addr) } -> std::same_as<uint32_t>;
    { platform.reg_write32(addr, val) } -> std::same_as<void>;
};
```
Target platforms (Linux SBC + FPGA, RP2350 Pico 2, STM32, SITL) implement standard concepts. Compiler enforces interface validity.

---

### Pattern 2: `if constexpr` Compile-Time Branching
Instead of `#ifdef USE_DSHOT`:
```cpp
// Evaluated at compile-time by C++ compiler (Zero runtime overhead, Zero #ifdefs!)
template <EscProtocol Protocol>
void dispatch_motor_commands(const std::array<uint16_t, 4>& commands) {
    if constexpr (Protocol == EscProtocol::DShot600) {
        send_dshot_tlp(commands, 600);
    } else if constexpr (Protocol == EscProtocol::OneShot125) {
        send_oneshot_tlp(commands, 125);
    }
}
```

---

### Pattern 3: Virtual BAR Address Mapping
Instead of `#ifdef USE_SPI_DEVICE_1`:
- Peripherals are accessed over Virtual BAR address offsets (`PCIE_BAR_IMU_BASE`, `PCIE_BAR_ESC_BASE`).
- Driver logic writes to virtual BAR offsets regardless of whether the hardware target is a Zynq FPGA, RP2350 PIO, or STM32 MDMA!

---

### Pattern 4: C++20 Policy Templates for Airframes (`Mixer<N>`)
Instead of `#ifdef MAX_SUPPORTED_MOTORS`:
```cpp
// Mixer parameterized on exact motor count (N=4, N=6, N=8, N=5)
abstractx::flight::Mixer<4> quad_mixer(presets::QuadX);
abstractx::flight::Mixer<8> octo_mixer(presets::OctoX8);
```
- Allocates an array of *exact* motor size. Zero RAM waste, zero unused loop iterations.
