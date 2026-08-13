# Portable Vector & Matrix Math Specification vs Legacy ARM CMSIS-DSP

> [!IMPORTANT]
> **CMSIS-DSP VENDOR LOCK-IN ELIMINATION**
> In legacy iNav, matrix and trigonometric calculations relied on vendor-locked ARM CMSIS-DSP routines (`arm_math.h`, `arm_sin_f32`, `arm_mat_mult_f32`).
>
> In **`inav-abstractx`**, all flight math is implemented using **Portable C++20 Standard Vector & Matrix Templates** that compile to single-cycle FPU SIMD instructions on ARM Cortex-M33, RISC-V (Hazard3), ARM64, and x86_64 SITL!

---

## 1. CMSIS-DSP Replacement Matrix

| Legacy iNav / Betaflight Function (CMSIS-DSP) | Vendor Locking Problem | New `inav-abstractx` Portable C++20 Math | Vectorization / FPU Speed |
| :--- | :--- | :--- | :--- |
| `arm_sin_f32(angle)` | ARM Cortex-M only | `std::sin(angle)` | **Single-cycle Hardware FPU / Auto-Vectorized** |
| `arm_cos_f32(angle)` | ARM Cortex-M only | `std::cos(angle)` | **Single-cycle Hardware FPU / Auto-Vectorized** |
| `arm_sqrt_f32(val, &out)` | ARM Cortex-M only | `std::sqrt(val)` | **Native FPU `vsqrt.f32` instruction** |
| `arm_mat_mult_f32(&A, &B, &C)` | ARM Cortex-M only | `Matrix3x3::operator*()` | **Compiler auto-vectorized loop unrolling (`-O3`)** |
| `arm_biquad_cascade_df1_f32()` | ARM Cortex-M only | `BiquadFilter::update()` | **Unrolled FPU Multiply-Accumulate (`vfma.f32`)** |

---

## 2. Why Portable C++20 Math Beats Vendor CMSIS-DSP

1. **Cross-Platform Compilation**:
   - The exact same EKF3 filter code compiles natively on **ARM Cortex-M33** (RP2350), **RISC-V Hazard3** (RP2350), **x86_64 Desktop Linux SITL**, and **ARM64 Linux SBCs**.
2. **Zero MCU Library Dependency**:
   - Does not require bundling 50,000 lines of ARM CMSIS C source code into the build system.
3. **Compiler Optimization Parity**:
   - Modern GCC 12+ and Clang 15+ auto-vectorizers emit single-cycle `vadd.f32`, `vmul.f32`, and `vfma.f32` instructions directly from clean C++20 vector loops!
