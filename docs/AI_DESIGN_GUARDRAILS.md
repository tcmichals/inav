# AI Assistant Design Guardrails & Coding Rules (`tcmichals/inav` - `pcie-clean` branch)

> [!IMPORTANT]
> **MANDATORY AI AGENT DIRECTIVE**
> Any AI assistant, pair programmer, or developer modifying or generating code in this repository MUST strictly follow the design rules and constraints documented in this file. Failure to obey these guardrails breaks flight determinism and hardware safety.

---

## 1. Zero Dynamic Memory Allocation Guarantee

1. **NO Heap Allocation During Execution**:
   - `malloc()`, `calloc()`, `realloc()`, `free()`, `new`, and `delete` are **STRICTLY FORBIDDEN** in all driver, flight controller, sensor, and telemetry modules.
   - STL containers requiring dynamic allocation (`std::vector`, `std::string`, `std::map`, `std::unordered_map`, `std::deque`, `std::list`) are **FORBIDDEN**. Use fixed-size arrays (`std::array`), fixed-capacity spans, or static SPSC ring buffers.
2. **Static & Stack Allocation Only**:
   - All buffers, rings, driver instances, and state machines MUST be statically allocated at startup or embedded directly inside statically allocated container objects.

---

## 2. C++20 Coroutine Constraints & Deterministic Timing

1. **Zero-Heap Coroutine Promises**:
   - Standard C++20 coroutine allocation (`operator new` for coroutine frames) is banned unless backed by static memory pools or inline promise frame storage.
   - All custom coroutine promise types (`Task<T>::promise_type`) MUST overload `operator new(size_t size)` to allocate from static buffer pools (`asp_coroutine_pool`).
2. **No Thread Blocking or Mutex Stalls**:
   - Fast-path sensor and flight loops MUST NEVER invoke blocking thread synchronization (`std::mutex`, `pthread_mutex_lock`, `std::condition_variable`).
   - Use wait-free Single-Producer Single-Consumer (SPSC) ring buffers (`abstractx::SpscTlpRing`) for inter-thread and inter-core TLP message passing.

---

## 3. AbstractX PCIe TLP Bus & Canonical Header Enforcement

1. **Canonical Header Location**:
   - All AbstractX TLP structures (`asp_tlp64_t`), opcodes, virtual BAR register base addresses (`PCIE_BAR_IMU_BASE`), and byte layouts MUST be included directly from `../AbstractX/include/` (`asp_tlp64.h`, `pcie_reg_api.h`, `asp_tlp64.hpp`).
   - DO NOT duplicate or redefine TLP wire structures in this repository.
2. **Strict 64-Byte Packet Alignment**:
   - All TLP memory transactions MUST maintain 64-byte alignment (`__attribute__((packed, aligned(64)))`).
   - Wire representations MUST verify layout at compile time using `static_assert(sizeof(asp_tlp64_t) == 64)`.

---

## 4. Hardware Vendor SDK Isolation

1. **No Vendor HAL Leakage**:
   - Driver code in `src/drivers/` and flight logic in `src/flight/` MUST NEVER include hardware vendor SDK headers (`stm32h7xx_hal.h`, `pico/stdlib.h`, `xilinx_gpiops.h`).
   - All hardware interaction MUST be mediated through `abstractx::pcie_reg_read32()`, `abstractx::pcie_reg_write32()`, or `abstractx::SpscTlpRing` streams.
2. **Target Backend Encapsulation**:
   - Hardware-specific peripheral initializers, DMA ring setups, or PIO state machine loaders MUST reside exclusively inside `src/target/<platform>/`.

---

## 5. Strict Code Quality & Safety Rules

1. **No Unchecked Pointers**:
   - Null pointers, unchecked raw pointer arithmetic, and `reinterpret_cast` across non-aligned memory bounds are forbidden. Use strongly typed references or `std::span`.
2. **Deterministic Embedded Timing**:
   - All sensor data packets MUST preserve 64-bit nanosecond hardware timestamps (`timestamp_ns`) latched at the exact hardware trigger edge (`DRDY`). Software timestamping in user code is forbidden for EKF inputs.
