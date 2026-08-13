# AI Assistant Design Guardrails & Coding Rules (`tcmichals/inav` - `pcie-clean` branch)

> [!IMPORTANT]
> **MANDATORY AI AGENT DIRECTIVE**
> Any AI assistant, pair programmer, or developer modifying or generating code in this repository MUST strictly follow the design rules and constraints documented in this file. Failure to obey these guardrails breaks flight determinism and hardware safety.

---

## 1. MISRA C++:2023 & Safety-Critical Standards Compliance (ISO 26262 / DO-178C)

All code written in this repository MUST adhere to **MISRA C++:2023** safety-critical coding standards:

1. **MISRA Rule 18-0-1 (Zero Dynamic Heap Memory)**:
   - `malloc()`, `calloc()`, `realloc()`, `free()`, `new`, and `delete` are **STRICTLY FORBIDDEN**.
   - STL dynamic containers (`std::vector`, `std::string`, `std::map`, `std::deque`) are **BANNED ON ALL TARGETS**.
2. **MISRA Rule 5-0-15 (No Raw Pointer Arithmetic)**:
   - Pointer arithmetic is forbidden. All memory access MUST use strongly typed `std::span<T>` or `std::array<T, N>`.
3. **MISRA Rule 5-2-4 (No C-Style Casts)**:
   - C-style casting (`(int)value`) is forbidden. Explicit `static_cast<T>()` or `reinterpret_cast<T>()` must be used.
4. **MISRA Rule 6-4-1 (Exhaustive Switch Labels)**:
   - All `switch` statements MUST include an explicit `default:` branch label.
5. **DO-178C Level A Determinism (Zero Recursion)**:
   - Function recursion is strictly forbidden. Call graphs MUST be bounded and statically verifiable at compile time.

---

## 2. Uniform ETL (Embedded Template Library) & Thread Safety Policy

1. **Uniform ETL Container Usage**:
   - Use fixed-capacity ETL containers (`etl::vector<T, N>`, `etl::string<N>`, `etl::deque<T, N>`) uniformly across **Linux SITL**, **Linux SBC + FPGA**, and **RP2350 Pico 2**.
2. **Thread Safety & Multi-Core IPC**:
   - ETL container instances are non-blocking and single-thread reentrant by default.
   - For inter-thread and inter-core TLP messaging across cores (Core 0 vs Core 1 or Linux threads), code MUST use lock-free, wait-free Single-Producer Single-Consumer SPSC ring buffers (`abstractx::SpscTlpRing<64>`) with atomic memory barriers.

---

## 3. BANNED: No `<iostream>`, `<fstream>`, `std::cout`, or `printf` Stream IO

1. **NO `<iostream>`, `<fstream>`, `std::cout`, or `printf` Headers**:
   - `<iostream>`, `<fstream>`, `std::cout`, `std::cerr`, `std::cin`, `printf()`, `puts()`, and POSIX file stream I/O are **STRICTLY BANNED** in all core flight loop, driver, sensor, navigation, and storage modules.
   - **Rationale**: Standard stream headers inflate MCU flash binary size by $> 200\text{ KB}$, fail on bare-metal ARM Cortex-M33 / RISC-V targets, block execution threads, and ruin real-time loop timing.
2. **Mandatory BareCTF Binary Tracing**:
   - All event logging, warnings, flight data records, and trace metrics MUST use the zero-allocation **BareCTF Logger** ([`blackbox_logger.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/blackbox_logger.hpp)):
     - `BlackboxLogger::log_info(log_ring, "Event description", timestamp_ns);`
     - `BlackboxLogger::log_ctf_event(log_ring, timestamp_ns, ctf_event);`
   - Logs stream over 64-byte TLPs on channel `ASP_CHANNEL_FC_LOG` (`0x03`).

---

## 4. Platform-Agnostic Storage & Flash Abstraction API

1. **Memory Span Storage Operations**:
   - Storage reads and writes MUST use `std::span<uint8_t>` memory spans over the `FlashStorageAdapter` API ([`flash_storage.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/flash_storage.hpp)).
   - POSIX file I/O operations are restricted exclusively to SITL desktop targets inside `src/target/sitl/`.

---

## 5. 100% Feature Complete Driver & Peripheral Mandate

1. **Complete Peripheral & Driver Coverage**:
   - All driver interfaces (RC receiver protocols, DShot/OneShot ESCs, PWM decoders, WS2812 LED strip / Light controllers, Power ADC meters, Barometers, Magnetometers, and 4way-if ESC Passthrough) MUST maintain 100% feature completeness without dummy stubs.
2. **BLHeli / AM32 ESC Passthrough (`4way-if`)**:
   - The flight engine MUST provide full 4way-interface serial passthrough support (`MSP_SET_4WAY_IF`) so **BLHeli Configurator**, **AM32 Configurator**, or web ESC configurators can flash and configure ESCs directly through the FC.

---

## 6. C++20 Coroutine Constraints & Deterministic Timing

1. **Zero-Heap Coroutine Promises**:
   - Standard C++20 coroutine allocation (`operator new` for coroutine frames) is banned unless backed by static memory pools or inline promise frame storage.
   - All custom coroutine promise types (`Task<T>::promise_type`) MUST overload `operator new(size_t size)` to allocate from static buffer pools (`asp_coroutine_pool`).
2. **No Thread Blocking or Mutex Stalls**:
   - Fast-path sensor and flight loops MUST NEVER invoke blocking thread synchronization (`std::mutex`, `pthread_mutex_lock`, `std::condition_variable`).
   - Use wait-free Single-Producer Single-Consumer (SPSC) ring buffers (`abstractx::SpscTlpRing`) for inter-thread and inter-core TLP message passing.

---

## 7. AbstractX PCIe TLP Bus & Canonical Header Enforcement

1. **Canonical Header Location**:
   - All AbstractX TLP structures (`asp_tlp64_t`), opcodes, virtual BAR register base addresses (`PCIE_BAR_IMU_BASE`), and byte layouts MUST be included directly from `../AbstractX/include/` (`asp_tlp64.h`, `pcie_reg_api.h`, `asp_tlp64.hpp`).
2. **Strict 64-Byte Packet Alignment**:
   - All TLP memory transactions MUST maintain 64-byte alignment (`__attribute__((packed, aligned(64)))`).
   - Wire representations MUST verify layout at compile time using `static_assert(sizeof(asp_tlp64_t) == 64)`.

---

## 8. Hardware Vendor SDK Isolation

1. **No Vendor HAL Leakage**:
   - Driver code in `src/drivers/` and flight logic in `src/flight/` MUST NEVER include hardware vendor SDK headers (`stm32h7xx_hal.h`, `pico/stdlib.h`, `xilinx_gpiops.h`).
   - All hardware interaction MUST be mediated through `abstractx::pcie_reg_read32()`, `abstractx::pcie_reg_write32()`, or `abstractx::SpscTlpRing` streams.
2. **Target Backend Encapsulation**:
   - Hardware-specific peripheral initializers, DMA ring setups, or PIO state machine loaders MUST reside exclusively inside `src/target/<platform>/`.
