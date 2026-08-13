# Embedded Template Library (ETL) Integration & Thread Safety Specification

> [!IMPORTANT]
> **ETL THREAD SAFETY & MULTI-CORE IPC**
> In **`inav-abstractx`**, **ETL (Embedded Template Library)** containers are used uniformly across **Linux SITL**, **Linux SBC + FPGA**, and **RP2350 Pico 2**.
>
> Below is the thread safety breakdown for ETL containers across multi-threaded and multi-core execution:

---

## 1. ETL Container Thread Safety Architecture

1. **Single-Thread Reentrancy**:
   - Standard ETL containers (`etl::vector`, `etl::string`, `etl::array`) are lightweight and non-blocking.
   - Concurrent reads from multiple threads are **100% thread-safe**.
   - Concurrent writes to the *same container instance* from multiple threads require external synchronization or lock-free queues.

2. **Multi-Thread / Multi-Core Inter-Process Communication (IPC)**:
   - ETL includes lock-free atomic queues (`etl::spsc_queue<T, N>`) designed for lock-free IPC across CPU cores.
   - In **`inav-abstractx`**, inter-thread and inter-core TLP messaging uses our lock-free, wait-free **Single-Producer Single-Consumer TLP Ring** ([`spsc_tlp_ring.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/spsc_tlp_ring.hpp)), which uses C++20 atomic memory barriers (`std::atomic_thread_fence`) to pass 64B TLPs between Core 0 and Core 1 or Linux threads without mutex stalls!

---

## 2. Standard ETL Containers Sanctioned

```cpp
#include "etl/vector.h"
#include "etl/string.h"
#include "etl/array.h"

namespace abstractx::flight {

// 1. Waypoint Vector (Fixed-capacity 16 waypoints, 0 bytes dynamic heap!)
using WaypointVector = etl::vector<Waypoint, 16>;

// 2. Pilot Name String (Fixed-capacity 32 chars, 0 bytes dynamic heap!)
using PilotNameString = etl::string<32>;

// 3. Motor Command Output Array
using MotorOutputArray = etl::array<uint16_t, 8>;

} // namespace abstractx::flight
```

---

## 3. Enforcement

1. `std::vector`, `std::string`, `std::map`, `std::unordered_map`, and `std::deque` are **BANNED ON ALL TARGETS** (Linux SITL included).
2. ETL fixed-capacity containers are used across all 3 targets: Desktop Linux SITL, Linux SBC + FPGA, and RP2350 Pico 2.
