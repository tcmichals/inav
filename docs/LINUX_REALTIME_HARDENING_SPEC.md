# Linux Real-Time Thread Hardening Specification

> [!IMPORTANT]
> **ISOLATED CPU CORE, SCHED_FIFO PRIORITY 99, & LOCKED PHYSICAL RAM (`mlockall`)**
> When running `inav-abstractx` on Linux (SITL or Linux SBC + FPGA / Cubie A5E / Zynq-7020), the primary flight control thread is hardened against OS kernel latencies using POSIX real-time primitives ([`linux_rt_hardener.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_rt_hardener.hpp)).

---

## 1. POSIX Real-Time Hardening Steps

```cpp
#include "linux_rt_hardener.hpp"

int main() {
#if defined(__linux__)
    // Pin to Isolated CPU Core 3, set SCHED_FIFO priority 99, lock physical RAM
    target::linux_rt::LinuxRtHardener::harden_realtime_thread(3, 99);
#endif
    // Start C++20 Flight Engine
}
```

---

## 2. Hardening Features Breakdown

1. **Physical Memory Page Locking (`mlockall(MCL_CURRENT | MCL_FUTURE)`)**:
   - Locks all current and future mapped memory pages into physical RAM.
   - Prevents Linux kernel page faults and page swapping during flight loop execution ($0.0\text{ ms}$ page fault delay).

2. **Real-Time Priority Scheduling (`pthread_setschedparam(SCHED_FIFO, 99)`)**:
   - Assigns highest real-time priority (`99`) under POSIX `SCHED_FIFO` policy.
   - Ensures the flight loop thread immediately preempts background Linux user processes.

3. **Isolated CPU Core Pinning (`pthread_setaffinity_np`)**:
   - Pins the flight loop thread to a dedicated isolated CPU core (e.g. `CPU 3`).
   - Combined with Linux kernel boot argument `isolcpus=3` to guarantee zero OS timer interrupt interference!

4. **POSIX Stack Pre-Faulting**:
   - Pre-touches 64 KB of stack memory upon thread startup, eliminating demand paging stalls inside the PID loop.
