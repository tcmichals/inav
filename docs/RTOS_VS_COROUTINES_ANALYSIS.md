# Real-Time Execution Model Analysis: Preemptive RTOS vs C++20 Coroutines

This document provides a technical comparison between preemptive RTOS scheduling and zero-allocation C++20 coroutine execution for low-latency flight control loops.

---

## 1. Technical Trade-Off Comparison

| Metric / Dimension | Preemptive RTOS (e.g. FreeRTOS / ChibiOS) | C++20 Coroutines + PIO (`inav-abstractx`) |
| :--- | :--- | :--- |
| **Scheduling Type** | Preemptive timer tick | Cooperative yielding (`co_await`) |
| **Context Switch Overhead** | CPU register push/pop to task stack | Cooperative coroutine frame pointer swap |
| **Task Stack Memory** | Individual static/dynamic stacks per task | Static frame memory pool (`CoroutineStaticPool`) |
| **I/O Processing** | CPU interrupt handlers | Hardware PIO state machines & DMA |
| **Synchronization** | Semaphores, mutexes, task notifications | Lock-free SPSC TLP ring buffer |

---

## 2. Technical Characteristics

1. **Preemptive RTOS**: Provides priority-based preemption, task isolation, and standard POSIX/FreeRTOS API compatibility. Incurs context switch overhead and per-task stack allocation.
2. **C++20 Coroutines**: Provides explicit cooperative scheduling points, low context switch latency, and single-stack execution. Requires non-blocking task design.
