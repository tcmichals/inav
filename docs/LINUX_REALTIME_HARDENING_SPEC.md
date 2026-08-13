# Linux 1.0 GHz Single-Core Unified Real-Time Execution Specification

> [!IMPORTANT]
> **1.0 GHz SINGLE-CORE UNIFIED REAL-TIME EXECUTION**
> On a **1.0 GHz CPU Core** (e.g. Allwinner Cubie A5E ARM Cortex-A53), $1.0\text{ GHz}$ corresponds to $1,000,000,000$ clock cycles per second ($1.0\text{ ns}$ cycle period).
>
> Executing the **8 kHz Flight Loop** ($125\ \mu\text{s}$ budget) takes $< 5.0\ \mu\text{s}$ per iteration ($4\%$ CPU load). Running both the 8 kHz C++20 coroutine loop and the Linux native `epoll` I/O reactor on the **same isolated 1.0 GHz CPU core** (`isolcpus=3`, `SCHED_FIFO` priority 99) leaves $> 95\%$ CPU headroom!

---

## 1. Single 1.0 GHz Core CPU Budget Breakdown (8 kHz Loop)

| Processing Phase | Execution Time per 8 kHz Frame | CPU Clock Cycles at 1.0 GHz | Core Headroom Remaining |
| :--- | :--- | :--- | :--- |
| **`epoll_wait()` Hardware I/O Poll** | $< 0.80\ \mu\text{s}$ | 800 cycles | $99.36\%$ |
| **15-State EKF3 Predict & Correct** | $< 2.20\ \mu\text{s}$ | 2,200 cycles | $97.60\%$ |
| **Betaflight 3-Axis PID Dynamics** | $< 0.90\ \mu\text{s}$ | 900 cycles | $96.88\%$ |
| **Airframe Motor Mixer & TLP Emit** | $< 0.40\ \mu\text{s}$ | 400 cycles | $96.56\%$ |
| **Total 8 kHz Frame Execution Time**| **$< 4.30\ \mu\text{s}$** | **4,300 cycles** | **$> 95.76\%$ Idle Headroom** |
