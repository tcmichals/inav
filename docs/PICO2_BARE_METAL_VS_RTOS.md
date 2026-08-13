# RP2350 Pico 2 & Pico 2 W: Bare-Metal Dual-Core vs RTOS Technical Comparison

This document analyzes the execution models for the RP2350 dual-core processor, comparing a bare-metal dual-core architecture with a preemptive Real-Time Operating System (RTOS).

---

## 1. Dual-Core Task Allocation

```
  RP2350 Pico 2 W Dual ARM Cortex-M33 (150 MHz)
  ┌───────────────────────────────────────────────┐
  │                                               │
  │  Core 0: Peripheral & Telemetry Offloader    │
  │  - PIO0 Engine: DShot Motor Pulse Waves       │
  │  - PIO1 Engine: CRSF RC Receiver Serial RX   │
  │  - PIO2 Engine: Auto-SPI IMU Burst DMA        │
  │  - CYW43439: Wi-Fi iNav Configurator (5760)   │
  │                                               │
  ├───────────────────────────────────────────────┤
  │                                               │
  │  Core 1: Dedicated Flight Control Engine      │
  │  - 8 kHz Zero-Alloc C++20 Coroutine Loop      │
  │  - 15-State EKF3 Multi-Sensor Fusion          │
  │  - Betaflight 3-Axis PID Dynamics             │
  │  - iNav 3D Autonomous RTH Navigation          │
  │                                               │
  └───────────────────────────────────────────────┘
```

---

## 2. Execution Model Comparison

| Parameter | Preemptive RTOS (e.g. FreeRTOS) | Bare-Metal Dual-Core (`inav-abstractx`) |
| :--- | :--- | :--- |
| **Task Scheduling** | Preemptive, priority-based | Cooperative coroutine loop on Core 1 |
| **Context Switch Mechanism** | Register save/restore to task stack | Cooperative frame pointer swap |
| **Stack Memory** | Dedicated per-task stacks (e.g., 2–4 KB each) | Shared static pool (`CoroutineStaticPool`) |
| **I/O Handling** | Interrupt Service Routines (ISRs) | PIO state machines & DMA on Core 0 |
| **Data Passing** | Queues / Mutexes | Atomic SPSC ring buffer (`SpscTlpRing`) |
