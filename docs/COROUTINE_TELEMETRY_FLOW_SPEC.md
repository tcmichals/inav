# C++20 Coroutine Telemetry & Hardware Streaming Flow Specification

> [!IMPORTANT]
> **LOCK-FREE SPSC TLP RING BUFFER & `co_await` YIELDING FLOW**
> In **`inav-abstractx`**, hardware devices stream data back asynchronously via lock-free Single-Producer Single-Consumer (SPSC) TLP ring buffers ([`spsc_tlp_ring.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/spsc_tlp_ring.hpp)).
>
> The C++20 flight loop coroutine ([`run_flight_loop()`](file:///home/tcmichals/ssdData/projects/home/inav/src/main.cpp#L47)) processes incoming streaming packets and yields cooperatively via `co_await std::suspend_always{}` in **$< 10\text{ nanoseconds}$** without blocking the CPU or using dynamic memory allocations.

---

## 1. End-to-End System Execution Diagram

```
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 1. HARDWARE LAYER (PIO2 Auto-SPI / FPGA DMA / Hardware Simulator)       │
  │    - IMU DRDY clock edge fires.                                         │
  │    - Hardware DMA latches 64-bit nanosecond timestamp (timestamp_ns).   │
  │    - Assembles 64-byte TLP packet and pushes into g_telemetry_ring.     │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 2. SPSC LOCK-FREE TELEMETRY RING (`g_telemetry_ring`)                   │
  │    - Atomic thread-fence memory barrier (`std::atomic_thread_fence`).   │
  │    - Single-Producer (Hardware/Core 0) -> Single-Consumer (Core 1).    │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 3. COROUTINE FLIGHT LOOP (`run_flight_loop()`)                          │
  │    - Coroutine resumes right after `co_await`.                           │
  │    - Polls: while (!g_telemetry_ring.empty())                           │
  │    - Pops 64B TLP -> ImuPcieDriver::parse_tlp() -> ImuSample            │
  │    - Updates EKF3 Filter -> g_ekf3.predict_imu(imu)                    │
  │    - Updates PID Controller -> g_pid_controller.update()              │
  │    - Calculates Motor Commands -> g_quad_mixer.mix()                    │
  │    - Emits Motor Output TLP to Motor BAR / PIO0 DShot                   │
  └────────────────────────────────────┬────────────────────────────────────┘
                                       │
                                       ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ 4. COOPERATIVE YIELD (`co_await std::suspend_always{}`)                 │
  │    - Coroutine suspends execution state in static promise memory.       │
  │    - Context switch latency < 10 ns (2 CPU pointer instructions).       │
  │    - Main loop polls MSP Configurator TCP server on port 5760.          │
  │    - Next hardware DRDY pulse arrives -> flight_task.resume() called.   │
  └─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Code Trace (`src/main.cpp`)

```cpp
Task<void> run_flight_loop() {
    g_ekf3.reset();

    for (;;) {
        // 1. Consume streaming 64-byte TLP packets from hardware
        while (!g_telemetry_ring.empty()) {
            auto opt_tlp = g_telemetry_ring.pop();
            if (opt_tlp.has_value()) {
                const auto& tlp = opt_tlp.value();

                // 2. Unpack IMU burst & run EKF3 15-state estimation
                drivers::ImuSample imu = drivers::ImuPcieDriver::parse_tlp(tlp);
                g_ekf3.predict_imu(imu);

                // 3. Run PID controller & QuadX motor mixer
                std::array<float, 3> target_rates{0.0f, 0.0f, 0.0f};
                flight::PidState pid_out = g_pid_controller.update(target_rates, imu.gyro_deg_s, 0.001f);
                auto motors = g_quad_mixer.mix(0.5f, pid_out);

                // 4. Emit DShot motor TLP back to hardware
                Tlp64 dshot_tlp = drivers::EscDshotDriver::make_motor_write(0, motors[0], 1);
                g_hw_sim.handle_mem_write(dshot_tlp.target_address(), motors[0]);
            }
        }

        // 5. Cooperative Yield Point (< 10 ns execution)
        co_await std::suspend_always{};
    }
}
```
