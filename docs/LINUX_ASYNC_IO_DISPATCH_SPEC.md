# Linux Asynchronous Non-Blocking Hardware I/O Dispatch Specification

> [!IMPORTANT]
> **PARALLEL NON-BLOCKING HARDWARE DISPATCHING ON LINUX**
> Linux `spidev` (`/dev/spidev0.0`) and `i2c-dev` (`/dev/i2c-0`) `ioctl()` calls are synchronous blocking kernel operations ($50\text{--}200\ \mu\text{s}$ delay).
>
> **`inav-abstractx`** prevents kernel blocking by submitting 64-byte TLP requests to an asynchronous submission queue ([`linux_async_io_dispatcher.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_async_io_dispatcher.hpp)). Core 1 (Flight Loop) submits TLP requests in **$< 10\text{ ns}$** while background I/O worker threads / `io_uring` execute SPI, I2C, and UART transfers concurrently in parallel!

---

## 1. Parallel Hardware Dispatch Diagram

```
                             8 kHz C++20 Flight Loop (Core 1)
                                            │
                      Submits TLP Writes to Lock-Free SPSC Ring (< 10 ns)
                                            │
                     ┌──────────────────────┼──────────────────────┐
                     │                      │                      │
                     ▼                      ▼                      ▼
           SPI Worker Thread       I2C Worker Thread       UART Worker Thread
         (/dev/spidev0.0)           (/dev/i2c-0)            (/dev/ttyS1)
         ┌────────────────┐        ┌────────────────┐      ┌────────────────┐
         │ Executess SPI   │        │ Executes I2C   │      │ Executes UART  │
         │ IMU Auto-DMA   │        │ Baro / Compass │      │ GPS/CRSF Read  │
         └───────┬────────┘        └───────┬────────┘      └───────┬────────┘
                 │                         │                       │
                 └─────────────────────────┼───────────────────────┘
                                           │
                        Pushes Result TLPs into g_telemetry_ring
                                           │
                                           ▼
                             8 kHz C++20 Flight Loop (Core 1)
```

---

## 2. Technical Characteristics

1. **Zero Flight Loop Overheads**: Core 1 submits TLP write requests into `g_command_ring` in $< 10\text{ ns}$ and immediately resumes math processing. It never blocks on Linux `ioctl(SPI_IOC_MESSAGE)` or `ioctl(I2C_RDWR)` calls.
2. **Parallel Device Dispatch**: SPI IMU, I2C Barometer, I2C Compass, and UART GPS transfers occur simultaneously on independent kernel I/O queues.
3. **Hardware Timestamps (`timestamp_ns`)**: Background I/O threads latch 64-bit nanosecond hardware timestamps at the exact instant the `DRDY` GPIO edge triggers, preserving nanosecond EKF3 time accuracy.
