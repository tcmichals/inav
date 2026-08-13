# Linux Native `epoll` Asynchronous Event Reactor Specification

> [!IMPORTANT]
> **ZERO-DEPENDENCY ASYNCHRONOUS EVENT-DRIVEN HARDWARE REACTOR**
> In Linux, I/O peripherals (`/dev/rpmsg0`, `/dev/ttyS1` Serial UART, `/dev/spidev0.0`, `/dev/i2c-0`) are **file-descriptor event sources**.
>
> Instead of spawning a dedicated CPU thread per I/O bus, **`inav-abstractx`** uses a zero-dependency **Linux `epoll` Event Reactor** ([`linux_epoll_reactor.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_epoll_reactor.hpp)). A single real-time I/O reactor thread monitors all peripheral FDs concurrently using `epoll_wait()`.

---

## 1. Asynchronous Event Reactor Diagram

```
                  Linux Hardware Event Sources (File Descriptors)
      ┌───────────────┬───────────────┬───────────────┬───────────────┐
      │               │               │               │               │
  /dev/rpmsg0     /dev/ttyS1     /dev/spidev0.0   /dev/i2c-0       DRDY GPIO
  (RPMsg VirtIO)  (GPS / CRSF)    (SPI IMU DMA)  (I2C Baro/Mag)    (Interrupt)
      │               │               │               │               │
      └───────────────┴───────┬───────┴───────────────┴───────────────┘
                              │
                              ▼
           Linux Native `epoll_wait()` Event Reactor
           - Single Real-Time POSIX Thread (Priority 98)
           - Sub-microsecond event wakeup latency (< 1 us)
           - Latch 64-bit nanosecond hardware timestamps
                              │
                              ▼
           Pushes 64-byte TLP into Lock-Free SPSC Ring Buffer
                              │
                              ▼
           8 kHz C++20 Flight Loop Coroutine (Core 1)
```

---

## 2. Advantages over Boost.Asio / Libevent / Thread-per-FD

1. **Zero External Dependencies**: Uses native Linux `<sys/epoll.h>` system calls without pulling in heavy C++ libraries (Boost.Asio) or dynamic C shared libraries (`libevent`).
2. **Zero Dynamic Allocation**: Fixed-size static event array (`std::array<struct epoll_event, 16>`) with 0 bytes dynamic heap allocation.
3. **Sub-Microsecond Wakeup Latency**: `epoll_wait()` wakes up instantly when an RPMsg packet or Serial UART frame arrives on file descriptors.
