# Linux Dedicated Thread-Per-Bus Peripheral Architecture Specification

> [!IMPORTANT]
> **DEDICATED POSIX REAL-TIME THREAD PER I/O BUS**
> To isolate hardware peripherals completely, **`inav-abstractx`** supports a **Dedicated Thread-Per-Bus Model** ([`linux_per_bus_thread_manager.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_per_bus_thread_manager.hpp)).
> Each peripheral bus (SPI, I2C, Serial UART, and RPMsg) runs on its own isolated POSIX Real-Time worker thread.

---

## 1. Thread-Per-Bus Allocation Matrix

| Dedicated Thread | Target Hardware Bus | POSIX Priority | Assigned CPU Core | Hardware Responsibilities |
| :--- | :--- | :--- | :--- | :--- |
| **SPI Thread** | `/dev/spidev0.0` | `SCHED_FIFO` **98** | CPU Core 2 (`isolcpus=2`) | High-speed 8 kHz SPI IMU Auto-DMA burst reading |
| **I2C Thread** | `/dev/i2c-0` | `SCHED_FIFO` **97** | CPU Core 2 (`isolcpus=2`) | BMP280 Barometer & QMC5883L Compass I2C transfers |
| **Serial UART Thread** | `/dev/ttyS1` | `SCHED_FIFO` **96** | CPU Core 1 (`isolcpus=1`) | GPS UBX/NMEA 3D fix reading & CRSF RC receiver frames |
| **RPMsg Thread** | `/dev/rpmsg0` | `SCHED_FIFO` **95** | CPU Core 1 (`isolcpus=1`) | RPMsg virtio ring communication to RISC-V co-processor & FPGA |

---

## 2. Dedicated Thread Architecture Diagram

```
                                  Linux ARM Host (Cubie A5E)
                                              │
        ┌───────────────────┬─────────────────┴─────────────────┬───────────────────┐
        ▼                   ▼                                   ▼                   ▼
    SPI Thread          I2C Thread                          Serial Thread       RPMsg Thread
  - Priority 98       - Priority 97                       - Priority 96       - Priority 95
  - Core 2            - Core 2                            - Core 1            - Core 1
  - /dev/spidev0.0    - /dev/i2c-0                        - /dev/ttyS1        - /dev/rpmsg0
        │                   │                                   │                   │
        └───────────────────┴─────────────────┬─────────────────┴───────────────────┘
                                              │ Lock-Free SPSC TLP Ring Buffer
                                              ▼
                                8 kHz C++20 Flight Loop (Core 3)
```
