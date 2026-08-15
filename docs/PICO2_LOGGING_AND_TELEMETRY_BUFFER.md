# Pico 2 W Pre-Connect Logging & TCP Telemetry Offload Architecture

> **Document Status**: Production Specification & Verification Standard  
> **Target Hardware**: Raspberry Pi Pico 2 W (RP2350 Dual Cortex-M33 @ 150 MHz + Infineon CYW43439 802.11n Wi-Fi)  
> **Primary Source References**:  
> - [`src/target/pico2_rp2350/pico2_main.cpp`](../src/target/pico2_rp2350/pico2_main.cpp)  
> - [`src/target/pico2_rp2350/lwip_msp_transport.hpp`](../src/target/pico2_rp2350/lwip_msp_transport.hpp)  
> - [`include/spsc_tlp_ring.hpp`](../include/spsc_tlp_ring.hpp)  
> - [`include/asp_tlp64.hpp`](../include/asp_tlp64.hpp)  

---

## 1. Executive Summary & Problem Statement

When the Pico 2 W powers on:
1. **Core 0** initializes peripheral buses (SPI1, I2C1, UART0), loads `MasterConfig` from on-chip flash, and begins CYW43439 Wi-Fi link association.
2. **Core 1** boots immediately into the hard real-time flight loop, executing sensor discovery (`WHO_AM_I` checks), soft resets, register configuration, PROM calibration coefficient readouts (BMP280/DPS310/MS5611), gyro bias calculation, and pre-arm sanity tests.
3. **Connection Latency**: An external Ground Control Station (GCS), Configurator, or remote logging server attaches **2 to 10 seconds post-boot** due to 802.11 association, WPA2/WPA3 4-way handshakes, and DHCP address negotiation.

To prevent dropping early boot diagnostics, calibration tables, and initial sensor baselines, the system incorporates an **in-memory lock-free SPSC retention ring buffer (`SpscTlpRing<1024u>`)**.

---

## 2. Exact Boot Data Volume Breakdown

During boot, the flight engine generates two distinct classes of records: **discrete initialization records** (critical, non-evictable) and **periodic sensor telemetry** (rate-decimated).

| Log Category | Number of 64B TLPs | SRAM Footprint | Content & Mathematical Fields |
| :--- | :--- | :--- | :--- |
| **System Boot & Flash Config** | 2 TLPs | 128 Bytes | CPU clock (150 MHz), flash sector checksum, firmware Git hash, arming flags. |
| **SPI1 & I2C1 Bus Probe** | 2 TLPs | 128 Bytes | Hardware bus clock frequencies, GPIO pin routing validation. |
| **ICM-42688P / BMI088 IMU Init** | 4 TLPs | 256 Bytes | WHO_AM_I register verification, AAF filter coefficients ($\delta=6$), INT1 DRDY routing. |
| **BMP280 / DPS310 Baro Init & Calib** | 3 TLPs | 192 Bytes | Complete 24-byte Bosch PROM readout (`dig_T1..dig_P9`) or 18-byte Infineon MRCL coefficients. |
| **QMC5883L / IST8310 Mag Init** | 2 TLPs | 128 Bytes | Chip ID verification, 200 Hz continuous mode, SET/RESET period registers. |
| **Gyro & Accel Bias Zeroing** | 4 TLPs | 256 Bytes | Pre-flight 1000-sample mean bias offsets ($X_0, Y_0, Z_0$) and sensor noise floor variance. |
| **Pre-Arm Status & EKF Convergence** | 3 TLPs | 192 Bytes | AHRS quaternion convergence, baro sea-level reference, arming disable flags. |
| **Total Discrete Boot Diagnostics** | **20 TLPs** | **1,280 Bytes** | **All critical init & calibration tables require only ~1.3 KB!** |

---

## 3. RP2350 SRAM Allocation & Buffer Size Options

The RP2350 microcontroller provides **520 KB of on-chip SRAM** across 10 independent SRAM banks (Bank 0–7: 64 KB each = 512 KB, Bank 8–9: 4 KB each = 8 KB).

| Subsystem / Allocation | Memory Footprint (RAM) | Flash / ROM | Memory Notes |
| :--- | :--- | :--- | :--- |
| **Static Coroutine Frame Pool & Tasks** | 8.2 KB | 0 KB | Zero dynamic allocations (`static_pool_executor`) |
| **InavImu (Mahony AHRS) + PosEstimator** | 4.8 KB | 0 KB | Attitude quaternions & kinematic state matrices |
| **Gyro Spectral Analyzer (64-pt FFT)** | 2.1 KB | 0 KB | Ping-pong sample buffers & Hanning window table |
| **PID Controller & Filter Banks** | 3.4 KB | 0 KB | PT1/PT2/PT3, Biquad notch, and DShot RPM notch banks |
| **lwIP NO_SYS Raw TCP/IP Stack** | 18.5 KB | 0 KB | `TCP_MSS=1460`, `TCP_WND=4096`, 4 PCBs |
| **Core 0 & Core 1 Stacks** | 16.0 KB | 0 KB | 8 KB per core |
| **Core Flight Engine Baseline Total** | **~53.0 KB** | **~128 KB** | Base flight software footprint |
| **Telemetry & Boot Retention Ring (`1024u`)**| **65.5 KB** | **0 KB** | **1024 $\times$ 64B TLP frames (65,536 Bytes in dedicated Bank 1)** |
| **Unallocated SRAM Headroom** | **> 400 KB** | **3.8+ MB** | **> 77% SRAM remaining** |

---

## 4. Time Window Coverage vs. Buffer Sizing

After storing the discrete 20 initialization TLPs (1.3 KB), the remaining slots buffer periodic continuous flight telemetry frames while awaiting TCP socket attachment:

| Buffer Capacity (`SpscTlpRing<N>`) | SRAM Footprint | At 10 Hz Telemetry | At 50 Hz Telemetry | At 100 Hz Telemetry |
| :--- | :--- | :--- | :--- | :--- |
| **512 TLPs** | **32 KB** | **49.2 seconds** | **9.8 seconds** | **4.9 seconds** |
| **1024 TLPs** *(Standard)* | **64 KB** | **100.4 seconds** | **20.0 seconds** | **10.0 seconds** |
| **2048 TLPs** | **128 KB** | **202.8 seconds** | **40.5 seconds** | **20.2 seconds** |

---

## 5. Pre-Connect Buffering & Offload Pipeline

```
┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                                   CORE 1 (Flight Loop)                                     │
│  - Sensor Boot Events (WHO_AM_I, Reset, Register Config)                                   │
│  - Calibration Coefficients Readout (dig_T1..dig_P9, c0..c30, PROM C1..C6)                 │
│  - Gyro & Accel Zero-Bias Offsets & Thermal Baselines                                      │
│  - Continuous 50 Hz Decimated Flight Telemetry                                             │
└─────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                              │
                                   Push 64-Byte TLP Packets
                                              │
                                              ▼
┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                   LOCK-FREE SPSC RETENTION RING (`SpscTlpRing<1024u>`)                     │
│  - Capacity: 1024 Frames (65,536 Bytes in dedicated SRAM Bank 1)                           │
│  - Atomic Read/Write Pointers (Zero mutexes, zero core-to-core lock contention)             │
│  - Pre-Connect Mode: Accumulates all boot records while TCP socket is disconnected         │
└─────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                              │
                                  Drained upon TCP Handshake
                                              │
                                              ▼
┌────────────────────────────────────────────────────────────────────────────────────────────┐
│                             CORE 0 (lwIP Raw TCP Server)                                   │
│  1. Ground station connects via TCP Port 5760 (MSP) / 5761 (Log Stream)                    │
│  2. `on_accept_cb()` fires: Server enters Backlog Drain Phase                              │
│  3. Flushes entire retained 1024-frame buffer sequentially via `tcp_write()` / `tcp_output()`│
│  4. Enters Live Streaming Phase: Streams real-time flight telemetry with 0 init loss       │
└────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. TCP Dump Timing & Network Performance

When the client connects (or Pico connects to the server), the Core 0 drain loop executes:

- **Drain Rate**: 16 TLPs per 1 ms loop tick ($16 \times 64\,\text{B} = 1,024\,\text{Bytes/ms} = 1.024\,\text{MB/s}$).
- **512 TLPs (32 KB)** dumps completely in **32 ms** (under 3 TCP packets over 802.11n Wi-Fi).
- **1024 TLPs (64 KB)** dumps completely in **64 ms** (under 6 TCP packets).
- **Network Pacing**: lwIP NO_SYS raw API maintains `TCP_WND = 4096` and `TCP_MSS = 1460`, pacing bursts to prevent wireless link congestion.

---

## 7. Dual Connection Topologies Supported

### 7.1 Pico 2 W as TCP Server (Default)
- Pico 2 W listens on port `5760` (MSP) and port `5761` (CTF/TLP Binary Stream).
- Client (GCS / Configurator) connects to Pico's IP address.
- `on_accept_cb()` triggers immediate buffer flush.

### 7.2 Pico 2 W as TCP Client (Auto-Offload)
- Pico 2 W connects directly to configured telemetry host (`target_ip:port`, e.g. GCS running on laptop/SBC).
- `on_connected_cb()` triggers immediate buffer flush upon TCP handshake completion.

---

## 8. Ring Buffer Retention Policies & Edge Cases

1. **Discrete Boot Frame Protection**:
   - Discrete initialization TLPs (tags `0x00..0x1F`) are marked non-evictable.
   - If Wi-Fi connection takes > 30 seconds, only the oldest periodic continuous telemetry records are discarded, guaranteeing that calibration tables, WHO_AM_I responses, and fault codes are 100% preserved.
2. **Crash & Panic Preservation**:
   - On hardware fault / hard-fault handler, the memory region of `g_logging_ring` in SRAM Bank 1 is retained across soft resets without being cleared by the bootloader.
