# BareCTF Live UDP/TCP Streaming & Motor Mixer Specifications

> [!IMPORTANT]
> **LIVE TELEMETRY & TRACE STREAMING**
> In **`inav-abstractx`**, binary BareCTF trace packets can be streamed **live over Wi-Fi, Ethernet, or Localhost** without blocking flight execution or using dynamic memory:
> - **UDP Broadcast Mode**: Broadcasts 64-byte CTF TLPs on UDP port `19000` (ideal for live Wi-Fi telemetry visualization).
> - **TCP Server Mode**: Listens on TCP port `19000` for guaranteed trace recording.

---

## 1. Live Streaming Architecture

```
+-----------------------------------------------------------------------------------+
|                        iNAV-ABSTRACTX FLIGHT ENGINE                               |
|        - Emits 64B CTF Binary Trace TLPs into SPSC Ring                           |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|               BARECTF LIVE STREAM SERVER (ctf_stream_server.hpp)                  |
|    - Non-blocking UDP Broadcast (Port 19000) / TCP Server                         |
+-----------------------------------------------------------------------------------+
                                         │
        ┌────────────────────────────────┴────────────────────────────────┐
        ▼                                                                 ▼
+------------------------------------+          +------------------------------------+
|  LIVE HOST ANALYSIS TOOLING        |          |  LIVE BLACKBOX CONVERTER           |
|  - babeltrace2 live UDP listener   |          |  - python3 tools/ctf_to_blackbox.py|
|  - Trace Compass Live Stream       |          |  - Output .BBL for Explorer GUI    |
+------------------------------------+          +------------------------------------+
```

---

## 2. C++20 Motor Mixer Architecture (`src/flight/mixer.hpp`)

Motor mixing uses **C++20 Compile-Time Templates** (`abstractx::flight::Mixer<MotorCount>`):

| Airframe Type | Motor Count ($N$) | Preset Matrix | RAM Overhead |
| :--- | :--- | :--- | :--- |
| **Quadcopter X** | $N=4$ | `presets::QuadX` | Exactly 4 motor outputs |
| **PentaPusher Hybrid** | $N=5$ | `presets::PentaPusher` | Exactly 5 motor outputs (4 Quad + 1 Pusher) |
| **Hexacopter X** | $N=6$ | `presets::HexX` | Exactly 6 motor outputs |
| **Octocopter X8** | $N=8$ | `presets::OctoX8` | Exactly 8 motor outputs |

### Why This Beats Legacy iNav / Betaflight Mixers:
1. **Zero RAM Waste**: Allocates an array of *exact* motor count $N$ (`std::array<uint16_t, N>`) rather than hardcoding `MAX_SUPPORTED_MOTORS = 16`.
2. **Zero Unused Loop Iterations**: Matrix math loops iterate strictly over $N$ active motors.
3. **iNav Configurator Preset Compatibility**: Custom rules uploaded over MSP (`MSP_MIXER`) configure the mixer dynamically without recompilation.
