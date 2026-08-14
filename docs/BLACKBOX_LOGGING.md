# High-Speed Blackbox Logging & BareCTF Streaming Specification

## 1. Zero-CPU BareCTF Architecture

Traditional flight controllers incur 15%–25% CPU overhead during flight to serialize, bit-pack, and write variable-length Blackbox records to SPI flash or SD cards.

In `inav-abstractx`, blackbox logging is implemented via **fixed 64-byte BareCTF Transaction Layer Packets** streamed over a dedicated lock-free ring buffer ([`src/logging/ctf_stream_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/ctf_stream_server.hpp)):

* **Zero Bit-Packing in Flight Loop**: Raw IMU, PID terms, motor outputs, and setpoints are copied in a single 64-byte `memcpy` into `g_logging_ring`.
* **Zero Preemption / Zero IRQ Jitter**: Flight loop execution time is unaffected by logging status.
* **Stream Transport**: Core 0 streams binary packets over UDP Port `19000` (or saves to SPI Flash).

---

## 2. 64-Byte Blackbox TLP Packet Layout

```
Byte 0..3:   Event ID / Magic Header (`0x43544631` -> "CTF1")
Byte 4..11:  64-bit Hardware Nanosecond Timestamp (`timestamp_ns`)
Byte 12..23: 3D Filtered Gyro Rates (Roll, Pitch, Yaw in deg/s)
Byte 24..35: 3D Accelerometer Acceleration (X, Y, Z in Gs)
Byte 36..47: 3D PID Output Terms (P_out, I_out, D_out, Feedforward)
Byte 48..55: 4x Motor Output Setpoints (1000..2000 µs / DShot words)
Byte 56..59: Battery Voltage (mV) & Current Draw (cA)
Byte 60..63: Sequence Number & Checksum
```

---

## 3. Blackbox Explorer Integration (`tools/ctf_to_blackbox.py`)

The python converter ([`tools/ctf_to_blackbox.py`](file:///home/tcmichals/ssdData/projects/home/inav/tools/ctf_to_blackbox.py)) converts 64-byte BareCTF telemetry streams into standard **Betaflight / iNav `.BBL` files** compatible with **Blackbox Explorer**:

### Mode A: Live Real-Time UDP Stream Capture
Capture live telemetry streamed over Wi-Fi (Pico 2 W) or local UDP (Linux SITL) directly to a `.bbl` file:

```bash
cd /home/tcmichals/ssdData/projects/home/inav

# Listen on UDP port 19000 and record live to flight_log.bbl:
python3 tools/ctf_to_blackbox.py --live --port 19000 flight_log.bbl
```

### Mode B: Offline Flash Dump File Conversion
Convert raw binary dumps extracted from on-board SPI Flash or disk logs:

```bash
# Convert offline binary CTF dump to standard .BBL file:
python3 tools/ctf_to_blackbox.py flash_dump.bin flight_log.bbl
```

---

## 4. Opening Logs in iNav / Betaflight Blackbox Explorer

1. Download and launch the official **[Betaflight Blackbox Explorer](https://github.com/betaflight/blackbox-log-viewer)** or **[iNav Blackbox Explorer](https://github.com/iNavFlight/inav-blackbox-log-viewer)** GUI.
2. Click **"Open log file/video"** and select `flight_log.bbl`.
3. Inspect gyro rates, PID error curves, D-term noise frequencies, motor commands, and battery voltage in full high-resolution graph views.

