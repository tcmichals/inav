# TCP vs UDP Network Telemetry Transport Specification

> [!IMPORTANT]
> **TELEMETRY TRANSPORT PROTOCOL SELECTION**
> In **`inav-abstractx`**, telemetry and configuration networking is partitioned between **TCP** and **UDP** based on data reliability requirements ([`ctf_stream_server.cpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/logging/ctf_stream_server.cpp)).

---

## 1. Network Protocol Selection Matrix

| Protocol | Assigned Port | Primary Use Case | Reliability | Latency & Headroom Impact |
| :--- | :--- | :--- | :--- | :--- |
| **TCP** | `5760` | **iNav Configurator MSP CLI & Tuning** | $100\%$ Guaranteed Delivery | Connection-oriented; retransmission delays if link drops |
| **UDP** | `19000` | **High-Speed Live CTF/BBL Telemetry Streaming** | Best-Effort Datagram | **Sub-microsecond (< 1 us)**; zero TCP retransmission blocking |

---

## 2. Technical Characteristics

1. **iNav Configurator (TCP 5760)**: Uses TCP because writing configuration settings (`save`, `dump`) requires guaranteed in-order packet delivery without corrupted bytes.
2. **Live Telemetry & Blackbox Streaming (UDP 19000)**: Uses UDP because streaming 8 kHz IMU / EKF3 state data over Wi-Fi requires lowest possible latency. Dropped datagrams are skipped without stalling the flight controller loop.
