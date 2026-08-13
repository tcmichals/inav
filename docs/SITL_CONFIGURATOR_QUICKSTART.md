# SITL & iNav Configurator Quickstart Guide

> [!IMPORTANT]
> **LIVE HIL / SITL HARDWARE SIMULATION**
> **`inav-abstractx`** includes a built-in Linux SITL (Software-In-The-Loop) simulator engine.
> You can connect the official **iNav Configurator GUI** live to SITL on `localhost:5760`!

---

## 1. How to Build & Launch SITL

```bash
# 1. Navigate to inav repository
cd /home/tcmichals/ssdData/projects/home/inav

# 2. Build SITL binary with CMake
mkdir -p build && cd build
cmake .. && cmake --build .

# 3. Launch SITL Flight Engine
./inav_abstractx_sitl
```

Upon launching, `./inav_abstractx_sitl` will:
- Initialize the Linux Hardware Simulator (emulating continuous ICM-42688-P IMU Auto-DMA bursts and 64-bit nanosecond hardware timestamps).
- Auto-create `config.bin` with default settings if missing.
- Open non-blocking **MSP TCP Server on Port 5760**.

---

## 2. How to Connect iNav Configurator GUI Live

1. Launch official **iNav Configurator** on your computer.
2. In the top-right connection dropdown:
   - Select Connection Type: **TCP**
   - Address: `127.0.0.1` (or `localhost`)
   - Port: `5760`
3. Click **Connect**!

---

## 3. What Works Live in iNav Configurator

| Configurator Tab | Live SITL Capability | MSP Command |
| :--- | :--- | :--- |
| **Setup Tab** | Interactive 3D Quadcopter model rotates live in real-time | `MSP_ATTITUDE` |
| **Sensors Tab** | Real-time Accel, Gyro, and Baro telemetry plots | `MSP_RAW_IMU` |
| **PID Tuning Tab** | Live P, I, D parameter adjustment & D-term filter cutoffs | `MSP_PID`, `MSP_SET_PID` |
| **Receiver Tab** | Live RC channel bars (Roll, Pitch, Yaw, Throttle, Aux) | `MSP_RC` |
| **Motor Tab** | Motor sliders & ESC DShot command output | `MSP_MOTOR` |
| **Save & Reboot** | Persists updated parameters directly to `config.bin` | `MSP_EEPROM_WRITE` |
