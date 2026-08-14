# Software-In-The-Loop (SITL) Simulator & 6-DOF Physics Engine

> [!IMPORTANT]
> **TARGET**: Linux Desktop (x86_64 / ARM64) & CI Validation.
> **SOURCE ENTRY POINT**: [`src/target/sitl/sitl_main.cpp`](../src/target/sitl/sitl_main.cpp)
> **PHYSICS ENGINE**: [`src/target/sitl/hardware_simulator.hpp`](../src/target/sitl/hardware_simulator.hpp)
> **MSP TCP PORT**: `5760` (INAV / Betaflight Configurator)
> **BLACKBOX UDP PORT**: `19000` (Live Telemetry Stream)

---

## 1. SITL Simulator Architecture

The `inav-abstractx` SITL simulator executes the full, unmodified C++20 flight control stack in user space on a Linux host. Sensor inputs, motor outputs, and flight dynamics are coupled in a closed-loop real-time coroutine scheduler.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                  INAV-ABSTRACTX SITL PIPELINE                             |
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                                                                           |
|   +──────────────────────────+                  +─────────────────────────────────────+   |
|   |   INAV Configurator      |                  |   Blackbox Explorer (.BBL)          |   |
|   |   (TCP Port 5760)        |                  |   (UDP Port 19000 Live Stream)      |   |
|   +─────────────▲────────────+                  +──────────────────▲──────────────────+   |
|                 │ (MSP v1/v2)                                      │ (BareCTF 64B TLPs)   |
|   +─────────────▼──────────────────────────────────────────────────┴──────────────────+   |
|   |                        SITL Flight Engine (sitl_main.cpp)                         |   |
|   |   • C++20 Stackless Coroutines (`run_flight_loop`, `baro_sensor`, `gps_sensor`)   |   |
|   |   • Betaflight Rate PID (Feedforward 2.0, Anti-Gravity, D-Min, Kalman Filter)     |   |
|   |   • Mahony AHRS Quaternion Attitude Filter & INAV Inertial Position Estimator     |   |
|   |   • 3D Autonomous Waypoint & S-Curve RTH Navigation Engine                        |   |
|   |   • QuadX Motor Mixer (Motors 0..3)                                               |   |
|   +─────────────────────────────┬─────────────────────────────────▲───────────────────+   |
|                                 │                                 │                       |
|                   (Motor PWMs:  │                                 │ (Synthesized IMU      |
|                    1000..2000µs)│                                 │  DMA 64B TLPs)        |
|                                 ▼                                 │                       |
|   +───────────────────────────────────────────────────────────────┴───────────────────+   |
|   |              6-DOF Rigid-Body Multicopter Physics Engine                          |   |
|   |                         (hardware_simulator.hpp)                                  |   |
|   |   • Differential Motor Thrust -> 3-Axis Torques (Roll, Pitch, Yaw)                |   |
|   |   • Rotational Dynamics Integration (alpha = tau / I - d * omega)                 |   |
|   |   • Earth Gravity Vector Projection (Ax, Ay, Az) & Gyro Rates (Gx, Gy, Gz)        |   |
|   +───────────────────────────────────────────────────────────────────────────────────+   |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 2. 6-DOF Multicopter Physics Engine Mathematics

The physics engine in [`src/target/sitl/hardware_simulator.hpp`](../src/target/sitl/hardware_simulator.hpp) calculates closed-loop aerodynamics and inertial dynamics for standard **QuadX** geometry:

```text
                             (Front Left)   (Front Right)
                                 [M3]           [M1]
                                   \             /
                                    \   (CW)    / (CCW)
                                     \         /
                                      \       /
                                       +─────+
                                       |     |
                                       +─────+
                                      /       \
                                     /         \
                                    /   (CCW)   \ (CW)
                                   /             \
                                 [M2]           [M0]
                             (Rear Left)    (Rear Right)
```

### A. Motor Thrust & Torque Generation
Given motor PWM commands $P_i \in [1000, 2000]\,\mu\text{s}$, normalized thrust $T_i \in [0.0, 1.0]$ is computed as:
$$T_i = \frac{P_i - 1000}{1000}$$

The 3-axis aerodynamic torques are derived from the QuadX arm geometry:
* **Roll Torque ($\tau_{\text{roll}}$)** (Left vs Right differential thrust):
  $$\tau_{\text{roll}} = K_{\text{torque}} \cdot \big((T_0 + T_1) - (T_2 + T_3)\big)$$
* **Pitch Torque ($\tau_{\text{pitch}}$)** (Front vs Rear differential thrust):
  $$\tau_{\text{pitch}} = K_{\text{torque}} \cdot \big((T_1 + T_3) - (T_0 + T_2)\big)$$
* **Yaw Torque ($\tau_{\text{yaw}}$)** (Counter-rotating propeller drag balance):
  $$\tau_{\text{yaw}} = K_{\text{yaw}} \cdot \big((T_1 + T_2) - (T_0 + T_3)\big)$$

### B. Angular Rate Integration with Rotational Damping
Rotational acceleration $\vec{\alpha} = [\alpha_x, \alpha_y, \alpha_z]^T$ is computed considering airframe rotational inertia $I$ and aerodynamic damping $d_{\text{aero}}$:
$$\vec{\alpha} = \frac{\vec{\tau}}{I} - d_{\text{aero}} \cdot \vec{\omega}$$

Discrete integration advances the body-frame gyro rates:
$$\vec{\omega}_{k} = \vec{\omega}_{k-1} + \vec{\alpha} \cdot \Delta t$$

### C. Attitude Integration & Accelerometer Gravity Projection
The Euler attitude angles $(\phi, \theta, \psi)$ (Roll, Pitch, Yaw) integrate the angular rates:
$$\phi_k = \phi_{k-1} + \omega_{x,k} \cdot \Delta t$$
$$\theta_k = \theta_{k-1} + \omega_{y,k} \cdot \Delta t$$
$$\psi_k = \psi_{k-1} + \omega_{z,k} \cdot \Delta t$$

Earth gravity ($1G$) is projected into the aircraft's tilted body frame to emulate 3-axis accelerometer outputs:
$$A_x = -\sin(\theta)$$
$$A_y = \sin(\phi) \cos(\theta)$$
$$A_z = \cos(\phi) \cos(\theta) + A_{\text{thrust}}$$

### D. Sensor TLP Stream Generation
The physics engine packs these values into standard **InvenSense ICM-42688-P** 14-byte DMA telemetry packets ($2048\,\text{LSB}/g$ accelerometer sensitivity, $16.4\,\text{LSB}/(\text{deg}/\text{s})$ gyro sensitivity) and injects them into the coroutine telemetry ring buffer.

---

## 3. How to Build & Run the SITL Simulator

### Step 1: Compile the Simulator
```bash
cd /home/tcmichals/ssdData/projects/home/inav
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

### Step 2: Launch the Simulator
```bash
./build/inav_abstractx_sitl
```
*Console Output:*
```text
SITL Flight Engine Running (Linux Desktop) — TCP 5760 — Press Ctrl+C to Stop.
```

---

## 4. Connecting INAV Configurator

1. Open **INAV Configurator** (or Betaflight Configurator).
2. Set connection mode to **TCP**.
3. Enter Address: **`127.0.0.1`** (or `localhost`), Port: **`5760`**.
4. Click **Connect**.
5. **Real-Time Verification**:
   * **Setup Tab**: View live 3D drone model orientation, arming flags, and virtual sensors.
   * **PID Tuning**: Modify Feedforward, Anti-Gravity, and D-Min parameters.
   * **CLI Tab**: Access the interactive configuration terminal (`help`, `status`, `diff`, `set`, `save`).

---

## 5. Automated Python SITL Test Harness (`tools/test_sitl_integration.py`)

You can test the entire SITL simulator and MSP communication protocol programmatically using the automated Python integration harness:

```bash
# 1. Launch SITL in background
./build/inav_abstractx_sitl &

# 2. Run the automated Python test suite
python3 tools/test_sitl_integration.py
```

*Console Output:*
```text
============================================================
 INAV-ABSTRACTX SITL PYTHON TEST HARNESS
============================================================
Connected to SITL Simulator on 127.0.0.1:5760

[TEST 1/4] MSP_FC_VARIANT: 'INAV' ... PASSED!
[TEST 2/4] MSP_API_VERSION: Proto=2, API=2.0 ... PASSED!
[TEST 3/4] MSP_ATTITUDE: Roll=+0.0 deg, Pitch=+0.0 deg, Yaw=+0.0 deg ... PASSED!
[TEST 4/4] MSP_RAW_GPS: Fix=1 (3D), Sats=12, Lat=37.774900, Lon=-22.419400, Alt=10m, HDOP=1.20 ... PASSED!

============================================================
 ALL SITL PYTHON INTEGRATION TESTS PASSED 100%!
============================================================
```

---

## 6. Live Blackbox Telemetry Capture & Replay

While SITL is executing, stream real-time flight telemetry into an INAV Blackbox Explorer `.BBL` log file:

```bash
# Capture live UDP stream from SITL
python3 tools/ctf_to_blackbox.py --live --port 19000 -o live_sitl_flight.bbl
```

Open `live_sitl_flight.bbl` in **INAV Blackbox Explorer** to analyze PID step responses, gyro noise spectrum, and motor mixer traces.

---

## 7. External Simulator Coupling (Gazebo / FlightGear / RealFlight)

To couple `inav-abstractx` with external 3D visual simulators:
* **Actuator Output**: Read motor PWM values via the lock-free logging ring or UDP socket.
* **Sensor Feedback**: Inject ground-truth state ($x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \psi, p, q, r$) from Gazebo into `HardwareSimulator::step()`.

