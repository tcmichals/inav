# Project Requirements & Feature Synthesis Specification
## Merging iNav Navigation & Betaflight Acro Dynamics

## 1. Project Mission & Core Requirements

The core objective of **`inav-abstractx`** is to build a modern, zero-allocation C++20 flight controller that delivers:
1. **Full Autonomous 3D Navigation & Waypoints** (from **iNav**).
2. **High-Speed, Locked-In Acro Stick Response & Low-Latency Filtering** (from **Betaflight**).
3. **Seamless GUI Configuration** via the standard **iNav Configurator** over TCP Port 5760 (or UART).
4. **Zero-Allocation Hardware Abstraction** via the **AbstractX 64-byte TLP virtual bus** and RP2350 triple-PIO offloading.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                    INAV-ABSTRACTX SYNTHESIS                               |
+─────────────────────────────────────────────┬─────────────────────────────────────────────+
| WHAT WE LIFT FROM UPSTREAM INAV             | WHAT WE LIFT FROM UPSTREAM BETAFLIGHT       |
+─────────────────────────────────────────────┼─────────────────────────────────────────────+
| - 3D Autonomous Navigation (RTH/Waypoints)  | - Feedforward 2.0 (Jitter-free stick accel) |
| - Inertial-Complementary Position Estimator | - Anti-Gravity (Throttle-punchout boost)    |
| - Multicopter Braking & Deceleration Curves | - D-Min Dynamic D-Gain (Cooler motors)      |
| - Mahony AHRS with Centrifugal Correction   | - Cascaded PT1/PT2/PT3 & Biquad Filters     |
| - AutoTune (Relay Limit-Cycle Engine)       | - 1st/2nd-Order Gyro Kalman Noise Filter    |
| - EZ-Tune High-Level Parameter Synthesizer  | - RPM Harmonic & Dynamic Notch Filters      |
| - iNav Configurator MSP v1/v2 TCP Server    | - VBat Voltage Sag Compensation             |
+─────────────────────────────────────────────┴─────────────────────────────────────────────+
                                              │
                                              ▼
+───────────────────────────────────────────────────────────────────────────────────────────+
|                       UNIFIED REAL-TIME FLIGHT PIPELINE (1kHz Loop)                       |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 1. SENSOR FILTERING (Betaflight-Grade Noise Suppression)                                  |
|    Raw Gyro -> RPM Harmonic Notch -> Gyro Kalman -> Cascaded PT1/PT2 Low-Pass Filter      |
|                                                                                           |
| 2. STATE ESTIMATION (iNav-Grade Multi-Sensor Fusion)                                      |
|    Filtered Gyro + Accel + Baro + GPS -> INAV Inertial Position & Altitude Estimator      |
|                                                                                           |
| 3. SETPOINT CONTROLLER (Mode Dependent)                                                   |
|    - In Acro / Manual Mode: Betaflight Feedforward 2.0 + ActualRates (Fast stick response)|
|    - In Nav / RTH / Waypoint: iNav 3D Autonomous Velocity & Trajectory Controller         |
|                                                                                           |
| 4. CONTROL LOOP (Betaflight Dynamics + iNav Anti-Windup)                                  |
|    Kp (VBat Comp + TPA) + Ki (Anti-Gravity + Axis Rotation) + Kd (D-Min) + Feedforward    |
|                                                                                           |
| 5. MOTOR MIXER & HARDWARE OFFLOADER (AbstractX C++20)                                     |
|    Mixer<N> -> 64-Byte TLP Virtual Bus -> RP2350 PIO DShot150..1200 (0% CPU Load)         |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 2. Detailed Breakdown of Features Lifted from Upstream INAV

All navigation and estimation modules are ported from `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/`:

| Feature Subsystem | Upstream Reference C File | Architectural Responsibility |
| :--- | :--- | :--- |
| **3D Autonomous Navigation** | `navigation/navigation.c` | Autonomous state machine managing Climb, Turn-to-Home, Waypoint Cruise, Descent, and Emergency Landing. |
| **Waypoint Mission Engine** | `navigation/navigation_waypoint_mission.c` | Loading, sequencing, and executing pre-planned autonomous waypoint missions and geofences. |
| **Multicopter Nav Dynamics**| `navigation/navigation_multicopter.c` | S-curve deceleration profiles, velocity feedforward, and braking distance calculations. |
| **Inertial Position Estimator**| `navigation/navigation_pos_estimator.c` | Multi-sensor complementary fusion (Accel + Baro + GPS + Optical Flow) with dynamic weightings ($w_z, w_{xy}$) and GPS glitch rejection. |
| **Mahony AHRS** | `flight/imu.c` | Quaternion attitude filter with centrifugal acceleration compensation ($\vec{a}_{\text{cent}} = \vec{\omega} \times \vec{v}$) and magnetic declination calculation. |
| **AutoTune Engine** | `flight/pid_autotune.c` | Åström-Hägglund relay limit-cycle oscillation and peak detector for in-flight automatic PID tuning. |
| **EZ-Tune Synthesizer** | `flight/ez_tune.c` | Translating high-level Response, Damping, and Tracking sliders into PID gains and filter frequencies. |
| **iNav Configurator Protocol**| `msp/msp_server.c`, `io/rc_curves.c` | Full MSP v1/v2 server on TCP Port 5760 enabling live tuning, mission planning, and sensor calibration in **iNav Configurator**. |

---

## 3. Detailed Breakdown of Features Lifted from Upstream Betaflight

All high-speed acro dynamics and low-latency noise suppression filters are ported from `/home/tcmichals/ssdData/projects/home/flightcode/betaflight/src/main/`:

| Feature Subsystem | Upstream Reference C File | Architectural Responsibility |
| :--- | :--- | :--- |
| **Feedforward 2.0** | `flight/feedforward.c` | Eliminates stick latency by computing derivative setpoint acceleration with transition smoothing and jitter attenuation (`ff_jitter_factor`). |
| **Anti-Gravity** | `flight/anti_gravity.c` | Injects dynamic I-term boost during rapid throttle changes ($\frac{d(\text{throttle})}{dt}$) to eliminate pitch dip during punchouts. |
| **D-Min Dynamic D Gain** | `flight/pid.c` | Dynamically boosts D gain to $D_{\text{max}}$ only during aggressive stick maneuvers, lowering to $D_{\text{min}}$ during smooth cruise to keep motors cool. |
| **VBat PID Compensation** | `flight/pid.c` | Multiplies master PID gains by $\frac{V_{\text{nominal}}}{V_{\text{measured}}}$ to ensure identical stick feel across the entire battery discharge curve. |
| **Cascaded Filter Pipeline**| `flight/filter.c` | Cascaded PT1, PT2, PT3, and Biquad Direct Form II low-pass and notch filters providing steep high-frequency roll-off with minimal phase delay. |
| **Gyro Kalman Filter** | `flight/kalman.c` | Low-latency 1st/2nd-order scalar Kalman noise filter operating directly on raw gyro samples before PID calculation. |
| **RPM Harmonic Notch Filter**| `flight/rpm_filter.c` | Tracks motor rotational frequency (ERPM) via bidirectional DShot telemetry and dynamically targets 1st, 2nd, and 3rd motor harmonics. |
| **Dynamic Peak Notch Filter**| `flight/dyn_notch.c` | Real-time frequency tracker identifying and notching out mechanical frame resonance peaks. |

---

## 4. Integration into AbstractX C++20 Architecture

```
                                  [ iNav Configurator ]
                                             │
                                     TCP Port 5760 (MSP)
                                             │
                                             ▼
                                  [ ConfigRegistry ]
                                             │
                        ┌────────────────────┴────────────────────┐
                        ▼                                         ▼
           [ Betaflight Rate PID ]                    [ iNav 3D Navigation ]
           • Feedforward 2.0                          • Position Hold
           • Anti-Gravity                             • Waypoints / Safehomes
           • D-Min Dynamic D                          • Multi-stage RTH
           • Cascaded Gyro Filters                    • Inertial Pos Estimator
                        │                                         │
                        └────────────────────┬────────────────────┘
                                             │
                                             ▼
                                    [ Mixer<N> Engine ]
                                             │
                                   64-Byte TLP Virtual Bus
                                             │
                                             ▼
                             [ RP2350 PIO / SITL Hardware ]
```

By unifying **iNav's autonomous navigation suite** with **Betaflight's responsive acro control dynamics**, `inav-abstractx` provides precision autonomous flight combined with locked-in, low-latency pilot stick feel.
