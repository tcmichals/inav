# Flight Dynamics, Filtering & State Estimation Specification

## 1. Sensor Filtering Pipeline

To prevent motor overheating and frame resonance amplification, raw high-frequency gyro data (1kHz–8kHz) passes through a multi-stage, zero-allocation filtering pipeline ([`src/flight/filter.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/filter.hpp), [`src/flight/kalman.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/flight/kalman.hpp)):

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                  SENSOR FILTERING PIPELINE                                |
+───────────────────────────────────────────────────────────────────────────────────────────+
| Raw Gyro (ICM-42688-P / BMI088)                                                           |
|       │                                                                                   |
|       ▼                                                                                   |
| 1. DShot RPM Harmonic Notch Filters (1st, 2nd, 3rd harmonics per motor)                  |
|       │                                                                                   |
|       ▼                                                                                   |
| 2. Dynamic Peak Notch Filter (FFT / Matrix tracking frame resonance)                      |
|       │                                                                                   |
|       ▼                                                                                   |
| 3. Gyro Kalman Filter (1st/2nd-order scalar state-space noise filter)                    |
|       │                                                                                   |
|       ▼                                                                                   |
| 4. Cascaded Low-Pass Filters (PT1 / PT2 / PT3 / Biquad Direct Form II Transposed)         |
|       │                                                                                   |
|       ▼                                                                                   |
| Clean Gyro Rates $\to$ Attitude Estimator & PID Controller Loop                           |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

### Mathematical Formulations:

#### A. PT1 Low-Pass Filter:
$$k = \frac{\Delta t}{\Delta t + \frac{1}{2 \pi f_c}}, \quad y_k = y_{k-1} + k \cdot (x_k - y_{k-1})$$

#### B. Biquad Direct Form II Transposed:
$$y_k = b_0 x_k + s_1, \quad s_1 = b_1 x_k - a_1 y_k + s_2, \quad s_2 = b_2 x_k - a_2 y_k$$

#### C. 1D Gyro Kalman Filter:
$$\hat{x}_{k|k-1} = \hat{x}_{k-1}, \quad P_{k|k-1} = P_{k-1} + Q \cdot \Delta t$$
$$K_k = \frac{P_{k|k-1}}{P_{k|k-1} + R}, \quad \hat{x}_k = \hat{x}_{k|k-1} + K_k (z_k - \hat{x}_{k|k-1}), \quad P_k = (1 - K_k) P_{k|k-1}$$

---

## 2. Flight Dynamics & PID Controller

The rate controller blends **Betaflight acro flight dynamics** with **INAV multi-profile stability**:

```
$$\text{Output} = \text{P\_Term} + \text{I\_Term} + \text{D\_Term} + \text{Feedforward}$$
```

* **Feedforward 2.0 (`feedforward.c`)**: Setpoint derivative acceleration $\frac{d(\text{setpoint})}{dt}$ with transition smoothing to eliminate stick latency.
* **Anti-Gravity (`anti_gravity.c`)**: Dynamic I-term gain multiplier triggered during rapid throttle transients ($\frac{d(\text{throttle})}{dt}$) to prevent nose dip during punchouts.
* **D-Min Dynamic D-Gain**: Linearly scales $K_d$ between $D_{\text{min}}$ during steady cruise and $D_{\text{max}}$ during rapid stick deflection.
* **TPA (Throttle PID Attenuation)**: Scales $K_p$ and $K_d$ above `tpa_breakpoint` throttle to suppress high-throttle oscillations.
* **I-Term Coordinate Rotation**: Rotates the accumulated integral vector in 3D body space as the aircraft rotates, preventing yaw drift during fast rolls.

---

## 3. Inertial-Complementary Position & Altitude Estimator

State estimation uses the **INAV multi-sensor cascaded complementary architecture** (ported from upstream `src/main/navigation/navigation_pos_estimator.c`):

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                        INAV INERTIAL-COMPLEMENTARY POSITION ESTIMATOR                     |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 1. High-Frequency Prediction (1kHz Loop):                                                 |
|    - Accel Earth-frame rotation via Attitude Quaternion: $\vec{a}_{\text{earth}} = R \vec{a}_{\text{body}} - \vec{g}$ |
|    - Centrifugal acceleration removal: $\vec{a}_{\text{corr}} = \vec{a}_{\text{earth}} - (\vec{\omega} \times \vec{v})$ |
|    - Numerical integration: $\vec{p} += \vec{v} \Delta t + \frac{1}{2} \vec{a} \Delta t^2$ |
+───────────────────────────────────────────────────────────────────────────────────────────+
                                              │
                                              ▼
+───────────────────────────────────────────────────────────────────────────────────────────+
| 2. Barometer Altitude Correction (50Hz - 100Hz):                                          |
|    - Innovation: $\text{innov}_z = \text{alt}_{\text{baro}} - (-p_z)$                     |
|    - Altitude update: $p_z -= w_{z,\text{baro}} \cdot \text{innov}_z$                     |
|    - Accelerometer Z-bias continuous tracking: $b_{a,z} += k_b \cdot \text{innov}_z$      |
+───────────────────────────────────────────────────────────────────────────────────────────+
                                              │
                                              ▼
+───────────────────────────────────────────────────────────────────────────────────────────+
| 3. GPS Position & Velocity Correction (5Hz - 10Hz):                                       |
|    - GPS Glitch Gating: Position deviation & HDOP threshold validation                   |
|    - Horizontal Position update: $\vec{p}_{xy} += w_{xy,\text{pos}} (\vec{p}_{\text{gps}} - \vec{p}_{xy})$ |
|    - Horizontal Velocity update: $\vec{v}_{xy} += w_{xy,\text{vel}} (\vec{v}_{\text{gps}} - \vec{v}_{xy})$ |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 4. High-Frequency 8 kHz / 16 kHz Multi-Rate Loop Scheduling

The flight engine supports high-speed loop execution rates up to **16 kHz (62.5 µs period)** for minimum latency acro response:

* **16 kHz (62.5 µs)**: Gyro Kalman noise filter (`kalman.hpp`), PT2/PT3 cascaded low-pass (`filter.hpp`), Betaflight Rate PID (Feedforward 2.0, Anti-Gravity, D-Min), and PIO DShot600/1200 motor output.
* **1 kHz (1.0 ms)**: Accelerometer integration, Mahony quaternion attitude update (`attitude.hpp`), Angle/Horizon auto-level mode, and BareCTF Blackbox logging.
* **100 Hz (10 ms)**: Barometer altitude integration, continuous $b_{a,z}$ bias tracking, and VBat compensation.
* **10 Hz (100 ms)**: GPS position/velocity fusion, 3D waypoint trajectory planning, and RTH state machine.

