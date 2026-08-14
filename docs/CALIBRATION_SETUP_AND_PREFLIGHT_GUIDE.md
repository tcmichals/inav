# Master Calibration, Setup & Pre-Flight Field Guide
## Distilled Knowledge from 10+ Years of INAV, Betaflight & Cleanflight Engineering

> [!IMPORTANT]
> **PURPOSE**: This guide distills over a decade of collective flight engineering, sensor physics, electrical noise mitigation, and field troubleshooting from **INAV** and **Betaflight** into an authoritative reference manual for `inav-abstractx`.

---

## 1. Physical Airframe & Electrical Construction Rules

### 1.1 Sensor Mounting & Vibration Isolation
* **Soft-Mounting the Flight Controller**: High-frequency gyro sensors (e.g. **ICM-42688-P**, **BMI088**) have high acoustic sensitivity to motor stator harmonics (200–600 Hz). Always mount the flight controller on soft silicone rubber anti-vibration standoffs. Tighten screws until rubber barely compresses; never overtighten.
* **Barometer Foam Shielding**: Barometers (e.g. **DPS310**, **BMP280**) use high-precision MEMS pressure diaphragms.
  * **Wind Shielding**: Propeller downwash creates aerodynamic low-pressure vortices that cause severe altitude wobbles.
  * **Optical Shielding**: MEMS silicon is photo-sensitive; direct sunlight shifts pressure readings by up to 5 meters.
  * **Rule**: Place a small cube of **black, open-cell breathable foam** over the barometer chip.
* **Magnetometer (Compass) Placement**:
  * High-current battery wires carry 50–150 Amperes, generating magnetic fields ($B = \frac{\mu_0 I}{2\pi r}$) exceeding the Earth's magnetic field ($~0.5\text{ Gauss}$).
  * **Rule**: Mount the magnetometer/GPS mast **at least 5 to 10 cm elevated away** from ESC power leads, battery wires, and carbon fiber plates.

### 1.2 Electrical Filtering & Low-ESR Capacitors
* **Inductive Voltage Spikes**: When brushless motors brake or switch rapidly via DShot, inductive back-EMF creates voltage spikes (> 35V on a 6S battery) that can brownout the 3.3V MCU LDO regulator.
* **Rule**: Solder a **1000 µF, 35V (or 50V), Low-ESR electrolytic capacitor** (e.g. Rubycon ZLH / Panasonic FM) directly across the main battery power pads on the PDB/ESC.

---

## 2. 6-Point Accelerometer Calibration Protocol

INAV and Betaflight require precise accelerometer zero-bias and scale-factor matrices. Simple 1-point calibration is insufficient for 3D navigation because it only measures the Z-axis.

```
+───────────────────────────────────────────────────────────────────────────────────────────+
| INAV 6-POINT ACCELEROMETER CALIBRATION SEQUENCE                                           |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 1. LEVEL: Place drone flat and level on its landing skids.             (Z = +1.0G)        |
| 2. LEFT: Place drone on its left side.                                 (Y = +1.0G)        |
| 3. RIGHT: Place drone on its right side.                               (Y = -1.0G)        |
| 4. NOSE UP: Point the nose directly up toward the ceiling.             (X = -1.0G)        |
| 5. NOSE DOWN: Point the nose directly down toward the floor.           (X = +1.0G)        |
| 6. UPSIDE DOWN: Place drone upside down on its top plate.              (Z = -1.0G)        |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

* **Mathematical Purpose**: Calculates the $3 \times 3$ gain matrix $[S]$ and offset vector $[\vec{b}]$:
  $$\vec{a}_{\text{corrected}} = [S] (\vec{a}_{\text{raw}} - \vec{b})$$
* **Result**: Guarantees zero gravity bleed into horizontal velocity during banking maneuvers.

---

## 3. 3D Magnetometer (Compass) Calibration

1. Ensure the drone is outdoors or far away from steel desks, computers, and rebar concrete floors.
2. Click **Start Compass Calibration** (in Configurator or Python GUI).
3. **Execute 3D Sphere Rotation (within 30 seconds)**:
   * Rotate 360° along the **Yaw axis** (spin like a plate).
   * Rotate 360° along the **Pitch axis** (tumble nose over tail).
   * Rotate 360° along the **Roll axis** (cartwheel wing over wing).
4. **Validation**: Point the nose toward geographic North (check with phone compass). Verify heading in GUI reads **$0^\circ \pm 3^\circ$**.

---

## 4. RC Transmitter Calibration & Channel Deadbands

### 4.1 Channel Ranges & Midpoints
Ensure your RC transmitter (ExpressLRS, TBS Crossfire, FrSky, Futaba) outputs exact microsecond pulses:

| Channel | Function | Minimum (Low) | Center (Midpoint) | Maximum (High) |
| :--- | :--- | :--- | :--- | :--- |
| **CH 1 (Roll)** | Aileron | **1000 µs** | **1500 µs ($\pm 2\ \mu\text{s}$)** | **2000 µs** |
| **CH 2 (Pitch)** | Elevator | **1000 µs** | **1500 µs ($\pm 2\ \mu\text{s}$)** | **2000 µs** |
| **CH 3 (Throttle)** | Motor Power | **1000 µs** | N/A | **2000 µs** |
| **CH 4 (Yaw)** | Rudder | **1000 µs** | **1500 µs ($\pm 2\ \mu\text{s}$)** | **2000 µs** |
| **AUX 1** | Arming Switch | **1000 µs (DISARM)** | N/A | **2000 µs (ARM)** |

### 4.2 Stick Deadbands
Potentiometer and hall-effect gimbals have small electrical noise ($\pm 1 – 3\ \mu\text{s}$). Without deadband, the I-term will slowly drift:
* `rc_deadband = 3` (Ignores $1497 – 1503\ \mu\text{s}$ on Roll/Pitch).
* `rc_yaw_deadband = 5` (Ignores $1495 – 1505\ \mu\text{s}$ on Yaw).

---

## 5. Comprehensive Arming Prevention Flags Guide

If the drone refuses to arm, check the arming disable flags in the CLI (`status`) or Configurator:

| Arming Flag | Root Cause | Solution |
| :--- | :--- | :--- |
| **`CLI`** | CLI terminal session is active. | Type `exit` in the CLI tab to re-enable arming. |
| **`THROTTLE`** | Throttle stick is above idle threshold ($> 1050\ \mu\text{s}$). | Lower throttle stick to minimum ($1000\ \mu\text{s}$). |
| **`ANGLE`** | Aircraft is tilted beyond `small_angle` limit ($> 25^\circ$). | Place aircraft on level ground before arming. |
| **`ACC_CALIBRATION`** | Accelerometer has not been calibrated. | Run 6-Point Accelerometer Calibration. |
| **`COMPASS_CALIBRATION`** | Magnetometer enabled but uncalibrated. | Run 3D Compass Calibration. |
| **`GPS_FIX_REQUIRED`** | Navigation mode selected, but GPS lacks 3D fix. | Wait for 6+ satellites and GPS 3D fix lock before arming. |
| **`FAILSAFE`** | RC receiver signal lost or failsafe triggered. | Power on transmitter and confirm green link LED. |
| **`HW_FAIL`** | SPI IMU, I2C Baro, or sensor communication failure. | Check wiring harness and power supply voltages. |

---

## 6. Sensor Filtering & PID Tuning Order of Operations

Follow the golden sequence established by Betaflight and INAV tuners:

```
+───────────────────────────────────────────────────────────────────────────────────────────+
| 4-STAGE TUNING SEQUENCE                                                                   |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 1. FILTER TUNING (Vibration Mitigation First)                                             |
|    • Start with conservative PT1 / PT2 low-pass filters (150 Hz gyro, 100 Hz D-term).     |
|    • Enable Dynamic Notch Filter (tracks motor stator RPM harmonic peaks).                |
|    • Lower filter cutoffs if motors are warm; raise cutoffs if latency causes propwash.   |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 2. P & D GAIN BALANCE (D-Min Stabilization)                                               |
|    • Increase P-gain until fast stick snaps are crisp with zero overshoot.                |
|    • Increase D-gain to eliminate bounce-back oscillations on rapid stop.                 |
|    • Verify motor bell temperature after a 30-second hover (Must be < 50°C).              |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 3. I-TERM & ANTI-GRAVITY TUNING (Drift & Punchout Rejection)                              |
|    • Increase I-term until aircraft tracks trajectory through crosswinds without sagging. |
|    • Tune Anti-Gravity gain (`anti_gravity_gain = 3.5`) to eliminate pitch dips on punch. |
+───────────────────────────────────────────────────────────────────────────────────────────+
| 4. FEEDFORWARD 2.0 (Direct Stick Acceleration Response)                                   |
|    • Increase Feedforward (`f_roll`, `f_pitch`) to eliminate stick latency.               |
|    • Enable Feedforward Jitter Suppression to reject RC packet stepped transitions.       |
+───────────────────────────────────────────────────────────────────────────────────────────+
```

---

## 7. Pre-Flight Field Checklist (The 2-Minute Pre-Flight)

Every flight should follow this standardized sequence before takeoff:

1. **Visual & Mechanical Check**:
   * Inspect carbon frame for cracks, loose motor screws, and damaged prop blades.
   * Verify battery strap is tight and balance lead is tucked away from spinning propellers.
2. **Power Up & Link Acquisition**:
   * Turn ON transmitter $\to$ Connect flight battery with smoke stopper / current limiter on bench (or directly at field).
   * Confirm RC link solid (green LED on receiver).
3. **Telemetry & GPS Status**:
   * Confirm 3D fix lock (minimum 6 satellites, HDOP < 1.5).
   * Confirm Home Point latch coordinates in OSD / telemetry.
4. **Arming & Initial Lift-Off**:
   * Place aircraft 5 meters away facing downwind.
   * Arm in **Acro or Angle mode**.
   * Smoothly raise throttle to 30–40% to achieve knee-height hover (0.5 – 1.0 m).
   * Make gentle Roll, Pitch, and Yaw stick pulses to confirm damping.
5. **Post-Flight Inspection**:
   * Disarm immediately upon landing.
   * **Touch motor bells**: If any motor is hot (> 60°C), reduce D-gain and inspect frame for mechanical noise before flying again.
