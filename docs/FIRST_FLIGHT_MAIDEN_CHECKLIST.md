# First Flight (Maiden) Airframe Testing Protocol

> [!IMPORTANT]
> **CURRENT READINESS STATUS**:
> * **Software & Algorithmic Pipeline**: 100% Complete & Unit Tested (14/14 CppUTest Suites).
> * **Hardware Simulation & Bench Register Testing**: 100% Passed (`pico2_hw_test`).
> * **Physical Airframe Flight Status**: **PENDING PHYSICAL MAIDEN HOVER TEST**.
>
> NEVER attach propellers until all Stage 1 (Bench & Props-Off) tests have passed with 100% certainty.

---

## Stage 1: Bench Inspection & Props-Off Verification (Dry Run)

### 1.1 Visual & Electrical Check (No Battery Connected)
- [ ] Inspect wiring harness against [`docs/PICO2W_WIRING_AND_SETUP.md`](PICO2W_WIRING_AND_SETUP.md).
- [ ] Continuity / Short Circuit Test: Use multimeter on continuity mode between `VSYS` (5V) and `GND`, and between Battery Pad (+) and (-). Ensure **NO short circuits**.
- [ ] Secure flight controller on soft rubber vibration standoffs. Check arrow points forward toward aircraft nose.

### 1.2 USB Power & Configurator Check (No Propellers)
- [ ] Connect USB to PC / phone. Connect to INAV Configurator (`127.0.0.1:5760` on SITL, or `192.168.4.1:5760` on Pico 2 W Wi-Fi AP).
- [ ] **Sensor Health**: Confirm Accelerometer, Gyroscope, Barometer, Magnetometer, and GPS show green status icons.
- [ ] **Attitude Orientation**: Pitch nose UP $\to$ 3D model pitches UP. Roll right wing DOWN $\to$ 3D model rolls RIGHT. Yaw nose RIGHT $\to$ 3D model yaws RIGHT.

### 1.3 RC Transmitter & Channel Range Validation
- [ ] Power on RC transmitter.
- [ ] Verify channel midpoints: Roll = `1500 µs`, Pitch = `1500 µs`, Yaw = `1500 µs`, Throttle = `1000 µs` (minimum).
- [ ] Verify channel endpoints: Full stick deflections reach `1000 µs` (min) and `2000 µs` (max).
- [ ] Verify Arming Switch: Assigned to AUX1. Disarmed = `1000 µs`, Armed = `2000 µs`.

### 1.4 Motor Spin Direction & DShot Protocol Check (PROPELLERS OFF!)
- [ ] Connect flight battery with smoke stopper / current limiter.
- [ ] In Configurator Motors tab: Check "I understand the risks".
- [ ] Spin Motor 1 (Rear Right) slowly: Confirm rotation direction (**Props-In: CCW** / **Props-Out: CW**).
- [ ] Spin Motor 2 (Front Right) slowly: Confirm rotation direction (**Props-In: CW** / **Props-Out: CCW**).
- [ ] Spin Motor 3 (Rear Left) slowly: Confirm rotation direction (**Props-In: CW** / **Props-Out: CCW**).
- [ ] Spin Motor 4 (Front Left) slowly: Confirm rotation direction (**Props-In: CCW** / **Props-Out: CW**).

### 1.5 Failsafe & Disarm Test (PROPELLERS OFF!)
- [ ] Arm the aircraft on the bench at idle throttle.
- [ ] Turn off the RC transmitter.
- [ ] Verify the motors immediately stop spinning (or switch to RTH failsafe descent mode).
- [ ] Turn transmitter back on; verify aircraft cannot re-arm until throttle is lowered to zero.

---

## Stage 2: Tethered Low-Altitude Hover Maiden (Props On)

> [!CAUTION]
> Conduct maiden flight in an open outdoor area or tethered safety rig with eye protection.

### 2.1 Pre-Arming Checks
- [ ] Mount propellers and tighten lock nuts securely.
- [ ] Place aircraft on level ground facing away from pilot.
- [ ] Power transmitter $\to$ Power aircraft.
- [ ] Wait for GPS 3D Fix (6+ satellites) and home position latch.
- [ ] Enable Blackbox logging to flash / UDP.

### 2.2 Maiden Lift-Off (Acro / Angle Mode)
- [ ] Flip Arm switch $\to$ Motors spin at idle throttle (`1050 µs`).
- [ ] Smoothly raise throttle to ~30–40% until aircraft becomes light on skids.
- [ ] Lift off to **0.5 – 1.0 meter (knee height)**.
- [ ] **Immediate Observation**:
  * If aircraft wobbles violently (P-gain oscillation): Disarm immediately.
  * If aircraft flips over instantly: Disarm immediately (motor direction or gyro axis inverted).
  * If hover is stable: Make small Roll, Pitch, and Yaw stick pulses to evaluate response damping.
- [ ] Hover for **20–30 seconds**, then gently land and disarm.

---

## Stage 3: Post-Flight Telemetry & Blackbox Analysis

1. **Motor Temperature Check**:
   * Touch each motor bell immediately after disarming.
   * If motors are cold/warm: PID and D-term filters are safe.
   * If motors are hot (> 60°C): Lower D-gain (`d_roll`, `d_pitch`) and lower gyro filter cutoff (`gyro_lpf1_cutoff_hz`).

2. **Blackbox Log Inspection**:
   * Export Blackbox log via `tools/ctf_to_blackbox.py` to `.BBL`.
   * Open log in **INAV Blackbox Explorer**.
   * Inspect Gyro spectrum (FFT): Verify frame resonance peaks (< 150 Hz) are properly attenuated by the cascaded notch and Kalman filters.
   * Inspect Motor traces: Verify motor outputs are smooth and not saturated against the 100% ceiling.

3. **Incremental Autonomous Mode Flight**:
   * Once Acro and Angle mode hovering are validated, proceed to **Altitude Hold** (Baro PID), **Position Hold** (GPS Nav), and **Return-to-Home (RTH)** testing.
