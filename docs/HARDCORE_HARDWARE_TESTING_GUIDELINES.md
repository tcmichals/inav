# Hardcore Hardware Validation & Failure Mode Testing Guidelines

This document provides a comprehensive, rigorous hardware verification and failure mode test specification for the RP2350 Pico 2 W, Linux SBC + FPGA (`rt_offloader`), and SITL target platforms.

---

## 1. Physical Verification Matrix & Acceptance Criteria

| Test Category | Hardware Domain | Test Method & Failure Simulation | Acceptance Criteria / Required Metric |
| :--- | :--- | :--- | :--- |
| **Signal Integrity** | DShot Motors (`GPIO 2..5`) | Oscilloscope / Logic Analyzer probe on motor outputs | DShot150/300/600 bit timing within $\pm 50\text{ ns}$ of spec |
| **Signal Integrity** | Parallel PWM RC (`GPIO 6..11`) | Pulse generator sweep ($800\text{--}2200\ \mu\text{s}$) | Measured pulse width error $< 1.0\ \mu\text{s}$ |
| **Loss of Signal (LOS)** | RC Receiver Link | Disconnect RC receiver wire during $100\%$ throttle demand | System enters Failsafe mode within $\le 100\text{ ms}$ |
| **Sensor Disconnect** | SPI IMU (ICM-42688-P) | Disconnect SPI CS or DRDY line during flight loop | System detects missing samples within $\le 1.0\text{ ms}$, flags `health = false` |
| **Mixer Saturation** | QuadX Motors | Apply $100\%$ Throttle + $100\%$ Roll + $100\%$ Pitch demand | Motor outputs remain bounded in $[1000, 2000]\ \mu\text{s}$ without 16-bit overflow |
| **Sensor Outlier Gate** | BMP280 Barometer | Inject $100\text{ Pa}$ pressure step (ground effect simulation) | EKF3 innovation gate rejects outlier, preserving stable altitude estimate |
| **GPS Glitch Fallback** | GPS Module (UART1) | Simulate HDOP degradation ($> 5.0$) or satellite drop | System transitions from Position Hold to Angle mode with alert |
| **Thermal & Power Stress**| RP2350 SoC & ADC | Run 8 kHz flight loop at $85^\circ\text{C}$ in thermal chamber | 0% loop time overrun ($125\ \mu\text{s}$ period maintained), ADC voltage compensation active |

---

## 2. Hardcore Test Procedures

### Test 1: RC Link Disconnect & Failsafe Stress Test
1. Set throttle stick to $1500\ \mu\text{s}$ (armed state).
2. Physically disconnect the RC receiver signal wire (`GPIO 6` or CRSF `GPIO 6`).
3. Verify that `PwmRc::parse_tlp()` or `Crsf::parse_tlp()` flags `connected = false` and `failsafe = true` within 100 ms.
4. Verify motor outputs immediately transition to minimum command ($1000\ \mu\text{s}$ or DShot `0`).

### Test 2: IMU SPI Bus Interruption & Health Monitoring
1. Run `pico2_hw_test`.
2. Simulate SPI bus disconnect by pulling DRDY line low continuously.
3. Verify `Ekf3Filter::state().is_healthy` transitions to `false`.
4. Verify system logs a critical telemetry event (`BlackboxLogger::log_error()`) to the SPSC ring buffer.

### Test 3: Motor Mixer Saturation & Anti-Windup Boundary Test
1. Input extreme PID commands: $P_{\text{roll}} = 1000$, $P_{\text{pitch}} = 1000$, $P_{\text{yaw}} = 1000$ at $100\%$ throttle ($2000\ \mu\text{s}$).
2. Execute `Mixer<4>::mix()`.
3. Verify all 4 motor outputs are constrained to $\le 2000\ \mu\text{s}$ and $\ge 1000\ \mu\text{s}$.
4. Verify PID integrator anti-windup prevents numerical overflow in `PidState`.

### Test 4: GPS Outlier & Loss of Satellite Fix Test
1. Establish 3D GPS fix lock in `NavigationEngine`.
2. Inject synthetic GPS coordinates jumping $> 100\text{ meters}$ in a single 10 Hz frame.
3. Verify `NavigationEngine` velocity gate flags the jump as invalid and holds position based on optical flow / IMU dead-reckoning.

---

## 3. Automated Command Verification

```bash
cd /home/tcmichals/ssdData/projects/home/inav/build

# 1. Execute 9-suite unit tests
cmake --build . --target run_unit_tests && ./run_unit_tests

# 2. Execute on-device hardware diagnostics
cmake --build . --target pico2_hw_test && ./pico2_hw_test

# 3. Execute 4-subsystem differential parity test
python3 ../tools/compare_inav_parity.py
```
