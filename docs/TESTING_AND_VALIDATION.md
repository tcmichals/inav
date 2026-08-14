# Automated Testing, Validation & Hardware Verification Specification

## 1. Unified Validation Pipeline

To verify flight code integrity, memory safety, protocol parsing, and hardware waveforms, run the master automated validation pipeline:

```bash
cd /home/tcmichals/ssdData/projects/home/inav

# Execute Master Validation Suite (Builds executables, runs unit tests, and validates logic traces)
python3 tools/run_all_validations.py
```

---

## 2. CppUTest Automated Unit Test Harness & Design Rule

> [!IMPORTANT]
> **DESIGN RULE: MANDATORY CPPUTEST COVERAGE**
> Every ported flight control module (Filters, Betaflight Rate PID, Feedforward 2.0, Anti-Gravity, D-Min, Mahony AHRS, INAV Position Estimator, AutoTune, EZ-Tune, Failsafe, Navigation) **MUST** have dedicated CppUTest test groups validating mathematical accuracy, filter attenuation, state transitions, and edge cases.

### Running CppUTest Suites:
```bash
cd /home/tcmichals/ssdData/projects/home/inav

# Build and execute unit tests with color output and verbose details:
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target run_unit_tests -j$(nproc)
./build/run_unit_tests -v -c
```

### CppUTest Group Architecture:
1. **`TEST_GROUP(FilterTests)`**: Verifies PT1, PT2, PT3 step responses, Biquad notch -40dB attenuation at target frequency, and slew rate limits (`DOUBLES_EQUAL`).
2. **`TEST_GROUP(KalmanTests)`**: Verifies 1D & 3D Gyro Kalman noise rejection and dynamic covariance matrix convergence.
3. **`TEST_GROUP(PidControllerTests)`**: Verifies Feedforward 2.0 stick acceleration, Anti-Gravity throttle step boost, D-Min gain scaling, TPA breakpoint attenuation, and I-term 3D coordinate rotation.
4. **`TEST_GROUP(MahonyAhrsTests)`**: Verifies quaternion attitude integration, centrifugal acceleration removal ($\vec{\omega} \times \vec{v}$), and 1.0G gravity vector locking.
5. **`TEST_GROUP(InertialPosEstimatorTests)`**: Verifies 2nd-order Baro/GPS position fusion, GPS glitch rejection gating, and continuous $b_{a,z}$ bias tracking.
6. **`TEST_GROUP(AutoTuneTests)`**: Verifies Åström-Hägglund relay limit-cycle oscillation detection, $T_u$ period extraction, and Ziegler-Nichols gain derivation.
7. **`TEST_GROUP(NavigationTests)`**: Verifies multicopter S-curve deceleration distances, 3D RTH state machine transitions, and Safehome distance selection.
8. **`TEST_GROUP(ProtocolTests)`**: Verifies MSP v1/v2 binary frame roundtrips, CRSF telemetry parser, SBUS channel unpacking, and Spektrum CRC16.
9. **`TEST_GROUP(CoroutineTests)`**: Verifies static promise pool allocation, 0 bytes dynamic heap allocation, and `when_any` / `operator||` timeout resolution.


---

## 3. Physical Signal & Logic Analyzer Verification

For physical hardware verification using **Saleae Logic** or **Kingst Logic Analyzer**:

| Channel | Signal | Expected Protocol / Baud Rate | Waveform Criteria |
| :--- | :--- | :--- | :--- |
| **CH 0** | Motor 1 (GPIO 2) | DShot600 (600 kHz) | Bit period $1.67\ \mu\text{s}$, 16-bit burst width $26.7\ \mu\text{s}$ |
| **CH 1** | Motor 2 (GPIO 3) | DShot600 (600 kHz) | Bit period $1.67\ \mu\text{s}$, 16-bit burst width $26.7\ \mu\text{s}$ |
| **CH 2** | Motor 3 (GPIO 4) | DShot600 (600 kHz) | Bit period $1.67\ \mu\text{s}$, 16-bit burst width $26.7\ \mu\text{s}$ |
| **CH 3** | Motor 4 (GPIO 5) | DShot600 (600 kHz) | Bit period $1.67\ \mu\text{s}$, 16-bit burst width $26.7\ \mu\text{s}$ |
| **CH 4** | RC Receiver (GPIO 8) | CRSF UART @ 420 kBaud | Frame period $4.0\ \text{ms}$ (250 Hz packet rate) |
| **CH 5** | IMU SCK (GPIO 10) | SPI SCK @ 24 MHz | Mode 3 (CPOL=1, CPHA=1), Burst size 15 bytes |
| **CH 6** | IMU DRDY (GPIO 14)| Hardware Interrupt Pulse | Frequency $1.000\ \text{kHz} \pm 0.1\%$ |
| **CH 7** | Status LED (GPIO 15) | WS2812 NRZ Signal | Bit period $1.25\ \mu\text{s}$ ($T_{0H}=0.4\ \mu\text{s}$, $T_{1H}=0.8\ \mu\text{s}$) |

### Automated CSV/VCD Signal Validation:
```bash
python3 tools/validate_logic_trace.py captured_trace.csv
```
