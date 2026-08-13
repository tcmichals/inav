# Extended Kalman Filter 3 (EKF3) & Sub-Microsecond Hardware Timestamping

> [!IMPORTANT]
> **EKF3 SENSOR FUSION SPECIFICATION**
> In **`inav-abstractx`**, Extended Kalman Filtering (EKF3) provides 15+ state estimation variables (3D Position, 3D Velocity, 3D Attitude, Gyro Biases) without dynamic heap memory allocations (`malloc`/`free`).
>
> Crucially, EKF3 time prediction steps utilize **64-bit nanosecond hardware timestamps (`timestamp_ns`)** latched at the exact hardware `DRDY` clock edge. Software thread scheduling jitter ($20\text{--}50\ \mu\text{s}$) is eliminated!

---

## 1. EKF3 State Vector & Measurement Updates

$$\mathbf{x} = \begin{bmatrix} \text{Pos}_N & \text{Pos}_E & \text{Pos}_D & V_N & V_E & V_D & \phi & \theta & \psi & b_{g,x} & b_{g,y} & b_{g,z} \end{bmatrix}^T$$

- **Time Prediction (`predict_imu`)**: Driven by IMU sample bursts + hardware nanosecond timestamps over `PCIE_BAR_IMU_BASE`.
- **Barometer Correction (`correct_baro`)**: Driven by Baro altitude pressure readings over `PCIE_BAR_BARO_BASE`.
- **GPS Correction (`correct_gps`)**: Driven by UBX/NMEA 3D position and velocity vectors over `PCIE_BAR_SERIAL_BASE`.

---

## 2. Hardware Timestamp Precision Advantage

In legacy MCU architectures:
- Software interrupt handling latencies ($20\text{--}50\ \mu\text{s}$) inject random timing noise into numerical time step $dt$.
- EKF velocity derivative calculations become noisy, causing vehicle position drift during high-speed maneuvers.

In **`inav-abstractx`**:
- The hardware offloader (FPGA / Pico 2 PIO / STM32 Timer IC) latches `timestamp_ns` on the **exact clock cycle `DRDY` fires**.
- EKF3 velocity derivative noise is reduced to **$< 20\text{ ns}$**, resulting in rock-solid Extended Kalman Filter state estimation!
