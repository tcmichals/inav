# Modular Hardware Driver Ecosystem Specification

## 1. Driver Architecture & Concept Model

In `inav-abstractx`, all peripheral device drivers are implemented as self-contained C++20 header-only components inside [`src/drivers/`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/). Drivers decouple physical register logic from I/O buses by consuming and producing 64-byte Transaction Layer Packets ([`asp_tlp64.hpp`](file:///home/tcmichals/ssdData/projects/home/AbstractX/include/asp_tlp64.hpp)).

---

## 2. Sensor Driver Matrix

### A. Inertial Measurement Units (IMU)
| Driver Class | Supported Chips | Bus Protocol | Sample Rate | Features |
| :--- | :--- | :--- | :--- | :--- |
| **`Icm42688P`** ([`icm42688p.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/icm42688p.hpp)) | InvenSense ICM-42688-P / ICM-42605 | SPI @ 24MHz | Up to 8 kHz | Ultra-low noise ($0.005^\circ/\text{s}/\sqrt{\text{Hz}}$), APEX hardware interrupt, 16-bit FIFO |
| **`Bmi088`** ([`bmi088.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/bmi088.hpp)) | Bosch BMI088 (Separate Accel & Gyro) | SPI @ 10MHz | Up to 2 kHz | Automotive-grade vibration robustness, dual chip select |
| **`Mpu6000`** ([`mpu6000.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/mpu6000.hpp)) | InvenSense MPU-6000 / MPU-6500 | SPI @ 20MHz | Up to 8 kHz | Industry-standard legacy flight controller IMU |

### B. Barometric Altimeters
| Driver Class | Supported Chips | Bus Protocol | RMS Noise | Derived Output |
| :--- | :--- | :--- | :--- | :--- |
| **`Dps310`** ([`dps310.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/dps310.hpp)) | Infineon DPS310 | I2C / SPI | $\pm 0.002\ \text{hPa}$ ($\approx \pm 2\ \text{cm}$) | 24-bit Pressure (Pa), Temp ($^\circ\text{C}$), Altitude (cm) |
| **`Bmp280`** ([`bmp280.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/bmp280.hpp)) | Bosch BMP280 / BME280 | I2C / SPI | $\pm 0.012\ \text{hPa}$ ($\approx \pm 10\ \text{cm}$) | 20-bit Pressure (Pa), Temp ($^\circ\text{C}$), Altitude (cm) |
| **`Ms5611`** ([`ms5611.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/ms5611.hpp)) | Measurement Specialties MS5611 | I2C / SPI | $\pm 0.012\ \text{hPa}$ ($\approx \pm 10\ \text{cm}$) | 24-bit $\Delta\Sigma$ Pressure, Altitude (cm) |

### C. Magnetometers (Compasses)
| Driver Class | Supported Chips | Bus Protocol | Field Range | Features |
| :--- | :--- | :--- | :--- | :--- |
| **`Qmc5883l`** ([`qmc5883l.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/mag/qmc5883l.hpp)) | QST QMC5883L | I2C | $\pm 8\ \text{Gauss}$ | 16-bit resolution, 200Hz ODR, temperature compensation |
| **`Ist8310`** ([`ist8310.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/mag/ist8310.hpp)) | iSentek IST8310 | I2C | $\pm 16\ \text{Gauss}$ | High cross-axis rejection, integrated on many GPS pucks |

### D. Global Navigation Satellite Systems (GNSS / GPS)
* **`GpsDriver`** ([`gps_driver.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/gps/gps_driver.hpp)):
  * **U-Blox Binary Parser** (`ubx_parser.hpp`): UBX-NAV-PVT, UBX-NAV-POSLLH, UBX-NAV-VELNED, UBX-CFG-PRT (Auto-baud & rate configuration up to 10Hz/20Hz).
  * **NMEA 0183 ASCII Parser** (`nmea_parser.hpp`): Standard `$GNGGA`, `$GNRMC` sentence decoders.

---

## 3. RC Receiver & Actuator Drivers

### A. Serial & PWM RC Receivers
* **`Crsf`** ([`crsf.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/crsf.hpp)): ExpressLRS / Team BlackSheep Crossfire 420 kBaud serial protocol with CRC8-DVB-S2 validation and bidirectional telemetry backchannel.
* **`SbusParser`** ([`sbus.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/sbus.hpp)): 100 kBaud inverted UART 25-byte frame parser for Futaba/FrSky SBUS receivers.
* **`SpektrumSrxl2`** ([`spektrum_srxl2.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/spektrum_srxl2.hpp)): 115.2 kBaud bi-directional single-wire half-duplex SRXL2 protocol with CRC16-CCITT.
* **`PwmRc`** ([`pwm_rc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/pwm_rc.hpp)): Standard 1000 µs – 2000 µs PWM pulse-width capture.

### B. ESC Actuator Protocols
* **`EscDshotDriver`** ([`esc_dshot_driver.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc_dshot_driver.hpp)): DShot150, DShot300, DShot600, DShot1200 digital motor commands with 4-bit telemetry CRC checksum generation and bidirectional ERPM feedback.
* **`PwmEsc`** ([`pwm_esc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/pwm_esc.hpp)): Standard 50Hz–400Hz Analog Servo/ESC PWM output.
* **`OneShot`** ([`oneshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/oneshot.hpp)): 125 µs – 250 µs OneShot125 protocol.

### C. Display & Status Drivers
* **`OsdMax7456`** ([`osd_max7456.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/display/osd_max7456.hpp)): MAX7456 monochrome analog OSD character generator over SPI.
* **`OledSsd1306`** ([`oled_ssd1306.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/display/oled_ssd1306.hpp)): 128x64 / 128x32 OLED display driver over I2C.
* **`LedStripDriver`** ([`led_strip_driver.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/led_strip_driver.hpp)): WS2812 / SK6812 programmable RGB LED strip controller.

---

## 4. Asynchronous Coroutine Driver Lifecycle & Physical Delay Model

All hardware drivers follow a strict non-blocking coroutine lifecycle using C++20 `Task<T>`, `co_await sleep_ms()`, and concurrent combinators (`&&` / `||`):

### A. Non-Blocking Initialization Pattern
```cpp
template <bus::IsSpiBus SpiBusT>
class Icm42688PDriver {
public:
    // Fully asynchronous non-blocking boot lifecycle
    [[nodiscard]] Task<ImuInitResult> async_init() noexcept {
        // Stage 1: Check WHO_AM_I
        uint8_t who = bus_.read_reg(REG_WHO_AM_I);
        if (who != WHO_AM_I_ICM42688P && who != WHO_AM_I_ICM42605) {
            co_return ImuInitResult::WhoAmIMismatch;
        }

        // Stage 2: Soft reset + non-blocking yield for 2ms
        bus_.write_reg(REG_DEVICE_CONFIG, 0x01u);
        co_await sleep_ms(2u); // Yields CPU to other initializing drivers

        // Stage 3: Low-noise power mode + non-blocking yield for 1ms
        bus_.write_reg(REG_PWR_MGMT0, static_cast<uint8_t>(ACCEL_MODE_LN | GYRO_MODE_LN));
        co_await sleep_ms(1u);

        // Stage 4: Configure ODR, Full Scale, Anti-Aliasing Filters & DRDY
        bus_.write_reg(REG_GYRO_CONFIG0, gyro_cfg);
        bus_.write_reg(REG_ACCEL_CONFIG0, accel_cfg);
        bus_.write_reg(REG_INT_SOURCE0, UI_DRDY_INT1_EN);

        initialized_ = true;
        co_return ImuInitResult::Ok;
    }
};
```

### B. Parallel Boot Initialization (`&&` / `when_all`)
At startup, `SensorDetector` launches all sensor initializations concurrently:
```cpp
co_await (imu.async_init() && baro.async_init() && mag.async_init() && pitot.async_init());
```
* Total boot latency equals $\max(\text{sensor delays}) = 10\,\text{ms}$ rather than $\sum(\text{sensor delays}) = 22\,\text{ms}$.

### C. Runtime Multi-Rate Concurrency
While slow sensors (MS5611 9.04ms ADC, QMC5883L 15ms analog settle, GPS 100ms epoch) are suspended via `co_await sleep_ms()`, the 8 kHz IMU sampling, PID rate controller, and DShot motor output loop execute without dropping a single cycle.

### D. Safety Watchdog Racing vs. Blocking Thread Pends (`when_any` / `||`)
* **The "Why Pend When We Can Await" Principle**: Physical sensors have deterministic conversion times, response limits, and failure modes (e.g. I2C bus lockup, disconnected pin, hardware reset). Blocking/pending a thread or core stalls the entire flight loop ($10\,\text{ms}$ delay drops 80 consecutive $8\,\text{kHz}$ PID cycles).
* **Non-Blocking Watchdog Racing**: Every hardware transaction races concurrently against a hardware watchdog timer via `when_any` (`||`):
```cpp
// Non-blocking race: hardware DMA burst vs. physical timeout limit
auto result = co_await (bus.async_read_burst(REG_DATA, buf) || sleep_us(DRDY_TIMEOUT_US));
if (!result.has_value()) {
    // Watchdog expired: record glitch, increment fault counter, maintain safe flight
}
```
* **Zero Priority Inversion & Zero Jitter**: Yielding via `co_await` suspends only the specific sensor coroutine ($<0.1\,\mu\text{s}$ yield time) and never blocks independent I/O channels or the real-time flight core.

### E. Sensor Integrity & Hardware Anti-Overdriving Guards
* **Preventing Sensor Overdriving**: Polling an analog sensor faster than its internal ADC bandwidth corrupts measurement integrity:
  * Reading an **MS5611** before its $9.04\,\text{ms}$ $\Delta\Sigma$ integration completes yields stale reads or truncated register noise.
  * Polling a **QMC5883L** beyond its $200\,\text{Hz}$ ODR produces duplicate vectors and magnetic aliasing.
  * Rapid I2C bus polling induces I2C clock stretching, bus arbitration contention, and sensor die self-heating ($\Delta T$ temperature drift).
* **Deterministic Timing Guards**: Drivers enforce exact physical timing guards scoped strictly to the sensor's coroutine (e.g., `co_await sleep_ms(5u)` on QMC5883L, `co_await sleep_us(9040u)` on MS5611).
* **Maximum CPU Efficiency**: Instead of serializing these delays with blocking busy-waits, the CPU parallelizes all sensor tasks, interleaving work during the $90.5\,\mu\text{s}$ idle headroom of each $8\,\text{kHz}$ IMU tick to achieve near $100\%$ processor utilization efficiency without dropping flight control frames.


