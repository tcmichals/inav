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
