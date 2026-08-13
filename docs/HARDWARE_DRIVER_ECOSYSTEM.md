# Hardware Driver & Protocol Ecosystem Specification

> [!IMPORTANT]
> **MODULAR HARDWARE DRIVER ARCHITECTURE**
> In **`inav-abstractx`**, every sensor chip, ESC protocol, RC receiver, LED controller, and display has its own dedicated, modular driver file inside **`src/drivers/`**.
>
> Sensor polling and protocol decoding are normalized over **AbstractX Virtual BAR Address Maps** (`PCIE_BAR_IMU_BASE`, `PCIE_BAR_BARO_BASE`, `PCIE_BAR_SERIAL_BASE`, `PCIE_BAR_ESC_BASE`).

---

## 1. Modular Driver Directory Tree (`src/drivers/`)

```
inav/src/drivers/
├── imu/
│   ├── imu_base.hpp           # Base ImuSample struct & RegWrite definitions
│   ├── icm42688p.hpp          # Dedicated TDK ICM-42688-P driver & init sequence
│   ├── bmi088.hpp             # Dedicated Bosch BMI088 driver & init sequence
│   └── mpu6000.hpp            # Dedicated InvenSense MPU-6000 driver & init sequence
│
├── baro/
│   ├── bmp280.hpp             # Dedicated Bosch BMP280 barometer driver
│   └── ms5611.hpp             # Dedicated MEAS MS5611 barometer driver
│
├── mag/
│   ├── qmc5883l.hpp           # Dedicated QST QMC5883L compass driver
│   └── ist8310.hpp            # Dedicated Isentek IST8310 compass driver
│
├── esc/
│   ├── dshot.hpp              # Dedicated DShot150/300/600/1200 + Bidirectional RPM driver
│   ├── oneshot.hpp            # Dedicated OneShot125 / OneShot42 / MultiShot driver
│   └── pwm_esc.hpp            # Dedicated FastPWM & Standard PWM motor driver
│
├── rc/
│   ├── crsf.hpp               # Dedicated ExpressLRS / TBS Crossfire RC driver
│   ├── sbus.hpp               # Dedicated Futaba / FrSky SBUS RC driver
│   └── ibus.hpp               # Dedicated FlySky IBUS RC driver
│
├── led/
│   ├── status_led.hpp         # Dedicated Onboard Status Indicator LEDs driver
│   └── ws2812.hpp             # Dedicated WS2812 RGB LED strip controller
│
└── display/
    ├── oled_ssd1306.hpp       # Dedicated I2C SSD1306 OLED display driver (128x64)
    └── osd_max7456.hpp        # Dedicated MAX7456 Analog Video OSD driver
```

---

## 2. IMU (Gyroscope & Accelerometer) Chipsets (`src/drivers/imu/`)

- **Bosch Sensortec**: Bosch BMI088 ([`bmi088.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/bmi088.hpp)), Bosch BMI270, Bosch BMI160, BMA280.
- **TDK InvenSense**: ICM-42688-P ([`icm42688p.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/icm42688p.hpp)), ICM-42605, MPU-6000 ([`mpu6000.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/imu/mpu6000.hpp)), MPU-6050, MPU-6500, ICM-20689, ICM-20602.
- **STMicroelectronics**: LSM6DSO, LSM6DS3.

---

## 3. Barometer & Compass Hardware (`src/drivers/baro/`, `src/drivers/mag/`)

- **Barometers**: Bosch BMP280 ([`bmp280.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/bmp280.hpp)), BMP388, BMP390, Infineon DPS310, MEAS MS5611 ([`ms5611.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/baro/ms5611.hpp)), Goertek SPL06-001.
- **Magnetometers / Compasses**: QST QMC5883L ([`qmc5883l.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/mag/qmc5883l.hpp)), Honeywell HMC5883L, Isentek IST8310, ST LIS3MDL.

---

## 4. ESC Motor Output Protocol Suite (`src/drivers/esc/`)

| ESC Driver | Protocol / Range | Feature / Advantage |
| :--- | :--- | :--- |
| [`dshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/dshot.hpp) | DShot150/300/600/1200 + BiDirectional | Digital stream with 4-bit CRC & eRPM return |
| [`oneshot.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/oneshot.hpp) | OneShot125 / OneShot42 / MultiShot | Fast pulse width analog protocols |
| [`pwm_esc.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/esc/pwm_esc.hpp) | FastPWM (2 kHz) & Standard PWM | Standard 50Hz..400Hz servo/ESC outputs |

---

## 5. Control Receiver Inputs & Peripherals (`src/drivers/rc/`, `led/`, `display/`)

- **RC Inputs**: ExpressLRS/CRSF ([`crsf.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/crsf.hpp)), Futaba/FrSky SBUS ([`sbus.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/rc/sbus.hpp)), FlySky IBUS, Spektrum SRXL2, Multi-channel PWM, PPM.
- **LED Indicators**: Onboard Status LEDs ([`status_led.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/led/status_led.hpp)) & WS2812 RGB LED Strip.
- **Displays**: SSD1306 OLED ([`oled_ssd1306.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/display/oled_ssd1306.hpp)) & MAX7456 Analog Video OSD ([`osd_max7456.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/drivers/display/osd_max7456.hpp)).
