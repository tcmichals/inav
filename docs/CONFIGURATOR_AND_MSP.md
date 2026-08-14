# iNav Configurator Integration, MSP Protocol & CLI Engine

## 1. iNav Configurator Connection Guide

`inav-abstractx` natively implements the **MultiWii Serial Protocol (MSP v1 and MSP v2)** ([`src/msp/msp_protocol.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/msp_protocol.hpp)) and exposes an asynchronous TCP server on **Port 5760** ([`src/msp/msp_server.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/msp/msp_server.hpp)).

### Connecting via iNav Configurator:
1. Launch **iNav Configurator** (v6.x / v7.x / v8.x).
2. In the top-right port selection drop-down, select **`TCP`**.
3. Set Address / Port to:
   * **Linux SITL / SBC**: `127.0.0.1:5760`
   * **Pico 2 W (Wi-Fi AP)**: `192.168.4.1:5760` (SSID: `iNav-Pico2W`)
4. Click **Connect**. The Configurator will automatically download PID settings, sensor calibration, mixer setup, and telemetry streams.

---

## 2. MSP Protocol Architecture (v1 & v2)

```
MSP v1 Frame: [ $ ] [ M ] [ < / > ] [ Size (1B) ] [ Cmd (1B) ] [ Payload (N B) ] [ Checksum (XOR) ]
MSP v2 Frame: [ $ ] [ X ] [ < / > ] [ Flag (1B) ] [ Cmd (2B) ] [ Size (2B) ] [ Payload (N B) ] [ CRC8-DVB-S2 ]
```

### Supported MSP Commands:
* **API & Identification**: `MSP_API_VERSION` (`1`), `MSP_FC_VARIANT` (`2` -> "INAV"), `MSP_FC_VERSION` (`3`), `MSP_BOARD_INFO` (`4`), `MSP_BUILD_INFO` (`5`).
* **Live Telemetry**: `MSP_STATUS` (`101`), `MSP_RAW_IMU` (`102`), `MSP_SERVO` (`103`), `MSP_MOTOR` (`104`), `MSP_RC` (`105`), `MSP_RAW_GPS` (`106`), `MSP_ATTITUDE` (`108`), `MSP_ALTITUDE` (`109`), `MSP_ANALOG` (`110`).
* **Tuning & Parameters**: `MSP_PID` (`112`), `MSP_SET_PID` (`202`), `MSP_BOXIDS` (`119`), `MSP_FILTER_CONFIG` (`92`), `MSP_SET_FILTER_CONFIG` (`93`).
* **ESC Passthrough**: `MSP_SET_4WAY_IF` (`245`): BLHeliSuite / AM32 ESC firmware flashing over MSP.
* **Storage Actions**: `MSP_EEPROM_WRITE` (`250`), `MSP_RESET_CONF` (`208`).

---

## 3. Command Line Interface (CLI Engine)

When iNav Configurator switches to the **CLI tab**, the firmware handles terminal commands via [`src/config/cli_engine.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/cli_engine.hpp):

### Supported CLI Commands:
* `version`: Prints firmware version, Git commit hash, and compiler toolchain information.
* `status`: Reports system uptime, CPU load %, I2C error count, sensor health, and battery voltage.
* `dump`: Outputs full flat configuration in human-readable text format.
* `save`: Commits current in-memory parameters to SPI Flash sector `0x1F0000` (or `config.bin`).
* `defaults`: Resets all parameters to safe factory defaults.
