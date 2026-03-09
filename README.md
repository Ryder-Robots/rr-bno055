# rr-bno055

C++ ROS 2 driver for the Bosch BNO055 9-axis IMU, targeting Raspberry Pi.

## Hardware Connection

### I2C (default)

The driver defaults to `/dev/i2c-1` at I2C address `0x28`.

| BNO055 pin | Raspberry Pi pin     | Notes                                |
|------------|----------------------|--------------------------------------|
| VCC        | 3.3 V (pin 1)        | Do not use 5 V -- BNO055 is 3.3 V    |
| GND        | GND (pin 6)          |                                      |
| SDA        | GPIO 2 / SDA (pin 3) |                                      |
| SCL        | GPIO 3 / SCL (pin 5) |                                      |
| PS0        | GND                  | Protocol select: PS1=0, PS0=0 -> I2C |
| PS1        | GND                  |                                      |
| ADR / COM3 | GND                  | Pull high for address 0x29           |

Enable I2C on the Raspberry Pi with:

```bash
sudo raspi-config   # Interface Options -> I2C -> Enable
```

Verify the sensor is visible before running the driver:

```bash
sudo i2cdetect -y 1   # should show 0x28 (or 0x29)
```

### UART (optional)

Set `TransportType::UART` in `TransportConfig` and provide the correct
device node (e.g. `/dev/ttyAMA0`).  Wire the BNO055 protocol-select pins
for UART mode:

| BNO055 pin | Connection | Notes                     |
|------------|------------|---------------------------|
| PS0        | 3.3 V      | PS1=0, PS0=1 -> UART mode |
| PS1        | GND        |                           |
| TX         | Pi RX      |                           |
| RX         | Pi TX      |                           |

## References

* Datasheet (BST-BNO055-DS000): [BST-BNO055-DS000](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)

* Quick Start Application Note (BST-BNO055-AN007): [BST-BNO055-AN007](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf)

## Third-Party Libraries

### Bosch Sensortec BNO055 SensorAPI

* Location: `vendor/bno055_sensorapi/`
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
* License: BSD-3-Clause
* Source: [BNO055_SensorAPI](https://github.com/boschsensortec/BNO055_SensorAPI)
