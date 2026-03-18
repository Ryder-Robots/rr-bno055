# rr-bno055

C++ driver for the Bosch BNO055 9-axis IMU, targeting Raspberry Pi over I2C or UART.

The library is focused on the BNO055 and uses the [Bosch BNO055 SensorAPI](https://github.com/boschsensortec/BNO055_SensorAPI) under the hood. The transport and factory layers are deliberately **sensor-agnostic** — they carry no BNO055-specific knowledge — so support for other sensors in the Bosch IMU family can be added without touching the transport infrastructure. See [Contributing](#contributing) for details.

---

## Table of Contents

1. [Architecture](#architecture)
2. [Building](#building)
3. [Hardware Setup](#hardware-setup)
4. [Quick Start](#quick-start)
5. [API Reference](#api-reference)
6. [Calibration Workflow](#calibration-workflow)
7. [Orientation Check Tool](#orientation-check-tool)
8. [Testing](#testing)
9. [Contributing](#contributing)
10. [References](#references)
11. [Third-Party Libraries](#third-party-libraries)

---

## Architecture

The library is structured in three layers, each with a single responsibility:

```text
┌──────────────────────────────────────────────┐
│  Bno055Device                                │
│  Sensor-specific logic: init, axis remap,    │
│  data reads, calibration, status queries.    │
│  Validates BNO055 addresses (0x28 / 0x29).   │
└───────────────────┬──────────────────────────┘
                    │ std::shared_ptr<HardwareTransport>
┌───────────────────▼──────────────────────────┐
│  HardwareTransport (abstract)                │
│  Bus read/write + static C trampolines for   │
│  the Bosch SensorAPI. Sensor-agnostic.       │
│  Concrete: I2cHardwareTransport              │
│            UartHardwareTransport             │
└───────────────────┬──────────────────────────┘
                    │ std::shared_ptr<HardwareTransport>
┌───────────────────▼──────────────────────────┐
│  TransportFactory                            │
│  Creates and caches transport instances by   │
│  type. Completely sensor-agnostic.           │
└──────────────────────────────────────────────┘
```

Configuration follows the **Builder pattern**:

- `TransportConfig::Builder` — bus type, device path, I2C address
- `RrBNO055Config::Builder` — extends `TransportConfig::Builder` with axis remap and sign

---

## Building

### Prerequisites

| Requirement                       | Minimum version                          |
| --------------------------------- | ---------------------------------------- |
| CMake                             | 3.16                                     |
| C++ compiler                      | C++17                                    |
| GTest                             | any recent (for tests)                   |
| Internet access at configure time | fetches Bosch SensorAPI via FetchContent |

### Steps

```bash
cmake -B build
cmake --build build
```

### Running the tests

```bash
cd build
ctest --output-on-failure
# or run directly:
./rr_bno055_tests
```

### Installing

```bash
cmake --install build
```

---

## Hardware Setup

### I2C (default)

The driver defaults to `/dev/i2c-1` at I2C address `0x28`.

| BNO055 pin | Raspberry Pi pin     | Notes                                 |
| ---------- | -------------------- | ------------------------------------- |
| VCC        | 3.3 V (pin 1)        | Do **not** use 5 V — BNO055 is 3.3 V |
| GND        | GND (pin 6)          |                                       |
| SDA        | GPIO 2 / SDA (pin 3) |                                       |
| SCL        | GPIO 3 / SCL (pin 5) |                                       |
| PS0        | GND                  | Protocol select: PS1=0, PS0=0 → I2C   |
| PS1        | GND                  |                                       |
| ADR / COM3 | GND                  | Pull high for address 0x29            |

#### Enabling I2C on Ubuntu (Raspberry Pi)

`raspi-config` is not available on Ubuntu. Enable I2C by editing
`/boot/firmware/config.txt` (**not** `/boot/config.txt`) and adding:

```text
dtoverlay=i2c1
```

Then reboot and verify:

```bash
ls /dev/i2c*                 # should include /dev/i2c-1
sudo apt install i2c-tools
sudo i2cdetect -y 1          # should show 0x28 or 0x29
```

> **Note:** The user running the driver must be in the `i2c` group:
>
> ```bash
> sudo usermod -aG i2c $USER   # log out and back in to take effect
> ```

### UART (optional)

Set `TransportType::UART` in `TransportConfig` and provide the correct device node
(e.g. `/dev/ttyAMA0`). Wire the protocol-select pins for UART mode:

| BNO055 pin | Connection | Notes                    |
| ---------- | ---------- | ------------------------ |
| PS0        | 3.3 V      | PS1=0, PS0=1 → UART mode |
| PS1        | GND        |                          |
| TX         | Pi RX      |                          |
| RX         | Pi TX      |                          |

---

## Quick Start

```cpp
#include "rr_bno055/bno055_transport_config.hpp"
#include "rr_bno055/bno055_device.hpp"
#include "rr_bno055/transport_factory.hpp"

using namespace rr_bno055;

// 1. Build config (defaults: I2C, /dev/i2c-1, 0x28, axis remap X↔Y, sign negative)
RrBNO055Config::Builder b{};
auto conf = std::make_shared<RrBNO055Config>(b.build());

// 2. Create transport via factory
TransportFactory factory;
auto hw = factory.get_or_create_transport(conf);

// 3. Initialise the sensor
Bno055Device imu;
if (!imu.initialize(conf, hw)) {
    std::cerr << "IMU init failed\n";
    return 1;
}

// 4. Switch to NDOF fusion mode
imu.set_op_mode(RRBNO055_OPERATION_MODE_NDOF);

// 5. Wait for calibration (optional — poll until all sensors reach level 3)
uint8_t calib_status;
while (!imu.is_fully_calibrated(calib_status)) {
    RrBno055CalibData data;
    imu.get_calibration_status(data);
    std::cout << "sys=" << (int)data.sys  << " gyro=" << (int)data.gyro
              << " accel=" << (int)data.accel << " mag=" << (int)data.mag << '\n';
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// 6. Read orientation
bno055_quaternion_t q{};
if (imu.read_quaternion(q)) {
    std::cout << "w=" << q.w << " x=" << q.x << " y=" << q.y << " z=" << q.z << '\n';
}

// 7. Clean up
imu.deinitialize();
factory.cleanup();
```

---

## API Reference

### `Bno055Device`

All methods return `bool` (true = success) unless noted otherwise.

#### Lifecycle

| Method               | Description                                                                  |
| -------------------- | ---------------------------------------------------------------------------- |
| `initialize(conf, hw)` | Open sensor, set power mode, apply axis remap. Validates address is 0x28 or 0x29. |
| `deinitialize()`     | Set sensor to low-power mode.                                                |
| `reset()`            | Hardware reset, 650 ms reboot delay, re-init to NDOF mode.                  |

#### Configuration

| Method                        | Description                                              |
| ----------------------------- | -------------------------------------------------------- |
| `set_op_mode(mode)`           | Set operating mode (see `RrBno055OpMode`). Verifies readback. |
| `set_power_mode(mode)`        | Set power mode (see `RrBnoPowerMode`).                   |
| `set_axis_remap(remap, sign)` | Remap physical axes to match mounting orientation.       |

#### Data Reading

| Method                           | Output type            | Description                                        |
| -------------------------------- | ---------------------- | -------------------------------------------------- |
| `read_quaternion(quat)`          | `bno055_quaternion_t`  | Orientation as quaternion (w, x, y, z).            |
| `read_angular_velocity(gyro)`    | `bno055_gyro_t`        | Angular velocity (x, y, z).                        |
| `read_linear_acceleration(accel)` | `bno055_linear_accel_t` | Linear acceleration (x, y, z), gravity removed.  |
| `read_gravity(gravity)`          | `bno055_gravity_t`     | Gravity vector (x, y, z).                          |

#### Status

| Method                          | Description                                                                      |
| ------------------------------- | -------------------------------------------------------------------------------- |
| `get_system_status(status, error)` | Raw system status and error register values.                                  |
| `get_calibration_status(data)`  | Raw calibration levels (0–3) for sys, gyro, accel, mag.                          |
| `is_fully_calibrated(calib_status)` | Returns true when all four sensors reach level 3. `calib_status` is a bitmask of `RrBno055CalibStatus` flags indicating which sensors are not yet calibrated. |

### Axis Remap (`RrBno055AxisRemap`)

| Constant                    | Value | Effect                         |
| --------------------------- | ----- | ------------------------------ |
| `RRBNO055_DEFAULT_AXIS`     | 0x24  | X=X; Y=Y; Z=Z (default)        |
| `RRBNO055_REMAP_X_Y`        | 0x21  | Z=Z; X=Y; Y=X                  |
| `RRBNO055_REMAP_Y_Z`        | 0x18  | X=X; Y=Z; Z=Y                  |
| `RRBNO055_REMAP_Z_X`        | 0x06  | Y=Y; X=Z; Z=X                  |
| `RRBNO055_REMAP_X_Y_Z_TYPE0` | 0x12 | X=Z; Y=X; Z=Y                  |
| `RRBNO055_REMAP_X_Y_Z_TYPE1` | 0x09 | X=Y; Y=Z; Z=X                  |

### Calibration Status Bitmask (`RrBno055CalibStatus`)

| Flag                  | Bit  | Meaning                            |
| --------------------- | ---- | ---------------------------------- |
| `FULLY_CALIBRATED`    | 0x00 | All sensors calibrated             |
| `SYS_NOT_CALIBRATED`  | 0x01 | System composite not at level 3    |
| `GYRO_NOT_CALIBRATED` | 0x02 | Gyroscope not at level 3           |
| `ACCEL_NOT_CALIBRATED` | 0x04 | Accelerometer not at level 3      |
| `MAG_NOT_CALIBRATED`  | 0x08 | Magnetometer not at level 3        |

---

## Calibration Workflow

The BNO055 must be calibrated before orientation data is reliable. Calibration is
retained in volatile memory and must be repeated each power cycle (or restored from
saved offsets — offset save/restore is not yet implemented).

1. Switch to **NDOF** mode (`set_op_mode(RRBNO055_OPERATION_MODE_NDOF)`).
2. Move the sensor through the calibration motions:
   - **Gyroscope**: leave completely still for a few seconds.
   - **Accelerometer**: place in 6 different stable orientations (each for a few seconds).
   - **Magnetometer**: rotate the sensor in a figure-8 pattern.
3. Poll `get_calibration_status()` until all four fields reach 3, or use
   `is_fully_calibrated()` for a simple pass/fail check.

> `is_fully_calibrated()` delegates to `get_calibration_status()` internally —
> calling both in the same loop does **not** cause duplicate I2C transactions.

---

## Orientation Check Tool

`imu_orientation_check` is a hardware diagnostic utility that connects to a real
BNO055, waits for full calibration, then displays a live rolling readout of
orientation, angular velocity, and calibration status — without needing a ROS
stack. Use it immediately after wiring the sensor to confirm the axis mapping is
correct before integrating the driver into a larger system.

### Running

```bash
# Default — I2C on /dev/i2c-1, address 0x28
sudo ./build/imu_orientation_check

# Different I2C address
sudo ./build/imu_orientation_check --address 0x29

# UART
sudo ./build/imu_orientation_check --uart --device /dev/ttyAMA0

# Skip calibration wait, faster refresh rate
sudo ./build/imu_orientation_check --no-wait --rate 20
```

### Options

| Option           | Default      | Description                                               |
| ---------------- | ------------ | --------------------------------------------------------- |
| `--device PATH`  | `/dev/i2c-1` | I2C or UART device node                                   |
| `--address ADDR` | `0x28`       | I2C address in hex (`0x28` or `0x29`)                     |
| `--uart`         | off          | Use UART transport; sets default device to `/dev/ttyAMA0` |
| `--no-wait`      | off          | Skip calibration wait; start reading immediately          |
| `--rate HZ`      | `10`         | Refresh rate in Hz (1–50)                                 |

### What it displays

```text
─── BNO055 Orientation Check ───────────────────

  Euler Angles (ZYX / aerospace)
  Roll  (X):  +0.12°
  Pitch (Y):  -1.45°
  Yaw   (Z):  +87.30°

  Raw Quaternion  (normalised)
  w=0.9990  x=0.0010  y=-0.0126  z=0.0431

  Angular Velocity (°/s)
  x=+0.0  y=+0.1  z=-0.1

  Calibration  (0=none  3=full)
  Sys  [====] 3
  Gyro [====] 3
  Accel[====] 3
  Mag  [====] 3
```

Angle values are colour-coded: **green** (< 5°), **yellow** (< 30°), **red** (≥ 30°).
Calibration bars turn green at level 3 and red at level 0.

### Verifying orientation

With the sensor lying flat, label-side up, in its intended mounting orientation:

1. **Roll and Pitch should read near 0°.** Tilt the sensor left/right and
   forward/back and confirm the correct axis responds.
2. **Yaw should change smoothly** as you rotate the sensor about the vertical
   axis (no jumps or reversals).
3. If an axis is inverted or swapped, adjust `axis_remap` in `RrBNO055Config`
   (see [Axis Remap](#axis-remap-rrbno055axisremap)) and re-run until the
   behaviour matches the physical mounting.

Press **Ctrl+C** at any time to exit cleanly.

---

## Testing

The test suite uses **Google Test** and runs without hardware by substituting a
`MockHardwareTransport` that opens `/dev/null` as a safe file descriptor and
returns zero for all register reads.

```bash
cd build && ./rr_bno055_tests
```

Tests are grouped by class:

| Suite                    | Coverage                                    |
| ------------------------ | ------------------------------------------- |
| `TransportConfigTest`    | Builder defaults and custom fields          |
| `Bno055TransportConfigTest` | BNO055-specific builder fields           |
| `TransportFactoryTest`   | Transport creation, caching, cleanup        |
| `HardwareTransportTest`  | Init guards, delay timing                   |
| `Bno055DeviceTest`       | Full device lifecycle, all data/status reads |

Hardware-dependent code paths (actual I2C/UART transactions, chip ID validation)
are excluded from mock-based tests by design.

---

## Contributing

Contributions are welcome. Please open a pull request against `main`.

### Adding support for another Bosch IMU sensor

The transport and factory layers are fully sensor-agnostic — they know nothing
about the BNO055 specifically. Adding a new Bosch sensor (e.g. BNO080, BMI088)
requires only:

1. **Config class** — create `RrYourSensorConfig` extending `TransportConfig`
   (see `bno055_transport_config.hpp` as a template). Add any sensor-specific
   configuration fields via the Builder pattern.

2. **Device class** — create `YourSensorDevice` (analogous to `Bno055Device`)
   that wires the sensor's C API function pointers into the transport trampolines
   via `HardwareTransport::get_bus_read_fn()` and `get_bus_write_fn()`, then
   implements the sensor's read and status methods.

3. **Unit tests** — add a `MockHardwareTransport` (or reuse the existing one)
   and write tests covering config validation, lifecycle, and data reads without
   requiring physical hardware.

The `HardwareTransport`, `I2cHardwareTransport`, `UartHardwareTransport`, and
`TransportFactory` classes require **no changes** for a new sensor.

If you would like to contribute support for an additional sensor, please raise an
issue first so the scope can be agreed on before work begins.

---

## References

- Datasheet (BST-BNO055-DS000): [BST-BNO055-DS000](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- Quick Start Application Note (BST-BNO055-AN007): [BST-BNO055-AN007](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf)

---

## Third-Party Libraries

### Bosch Sensortec BNO055 SensorAPI

Fetched at configure time via CMake `FetchContent`.

- Repository: [BNO055_SensorAPI](https://github.com/boschsensortec/BNO055_SensorAPI)
- Copyright (C) 2015–2016 Bosch Sensortec GmbH
- License: BSD-3-Clause
