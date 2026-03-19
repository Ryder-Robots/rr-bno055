// Copyright (c) 2026 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <array>
#include <memory>
extern "C" {
#include "bno055.h"
}
#include "rr_bno055/hardware_transport.hpp"
#include "rr_bno055/transport_factory.hpp"
#include "rr_bno055/bno055_transport_config.hpp"

namespace rr_bno055
{

enum RrBno055OpMode : uint8_t
{
  RRBNO055_OPERATION_MODE_CONFIG = BNO055_OPERATION_MODE_CONFIG,
  RRBNO055_OPERATION_MODE_ACCONLY = BNO055_OPERATION_MODE_ACCONLY,
  RRBNO055_OPERATION_MODE_MAGONLY = BNO055_OPERATION_MODE_MAGONLY,
  RRBNO055_OPERATION_MODE_GYRONLY = BNO055_OPERATION_MODE_GYRONLY,
  RRBNO055_OPERATION_MODE_ACCMAG = BNO055_OPERATION_MODE_ACCMAG,
  RRBNO055_OPERATION_MODE_ACCGYRO = BNO055_OPERATION_MODE_ACCGYRO,
  RRBNO055_OPERATION_MODE_MAGGYRO = BNO055_OPERATION_MODE_MAGGYRO,
  RRBNO055_OPERATION_MODE_AMG = BNO055_OPERATION_MODE_AMG,
  RRBNO055_OPERATION_MODE_IMUPLUS = BNO055_OPERATION_MODE_IMUPLUS,
  RRBNO055_OPERATION_MODE_COMPASS = BNO055_OPERATION_MODE_COMPASS,
  RRBNO055_OPERATION_MODE_M4G = BNO055_OPERATION_MODE_M4G,
  RRBNO055_OPERATION_MODE_NDOF_FMC_OFF = BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  RRBNO055_OPERATION_MODE_NDOF = BNO055_OPERATION_MODE_NDOF,
};

enum RrBnoPowerMode : uint8_t
{
  RRBNO055_POWER_MODE_NORMAL = BNO055_POWER_MODE_NORMAL,
  RRBNO055_POWER_MODE_LOWPOWER = BNO055_POWER_MODE_LOWPOWER,
  RRBNO055_POWER_MODE_SUSPEND = BNO055_POWER_MODE_SUSPEND,
};

enum RrBno055CalibStatus : uint8_t
{
  FULLY_CALIBRATED = 0x00,
  SYS_NOT_CALIBRATED = 0x01,
  GYRO_NOT_CALIBRATED = 0x02,
  ACCEL_NOT_CALIBRATED = 0x04,
  MAG_NOT_CALIBRATED = 0x08,
};

/**
 * @brief Raw calibration levels for each BNO055 sensor subsystem.
 * Each field ranges from 0 (uncalibrated) to 3 (fully calibrated).
 * Only valid if get_calibration_status() returns true.
 */
struct RrBno055CalibData
{
  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
};

class Bno055Device
{
public:
  Bno055Device() = default;
  virtual ~Bno055Device() = default;

  // Lifecycle:
  bool initialize(std::shared_ptr<RrBNO055Config> conf, std::shared_ptr<HardwareTransport> hw);
  void deinitialize();
  bool reset();

  // Configuration: (reference bno055_set_operation_mode)
  bool set_op_mode(RrBno055OpMode mode);

  // this will be handled internally but sohuld it need to be handled externally it is availabl.
  bool set_power_mode(RrBnoPowerMode mode);

  /**
   * @brief Applies axis remapping and per-axis sign correction to the BNO055.
   *
   * Must be called while the device is in CONFIG mode
   * (`RRBNO055_OPERATION_MODE_CONFIG`).  `initialize()` calls this
   * automatically using the values from `RrBNO055Config`; call it again
   * only if you need to change the mapping at runtime (switch back to
   * CONFIG mode first, then restore your operating mode afterwards).
   *
   * @param remap  Axis permutation to apply (see `RrBno055AxisRemap`).
   * @param sign   Per-axis output polarity (see `RrBno055AxisSignXYZ`).
   * @return true on success, false if any Bosch API call fails.
   */
  bool set_axis_remap(RrBno055AxisRemap remap, RrBno055AxisSignXYZ sign);

  // Data Reading:

  bool read_quaternion(bno055_quaternion_t& quat);
  bool read_angular_velocity(bno055_gyro_t& gyro);
  bool read_linear_acceleration(bno055_linear_accel_t& accel);
  bool read_gravity(bno055_gravity_t& gravity);

  // Status:

  /**
   * @brief Retrieves the raw calibration level for each BNO055 sensor subsystem.
   *
   * Each field in RrBno055CalibData ranges from 0 (uncalibrated) to 3 (fully
   * calibrated). This method is intended as a companion to is_fully_calibrated()
   * — use is_fully_calibrated() for the binary pass/fail check, and this method
   * when you need to know exactly where each sensor is in the calibration process.
   *
   * @param[out] data  Struct populated with raw calibration levels on success.
   *                   Fields are zero-initialised — if false is returned, field
   *                   values should not be trusted.
   *
   * @return true  if all four sensor status reads succeeded.
   * @return false if any read failed at the hardware level. Inspect
   *               get_system_status() for further diagnostic information.
   */
  bool get_calibration_status(RrBno055CalibData& data);

  /**
   * @brief Checks whether all BNO055 sensors are fully calibrated.
   *
   * Queries the calibration status of all four sensors — system, gyroscope,
   * accelerometer, and magnetometer — and reports which, if any, are not yet
   * fully calibrated.
   *
   * @param[out] calib_status A bitmask of type RrBno055CalibStatus indicating
   * the calibration state of each sensor. The value is built using bitwise OR,
   * meaning multiple sensors can be reported as uncalibrated simultaneously.
   *
   * Understanding the bitmask:
   *
   *   calib_status is initialised to FULLY_CALIBRATED (0x00) before any checks
   *   are performed. For each sensor that fails its calibration check, its
   *   corresponding flag bit is OR'd into calib_status. Because each flag
   *   occupies a unique bit position, multiple failures combine without
   *   collision:
   *
   *     FULLY_CALIBRATED     = 0x00  (0000 0000) — all sensors calibrated
   *     SYS_NOT_CALIBRATED   = 0x01  (0000 0001)
   *     GYRO_NOT_CALIBRATED  = 0x02  (0000 0010)
   *     ACCEL_NOT_CALIBRATED = 0x04  (0000 0100)
   *     MAG_NOT_CALIBRATED   = 0x08  (0000 1000)
   *
   *   Example: if gyro and mag are both uncalibrated, calib_status will be:
   *     0x02 | 0x08 = 0x0A  (0000 1010)
   *
   *   To test for a specific failure in the caller:
   *     if (calib_status & GYRO_NOT_CALIBRATED) { ... }
   *
   *   To test for complete calibration:
   *     if (calib_status == FULLY_CALIBRATED) { ... }
   *   or equivalently rely on the return value directly.
   *
   * A sensor is considered fully calibrated when its calibration level
   * reaches 3 (the maximum reported by the BNO055). Both a read failure
   * and a calibration level below 3 are treated as not calibrated, since
   * in either case the sensor cannot be relied upon.
   *
   * @return true  if all four sensors report calibration level 3.
   * @return false if any sensor is uncalibrated or its status could not
   *               be read. Inspect calib_status for detail.
   */
  bool is_fully_calibrated(uint8_t& calib_status);

  bool get_system_status(uint8_t& status, uint8_t& error);

private:
  bool check_bno_result(BNO055_RETURN_FUNCTION_TYPE rv, const char* context) const;

  std::shared_ptr<HardwareTransport> hw_;

  /**
   * @brief Returns a copy of the Bosch SensorAPI device struct.
   *
   * The returned struct contains the function pointers wired to this transport
   * and can be passed to higher-level Bosch API calls (e.g. reading sensor data).
   * Only valid after `initialize()` has succeeded.
   */
  bno055_t device_;

  // Intended mode after given a set_op_mode command.
  RrBno055OpMode current_op_mode_ = RRBNO055_OPERATION_MODE_CONFIG;

  // defines the delay between mode changes.
  static constexpr std::array<uint32_t, 13> OP_MODE_SWITCH_DELAY = {
    19,  // BNO055_OPERATION_MODE_CONFIG       = 0x00
    3,   // BNO055_OPERATION_MODE_ACCONLY      = 0x01
    3,   // BNO055_OPERATION_MODE_MAGONLY      = 0x02
    3,   // BNO055_OPERATION_MODE_GYRONLY      = 0x03
    3,   // BNO055_OPERATION_MODE_ACCMAG       = 0x04
    3,   // BNO055_OPERATION_MODE_ACCGYRO      = 0x05
    3,   // BNO055_OPERATION_MODE_MAGGYRO      = 0x06
    3,   // BNO055_OPERATION_MODE_AMG          = 0x07
    7,   // BNO055_OPERATION_MODE_IMUPLUS      = 0x08
    7,   // BNO055_OPERATION_MODE_COMPASS      = 0x09
    7,   // BNO055_OPERATION_MODE_M4G          = 0x0A
    7,   // BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B
    7,   // BNO055_OPERATION_MODE_NDOF         = 0x0C
  };
  static_assert(OP_MODE_SWITCH_DELAY.size() == BNO055_OPERATION_MODE_NDOF + 1,
                "OP_MODE_SWITCH_DELAY size must cover all operation modes");

  static constexpr std::array<uint32_t, 3> PWR_MODE_SWITCH_DELAY = {
    400,  // RRBNO055_POWER_MODE_NORMAL
    50,   // RRBNO055_POWER_MODE_LOWPOWER
    20,   // BNO055_POWER_MODE_SUSPEND
  };
  static_assert(PWR_MODE_SWITCH_DELAY.size() == BNO055_POWER_MODE_SUSPEND + 1,
                "PWR_MODE_SWITCH_DELAY size must cover all power modes");
};
}  // namespace rr_bno055