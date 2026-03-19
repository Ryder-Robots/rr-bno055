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

#include "rr_bno055/bno055_device.hpp"

using namespace rr_bno055;

// Translates a BNO055_RETURN_FUNCTION_TYPE result into a bool, logging the
// error code and caller context when the call did not succeed.
bool Bno055Device::check_bno_result(BNO055_RETURN_FUNCTION_TYPE rv, const char* context) const
{
  switch (rv)
  {
    case BNO055_SUCCESS:
      return true;
    case BNO055_E_NULL_PTR:
      std::cerr << "[Bno055Device] " << context << ": BNO055_E_NULL_PTR internal pointer is not set\n";
      return false;
    case BNO055_ERROR:
      std::cerr << "[Bno055Device] " << context << ": BNO055_ERROR general error condition\n";
      return false;
    default:
      std::cerr << "[Bno055Device] " << context << ": unknown error code " << (int)rv << '\n';
      return false;
  }
}

bool Bno055Device::initialize(std::shared_ptr<RrBNO055Config> conf, std::shared_ptr<HardwareTransport> hw)
{
  // verify that the device has a valid address assigned.
  if (conf->device.empty() || !(conf->address == 0x28 || conf->address == 0x29))
  {
    std::cerr << "[Bno055Device] Failed to open device: no device defined or address not one of 28 or 29\n";
    return false;
  }
  if (!hw || !hw->is_initilized())
  {
    std::cerr << "[Bno055Device] hardware transport is not initilized\n";
    return false;
  }
  hw_ = hw;

  device_.dev_addr = conf->address;
  device_.bus_read = HardwareTransport::get_bus_read_fn();
  device_.bus_write = HardwareTransport::get_bus_write_fn();
  device_.delay_msec = HardwareTransport::delay_msec;

  if (bno055_init(&device_) != 0)
  {
    std::cerr << "[Bno055Device] could not initialize IMU\n";
    hw_ = nullptr;
    return false;
  }

  // set power mode to normal.
  if (!set_power_mode(RrBnoPowerMode::RRBNO055_POWER_MODE_NORMAL))
  {
    std::cerr << "[Bno055Device] could not set power mode\n";
    hw_ = nullptr;
    return false;
  }

  // set orientation.
  if (set_op_mode(RRBNO055_OPERATION_MODE_CONFIG))
  {
    if (!set_axis_remap(conf->axis_remap, conf->axis_sign_xyz))
    {
      std::cerr << "[Bno055Device] could not set intended orientation\n";
      hw_ = nullptr;
      return false;
    }
  }
  else
  {
    std::cerr << "[Bno055Device] unable to enter configuration mode\n";
    hw_ = nullptr;
    return false;
  }

  return true;
}

void Bno055Device::deinitialize()
{
  if (hw_ && hw_->is_initilized())
  {
    // Attempt to set low power mode; continue deinitialization if this fails.
    if (!set_power_mode(RrBnoPowerMode::RRBNO055_POWER_MODE_LOWPOWER))
    {
      std::cerr << "[Bno055Device] unable to set device to low power mode\n";
    }
  }
}

bool Bno055Device::set_op_mode(RrBno055OpMode mode)
{
  // capture intended mode change first, doesn't matter if hardware is inilized
  // or not, we know the intention and should capture that.
  current_op_mode_ = mode;

  // this may be a little redundant but does show a different condition.
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] initilize has not be ran\n";
    return false;
  }

  BNO055_RETURN_FUNCTION_TYPE rv = BNO055_ERROR;
  if ((rv = bno055_set_operation_mode(mode)) == BNO055_SUCCESS)
  {
    hw_->delay_msec(OP_MODE_SWITCH_DELAY[mode]);
    uint8_t actual_mode = 0;
    if (bno055_get_operation_mode(&actual_mode) != BNO055_SUCCESS)
    {
      std::cerr << "[Bno055Device] set_op_mode: could not read back mode from hardware\n";
      return false;
    }
    if (actual_mode != static_cast<uint8_t>(current_op_mode_))
    {
      std::cerr << "[Bno055Device] set_op_mode: mode mismatch — intended " << (int)current_op_mode_ << " got "
                << (int)actual_mode << '\n';
      return false;
    }
    return true;
  }
  return check_bno_result(rv, "set_op_mode");
}

bool Bno055Device::set_power_mode(RrBnoPowerMode mode)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] initilize has not be ran\n";
    return false;
  }

  BNO055_RETURN_FUNCTION_TYPE rv = BNO055_ERROR;
  if ((rv = bno055_set_power_mode(mode)) == BNO055_SUCCESS)
  {
    hw_->delay_msec(PWR_MODE_SWITCH_DELAY[mode]);
    return true;
  }
  return check_bno_result(rv, "set_power_mode");
}

bool Bno055Device::reset()
{
  // since the object has not been initlized it does not need to be reset.
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] initilize has not be ran\n";
    return true;
  }
  BNO055_RETURN_FUNCTION_TYPE com_rslt = bno055_set_sys_rst(BNO055_BIT_ENABLE);
  if (com_rslt != BNO055_SUCCESS)
    return check_bno_result(com_rslt, "reset");

  // BNO055 datasheet: ~650ms to reboot after reset
  hw_->delay_msec(650);
  // Sensor comes up in CONFIG mode, re-apply your operating mode
  BNO055_RETURN_FUNCTION_TYPE init_rslt = bno055_init(&device_);
  if (init_rslt != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] bno055_init failed after reset: " << (int)init_rslt << '\n';
    return false;
  }
  return set_op_mode(RRBNO055_OPERATION_MODE_NDOF);
}

/*
 *    remap_axis_u8 |   result     | comments
 *   ------------|-------------------|------------
 *      0X21     | BNO055_REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | BNO055_REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | BNO055_REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | BNO055_REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | BNO055_REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | BNO055_DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 */
bool Bno055Device::set_axis_remap(RrBno055AxisRemap remap, RrBno055AxisSignXYZ sign)
{
  if (bno055_set_axis_remap_value(remap) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap: failed to set remap value\n";
    return false;
  }
  if (bno055_set_remap_x_sign(sign.x_sign) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap on x: failed to set remap sign\n";
    return false;
  }
  if (bno055_set_remap_y_sign(sign.y_sign) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap on y: failed to set remap sign\n";
    return false;
  }
  if (bno055_set_remap_z_sign(sign.z_sign) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap on z: failed to set remap sign\n";
    return false;
  }
  return true;
}

bool Bno055Device::read_quaternion(bno055_quaternion_t& quat)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] read_quaternion: initialize has not been run\n";
    return false;
  }
  return check_bno_result(bno055_read_quaternion_wxyz(&quat), "read_quaternion");
}

bool Bno055Device::read_angular_velocity(bno055_gyro_t& gyro)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] read_angular_velocity: initialize has not been run\n";
    return false;
  }
  return check_bno_result(bno055_read_gyro_xyz(&gyro), "read_angular_velocity");
}

bool Bno055Device::read_linear_acceleration(bno055_linear_accel_t& accel)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] read_linear_acceleration: initialize has not been run\n";
    return false;
  }
  return check_bno_result(bno055_read_linear_accel_xyz(&accel), "read_linear_acceleration");
}

bool Bno055Device::read_gravity(bno055_gravity_t& gravity)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] read_gravity: initialize has not been run\n";
    return false;
  }
  return check_bno_result(bno055_read_gravity_xyz(&gravity), "read_gravity");
}

bool Bno055Device::get_system_status(uint8_t& status, uint8_t& error)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] get_system_status: initialize has not been run\n";
    return false;
  }

  bool ok = true;

  if (bno055_get_sys_stat_code(&status) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_system_status: failed to read system status\n";
    ok = false;
  }

  // always attempt to read error code — may explain a stat failure
  if (bno055_get_sys_error_code(&error) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_system_status: failed to read error code\n";
    ok = false;
  }

  return ok;
}

bool Bno055Device::is_fully_calibrated(uint8_t& calib_status)
{
  RrBno055CalibData data;
  calib_status = RrBno055CalibStatus::FULLY_CALIBRATED;

  // Delegate reads to get_calibration_status to avoid duplicate I2C transactions.
  // Treat read failures as uncalibrated — sensor data cannot be trusted either way.
  bool ok = get_calibration_status(data);
  if (!ok || data.sys < 3)
    calib_status |= RrBno055CalibStatus::SYS_NOT_CALIBRATED;
  if (!ok || data.gyro < 3)
    calib_status |= RrBno055CalibStatus::GYRO_NOT_CALIBRATED;
  if (!ok || data.accel < 3)
    calib_status |= RrBno055CalibStatus::ACCEL_NOT_CALIBRATED;
  if (!ok || data.mag < 3)
    calib_status |= RrBno055CalibStatus::MAG_NOT_CALIBRATED;

  return calib_status == RrBno055CalibStatus::FULLY_CALIBRATED;
}

bool Bno055Device::get_calibration_status(RrBno055CalibData& data)
{
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] get_calibration_status: initialize has not been run\n";
    return false;
  }

  bool ok = true;

  if (bno055_get_sys_calib_stat(&data.sys) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_calibration_status: failed to read sys calibration\n";
    ok = false;
  }
  if (bno055_get_gyro_calib_stat(&data.gyro) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_calibration_status: failed to read gyro calibration\n";
    ok = false;
  }
  if (bno055_get_accel_calib_stat(&data.accel) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_calibration_status: failed to read accel calibration\n";
    ok = false;
  }
  if (bno055_get_mag_calib_stat(&data.mag) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] get_calibration_status: failed to read mag calibration\n";
    ok = false;
  }

  return ok;
}
