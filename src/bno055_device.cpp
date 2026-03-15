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

bool Bno055Device::initialize(std::shared_ptr<RrBNO055Config> conf, std::shared_ptr<HardwareTransport> hw)
{
  // verify that the device has a valid address assigned.
  if (conf->device.empty() || !(conf->address == 0x28 || conf->address == 0x29))
  {
    std::cerr << "[Bno055Device] Failed to open device: no device defined or address not one of 28 or 29" << std::endl;
    return false;
  }
  if (!hw || !hw->is_initilized())
  {
    std::cerr << "[Bno055Device] hardware transport is not initilized" << std::endl;
    return false;
  }
  hw_ = hw;

  if (bno055_init(&device_) != 0)
  {
    std::cerr << "[Bno055Device] could not initialize IMU" << std::endl;
    hw_ = nullptr;
    return false;
  }

  // set power mode to normal.
  if (!set_power_mode(RrBnoPowerMode::RRBNO055_POWER_MODE_NORMAL))
  {
    std::cerr << "[Bno055Device] could not set power mode" << std::endl;
    hw_ = nullptr;
    return false;
  }

  // set orientation.
  if (set_op_mode(RRBNO055_OPERATION_MODE_CONFIG))
  {
    if (!set_axis_remap(conf->axis_remap, conf->axis_sign))
    {
      std::cerr << "[Bno055Device] could not set intended orientation" << std::endl;
      hw_ = nullptr;
      return false;
    }
  }
  else
  {
    std::cerr << "[Bno055Device] unable to enter configuration mode" << std::endl;
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
    // TODO: use set_power_mode
    if (!set_power_mode(RrBnoPowerMode::RRBNO055_POWER_MODE_LOWPOWER))
    {
      std::cerr << "[Bno055Device] unable to set device to low power mode" << std::endl;
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
    std::cerr << "[Bno055Device] initilize has not be ran" << std::endl;
    return false;
  }

  BNO055_RETURN_FUNCTION_TYPE rv = BNO055_ERROR;
  if ((rv = bno055_set_operation_mode(mode)) == BNO055_SUCCESS)
  {
    hw_->delay_msec(OP_MODE_SWITCH_DELAY[mode]);
    uint8_t actual_mode = 0;
    if (bno055_get_operation_mode(&actual_mode) != BNO055_SUCCESS)
    {
      std::cerr << "[Bno055Device] set_op_mode: could not read back mode from hardware" << std::endl;
      return false;
    }
    if (actual_mode != static_cast<uint8_t>(current_op_mode_))
    {
      std::cerr << "[Bno055Device] set_op_mode: mode mismatch — intended " << (int)current_op_mode_ << " got "
                << (int)actual_mode << std::endl;
      return false;
    }
    return true;
  }
  switch (rv)
  {
    case BNO055_E_NULL_PTR:
      std::cerr << "[Bno055Device] BNO055_E_NULL_PTR: internal pointer is not set" << std::endl;
      return false;
    case BNO055_ERROR:
      std::cerr << "[Bno055Device] BNO055_ERROR general error condition set" << std::endl;
      return false;
    default:
      std::cerr << "[Bno055Device] could not set operation unknown mode." << std::endl;
  }

  return false;
}

bool Bno055Device::set_power_mode(RrBnoPowerMode mode)
{
  // BNO055_E_NULL_PTR
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] initilize has not be ran" << std::endl;
    return false;
  }

  BNO055_RETURN_FUNCTION_TYPE rv = BNO055_ERROR;
  if ((rv = bno055_set_power_mode(mode)) == BNO055_SUCCESS)
  {
    hw_->delay_msec(PWR_MODE_SWITCH_DELAY[mode]);
    return true;
  }
  switch (rv)
  {
    case BNO055_E_NULL_PTR:
      std::cerr << "[Bno055Device] BNO055_E_NULL_PTR: internal pointer is not set" << std::endl;
      return false;
    case BNO055_ERROR:
      std::cerr << "[Bno055Device] BNO055_ERROR general error condition set" << std::endl;
      return false;
    default:
      std::cerr << "[Bno055Device] could not set operation unknown mode." << std::endl;
  }

  return false;
}

bool Bno055Device::reset()
{
  // since the object has not been initlized it does not need to be reset.
  if (!hw_->is_initilized())
  {
    std::cerr << "[Bno055Device] initilize has not be ran" << std::endl;
    return true;
  }
  BNO055_RETURN_FUNCTION_TYPE com_rslt = bno055_set_sys_rst(BNO055_BIT_ENABLE);
  switch (com_rslt)
  {
    case BNO055_SUCCESS:
      break;  // fall through to delay + re-init
    case BNO055_E_NULL_PTR:
      std::cerr << "[Bno055Device] BNO055_E_NULL_PTR: internal pointer is not set" << std::endl;
      return false;
    case BNO055_ERROR:
      std::cerr << "[Bno055Device] BNO055_ERROR general error condition set" << std::endl;
      return false;
    default:
      std::cerr << "[Bno055Device] could not set operation unknown mode." << std::endl;
      return false;
  }
  // BNO055 datasheet: ~650ms to reboot after reset
  hw_->delay_msec(650);
  // Sensor comes up in CONFIG mode, re-apply your operating mode
  BNO055_RETURN_FUNCTION_TYPE init_rslt = bno055_init(&device_);
  if (init_rslt != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] bno055_init failed after reset: " << (int)init_rslt << std::endl;
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
bool Bno055Device::set_axis_remap(RrBno055AxisRemap remap, RrBno055AxisSign sign)
{
  if (bno055_set_axis_remap_value(remap) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap: failed to set remap value" << std::endl;
    return false;
  }
  if (bno055_set_remap_z_sign(sign) != BNO055_SUCCESS)
  {
    std::cerr << "[Bno055Device] set_axis_remap: failed to set remap sign" << std::endl;
    return false;
  }
  return true;
}
