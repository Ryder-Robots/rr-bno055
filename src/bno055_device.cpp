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

bool Bno055Device::initialize(const TransportConfig& conf, std::shared_ptr<HardwareTransport> hw)
{
  // verify that the device has a valid address assigned.
  if (conf.device.empty() || !(conf.address == 0x28 || conf.address == 0x29))
  {
    std::cerr << "[Bno055Device] Failed to open device: no device defined or address not one of 28 or 29" << std::endl;
    return false;
  }
  if (!(hw && hw_->is_initilized()))
  {
    std::cerr << "[Bno055Device] hardware transport is not initilized" << std::endl;
    return false;
  }

  if (bno055_init(&device_) != 0)
  {
    std::cerr << "[Bno055Device] could not initialize IMU" << std::endl;
    return false;
  }

  // set power mode to normal.
  if (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != 0)
  {
    std::cerr << "[Bno055Device] could not set power mode" << std::endl;
    return false;
  }
  hw_ = hw;

  // TODO: this becomes part of the hardware factories responsibilities.
  // // if the transport is already inilized then exit here.
  // if (hw_)
  // {
  //   if (hw_->is_initilized())
  //   {
  //     return true;
  //   }
  // }
  // else
  // {
  //   TransportFactory fact;
  //   hw_ = fact.create_transport(conf);
  // }

  // // wrap in a try catch
  // try
  // {
  //   hw_->initialize(conf);
  // }
  // catch (const std::runtime_error& e)
  // {
  //   std::cerr << "[Bno055Device] unable to initialise IMU" << std::endl;
  //   return false;
  // }

  return true;
}

void Bno055Device::deinitialize()
{
  if (hw_ && hw_->is_initilized())
  {
    // Attempt to set low power mode; continue deinitialization even if this fails.
    if (bno055_set_power_mode(BNO055_POWER_MODE_LOWPOWER) != 0)
    {
      std::cerr << "[Bno055Device] unable to set device to low power mode" << std::endl;
    }
  }
}

bool Bno055Device::set_mode(RrBno055OpMode mode)
{
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
