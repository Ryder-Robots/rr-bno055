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

#include "rr_bno055/hardware_transport.hpp"

using namespace rr_bno055;

HardwareTransport::HardwareTransport()
{
  device_.bus_write = bus_write_tmpl;
  device_.bus_read = bus_read_tmpl;
  device_.delay_msec = delay_msec;
  transport_ = -1;
}

HardwareTransport::~HardwareTransport()
{
  deinitialize();
}

void HardwareTransport::initialize(const TransportConfig& transport_config)
{
  if (is_initialized_.load(std::memory_order_acquire))
  {
    throw std::runtime_error("[HardwareTransport] already configured");
  }

  transport_ = initialize_trans(transport_config);

  if (transport_ == -1)
  {
    throw std::runtime_error("[HardwareTransport] could not create transport configuration error.");
  }
  instance_ = this;

  device_.bus_write = bus_write_tmpl;
  device_.bus_read = bus_read_tmpl;
  device_.delay_msec = delay_msec;

  if (bno055_init(&device_) != 0) 
  {
    instance_ = nullptr;
    throw std::runtime_error("[HardwareTransport] could not initlize IMU");
  }

  //set power mode to normal.
  if (bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != 0)
  {
    instance_ = nullptr;
    throw std::runtime_error("[HardwareTransport] could not set power mode");
  }

  is_initialized_.store(true, std::memory_order_release);
}

void HardwareTransport::deinitialize()
{
  if (transport_ != -1)
  {
    close(transport_);
  }
  transport_ = -1;
  
  // attempt to set low power mode, follow through with deinitilization, but warn the user.
  if (bno055_set_power_mode(BNO055_POWER_MODE_LOWPOWER) != 0)
  {
    std::cerr << "[HardwareTransport] unable to set device to low power mode" << std::endl;
  }

  instance_ = nullptr;
  is_initialized_.store(false, std::memory_order_release);
}

void HardwareTransport::delay_msec(uint32_t msec)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

int8_t HardwareTransport::bus_read_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (!instance_)
  {
    return -1;
  }
  return instance_->bus_read(dev_addr, reg_addr, data, len);
}

int8_t HardwareTransport::bus_write_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (!instance_)
  {
    return -1;
  }
  return instance_->bus_write(dev_addr, reg_addr, data, len);
}

bno055_t HardwareTransport::get_device()
{
  return device_;
}
