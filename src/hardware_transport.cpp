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
  // TODO: this is to be moved to the device.
  // device_.bus_write = bus_write_tmpl;
  // device_.bus_read = bus_read_tmpl;
  // device_.delay_msec = delay_msec;
  transport_ = -1;
}

HardwareTransport::~HardwareTransport()
{
  deinitialize();
}


// Note initlisation for hardware transport is performed within the factory
// this should check for the existence of the shared object and if it exits and
// is_initilised returns true, then no further action is required.
//
// Note that factory may need to be its own node, this is it can be shared
// with other hardware drivers.
void HardwareTransport::initialize(std::shared_ptr<TransportConfig> transport_config)
{
  if (is_initialized_.load(std::memory_order_acquire))
  {
    throw std::runtime_error("[HardwareTransport] already configured");
  }

  transport_ = initialize_trans(transport_config);

  if (transport_ == -1)
  {
    throw std::runtime_error("[HardwareTransport] could not open transport.");
  }
  instance_ = this;
  is_initialized_.store(true, std::memory_order_release);
}

// TODO this needs to change, there should not ba specific bno050 command in here, I think this will
// bno055_set_power_mode(BNO055_POWER_MODE_LOWPOWER) needs to be higher level up, 
void HardwareTransport::deinitialize()
{
  if (transport_ != -1)
  {
    close(transport_);
  }
  transport_ = -1;

  instance_ = nullptr;
  is_initialized_.store(false, std::memory_order_release);
}

void HardwareTransport::delay_msec(uint32_t msec)
{
  if (!instance_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(instance_->bus_mutex_);
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

int8_t HardwareTransport::bus_read_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (!instance_)
  {
    return -1;
  }
  std::lock_guard<std::mutex> lock(instance_->bus_mutex_);
  return instance_->bus_read(dev_addr, reg_addr, data, len);
}

int8_t HardwareTransport::bus_write_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (!instance_)
  {
    return -1;
  }
  std::lock_guard<std::mutex> lock(instance_->bus_mutex_);
  return instance_->bus_write(dev_addr, reg_addr, data, len);
}

