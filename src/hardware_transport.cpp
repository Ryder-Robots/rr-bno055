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

#include "rr_bno055/hardware_transport.hpp"

using namespace rr_bno055;

HardwareTransport::HardwareTransport()
{
  instance_ = this;
  device_.bus_write = bus_write_tmpl;
  device_.bus_read = bus_read_tmpl;
  device_.bus_write = bus_write_tmpl;
}

void HardwareTransport::initialize(const TransportConfig transport_config)
{
  {
    TransportFactory fact(transport_config);
    transport_ = fact.get_transport();

    if (transport_ == -1)
    {
      throw std::runtime_error("[HardwareTransport] could not create transport configuration error.");
    }
  }
}

void HardwareTransport::deinitialize()
{
  close(transport_);
}

void HardwareTransport::delay_msec(uint32_t msec)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

int8_t HardwareTransport::bus_read_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  return instance_->bus_read(dev_addr, reg_addr, data, len);
}

int8_t HardwareTransport::bus_write_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  return instance_->bus_write(dev_addr, reg_addr, data, len);
}

int8_t HardwareTransport::bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;
  if (write(transport_, &reg_addr, 1) != 1)
  {
    return -1;
  }

  // Then read back len bytes into data buffer
  if (read(transport_, data, len) != len)
  {
    return -1;
  }

  return 0;
}

int8_t HardwareTransport::bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;

  // Keep a constant here, 256 is the maximum uint8 can be and it is small enough just to allocate.
  uint8_t buf[256];
  buf[0] = reg_addr;
  std::memcpy(&buf[1], data, len);

  if (write(transport_, buf, len + 1) != len + 1)
  {
    return -1;  // BNO055_ERROR
  }
  return 0;  // BNO055_SUCCESS
}