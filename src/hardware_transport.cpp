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

  TransportFactory fact(transport_config);
  transport_ = fact.get_transport();

  if (transport_ == -1)
  {
    throw std::runtime_error("[HardwareTransport] could not create transport configuration error.");
  }
  instance_ = this;
  config_address_ = transport_config.address;
  is_initialized_.store(true, std::memory_order_release);
}

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


//TODO: These methods need to be moved to something that is returned by the factory,
// and not done here. They are different for UART and for I2C
int8_t HardwareTransport::bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;

  if (!is_initialized_.load(std::memory_order_acquire))
  {
    return -1;
  }

  struct i2c_msg msgs[2];

  msgs[0].addr = config_address_;  // stored I2C address
  msgs[0].flags = 0;               // write
  msgs[0].len = 1;
  msgs[0].buf = &reg_addr;

  // Message 2: read the response (repeated START, not a new START)
  msgs[1].addr = config_address_;
  msgs[1].flags = I2C_M_RD;  // read
  msgs[1].len = len;
  msgs[1].buf = data;

  struct i2c_rdwr_ioctl_data transfer;
  transfer.msgs = msgs;
  transfer.nmsgs = 2;
  if (ioctl(transport_, I2C_RDWR, &transfer) < 0)
  {
    return -1;
  }
  return 0;
}

int8_t HardwareTransport::bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;

  if (!is_initialized_.load(std::memory_order_acquire))
  {
    return -1;
  }

  // Keep a constant here, 256 is the maximum uint8 can be and it is small enough just to allocate.
  uint8_t buf[256];
  buf[0] = reg_addr;
  std::memcpy(&buf[1], data, len);

  struct i2c_msg msg;
  msg.addr = config_address_;
  msg.flags = 0;  // write
  msg.len = len + 1;
  msg.buf = buf;

  struct i2c_rdwr_ioctl_data transfer;
  transfer.msgs = &msg;
  transfer.nmsgs = 1;

  if (ioctl(transport_, I2C_RDWR, &transfer) < 0)
  {
    return -1;
  }
  return 0;
}