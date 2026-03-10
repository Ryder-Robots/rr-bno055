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

#include "rr_bno055/i2c_hardware_transport.hpp"

using namespace rr_bno055;

int I2CHardwareTransport::initialize_trans(const TransportConfig& transport_config)
{
  if (transport_config.device.empty() || !(transport_config.address == 0x28 || transport_config.address == 0x29))
  {
    std::cerr << "[TransportFactory] Failed to open device: no device defined or address not one of 28 or 29";
    return -1;
  }

  int transport = open(transport_config.device.c_str(), O_RDWR);
  if (transport != -1)
  {
    if (ioctl(transport, I2C_SLAVE, transport_config.address))
    {
      std::cerr << "[TransportFactory] unable to create I2C slave: " << strerror(errno);
      close(transport);
      return -1;
    }
  }
  else
  {
    std::cerr << "[TransportFactory] could not open device, verify that device string is correct: " << strerror(errno);
  }

  config_address_ = transport_config.address;
  return transport;
}

int8_t I2CHardwareTransport::bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (!is_initialized_.load(std::memory_order_acquire))
  {
    return -1;
  }

  struct i2c_msg msgs[2];

  msgs[0].addr = dev_addr;
  msgs[0].flags = 0;  // write
  msgs[0].len = 1;
  msgs[0].buf = &reg_addr;

  // Message 2: read the response (repeated START, not a new START)
  msgs[1].addr = dev_addr;
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

int8_t I2CHardwareTransport::bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
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