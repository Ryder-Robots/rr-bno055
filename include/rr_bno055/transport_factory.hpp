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

#include <string>
#include <cstdint>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <cstring>

namespace rr_bno055
{

enum TransportType {
  I2C,
  UART,
};

/**
 * The following defaults should be pretty sensible for most cases.
 * Assumes I2C interface.
 */
struct TransportConfig
{
  TransportType type = I2C;
  std::string device = "/dev/i2c-1";

  // To change thge address to 0x29 Address pin must be wired and set high.
  uint8_t address = 0x28;  // BNO055 default I2C address
};

class TransportFactory
{
public:
  TransportFactory(const TransportConfig config) : config_(config)
  {
  }
  ~TransportFactory() = default;
  int get_transport();

protected:
  int get_i2c_transport();

  int get_uart_transport();

private:
  TransportConfig config_;
};

}  // namespace rr_bno055
