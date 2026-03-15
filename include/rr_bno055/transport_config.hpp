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

namespace rr_bno055
{

enum TransportType
{
  I2C,   ///< I2C bus (PS1=0, PS0=0 on BNO055)
  UART,  ///< UART / serial (PS1=0, PS0=1 on BNO055)
};

class TransportConfig
{
public:
  const TransportType type;
  const std::string device;
  const uint8_t address;

  class Builder
  {
  public:
    Builder& with_type(TransportType type)
    {
      type_ = type;
      return *this;
    }
    Builder& with_device(std::string device)
    {
      device_ = std::move(device);
      return *this;
    }
    Builder& with_address(uint8_t address)
    {
      address_ = address;
      return *this;
    }

    TransportConfig build() const
    {
      return TransportConfig(type_, device_, address_);
    }

  protected:
    TransportType type_ = I2C;
    std::string device_ = "/dev/i2c-1";
    uint8_t address_ = 0x28;
  };

protected:
  TransportConfig(TransportType type, std::string device, uint8_t address)
    : type(type), device(std::move(device)), address(address)
  {
  }
};

}  // namespace rr_bno055
