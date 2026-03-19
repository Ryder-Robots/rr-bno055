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

/**
 * @brief Selects the physical bus protocol used to communicate with the sensor.
 *
 * Wire the BNO055 protocol-select pins to match:
 *   I2C  — PS1=GND, PS0=GND
 *   UART — PS1=GND, PS0=3.3 V
 */
enum TransportType
{
  I2C,   ///< I2C bus (PS1=0, PS0=0)
  UART,  ///< UART / serial (PS1=0, PS0=1)
};

/**
 * @brief Immutable configuration describing a physical bus transport.
 *
 * Holds the bus type, device node path, and I2C address for a single
 * sensor connection.  Constructed exclusively through `Builder`.
 *
 * @note This class is sensor-agnostic.  Sensor-specific configuration
 * (axis remap, sign, etc.) belongs in a subclass such as `RrBNO055Config`.
 */
class TransportConfig
{
public:
  const TransportType type;    ///< Bus protocol (I2C or UART).
  const std::string   device;  ///< Device node, e.g. `/dev/i2c-1` or `/dev/ttyAMA0`.
  const uint8_t       address; ///< I2C slave address (ignored for UART).

  /**
   * @brief Fluent builder for `TransportConfig`.
   *
   * Defaults: I2C, `/dev/i2c-1`, address `0x28`.
   *
   * Subclass builders (e.g. `RrBNO055Config::Builder`) inherit these methods.
   * Note that parent methods return `TransportConfig::Builder&`, so calling
   * a parent method in a chain loses the subclass type.  Use non-chained calls
   * when mixing parent and child builder methods:
   * @code
   *   RrBNO055Config::Builder b{};
   *   b.with_device("/dev/i2c-1");
   *   b.with_address(0x29);
   *   b.with_axis_remap(RRBNO055_REMAP_Y_Z);
   *   auto cfg = b.build();
   * @endcode
   */
  class Builder
  {
  public:
    /// Sets the bus protocol. Default: `I2C`.
    Builder& with_type(TransportType type)
    {
      type_ = type;
      return *this;
    }

    /// Sets the device node path. Default: `/dev/i2c-1`.
    Builder& with_device(std::string device)
    {
      device_ = std::move(device);
      return *this;
    }

    /// Sets the I2C slave address. Default: `0x28`. Ignored for UART.
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
    TransportType type_    = I2C;
    std::string   device_  = "/dev/i2c-1";
    uint8_t       address_ = 0x28;
  };

protected:
  TransportConfig(TransportType type, std::string device, uint8_t address)
    : type(type), device(std::move(device)), address(address)
  {
  }
};

}  // namespace rr_bno055
