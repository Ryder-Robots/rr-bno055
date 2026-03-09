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
#include <linux/i2c-dev.h>

namespace rr_bno055
{

/// Selects the physical interface used to communicate with the BNO055.
enum TransportType
{
  I2C,   ///< I2C bus (PS1=0, PS0=0 on BNO055)
  UART,  ///< UART / serial (PS1=0, PS0=1 on BNO055)
};

/**
 * @brief Configuration passed to `TransportFactory` and `HardwareTransport::initialize()`.
 *
 * Defaults are suitable for a BNO055 breakout wired to the Raspberry Pi
 * primary I2C bus with the address pin (ADR/COM3) pulled low.
 */
struct TransportConfig
{
  TransportType type = I2C;           ///< Interface type.
  std::string device = "/dev/i2c-1";  ///< Device node to open.

  /// I2C address of the BNO055.  Pull ADR/COM3 high to use 0x29 instead.
  uint8_t address = 0x28;
};

/**
 * @brief Opens and configures the transport file descriptor for the BNO055.
 *
 * Instantiate with a `TransportConfig`, then call `get_transport()` to
 * receive a ready-to-use file descriptor.  The caller is responsible for
 * closing it via `HardwareTransport::deinitialize()`.
 */
class TransportFactory
{
public:
  explicit TransportFactory(const TransportConfig& config) : config_(config)
  {
  }
  ~TransportFactory() = default;

  /**
   * @brief Opens the transport described by the stored config.
   * @return A valid file descriptor on success, or -1 on failure.
   */
  int get_transport();

private:
  /// Opens and configures an I2C file descriptor.
  int get_i2c_transport();

  /// Opens and configures a UART file descriptor.
  int get_uart_transport();

  TransportConfig config_;
};

}  // namespace rr_bno055
