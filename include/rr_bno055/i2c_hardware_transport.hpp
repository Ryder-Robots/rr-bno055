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

#include "rr_bno055/hardware_transport.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstring>

namespace rr_bno055
{

/**
 * @brief Linux I2C transport for Bosch IMU sensors.
 *
 * Opens `/dev/i2c-N` via the Linux I2C character device interface and
 * implements `bus_read` / `bus_write` using `ioctl(I2C_SLAVE)` followed by
 * standard `read()` / `write()` calls.
 *
 * The I2C slave address is set on every transaction via `ioctl(I2C_SLAVE,
 * dev_addr)`, so the same file descriptor can serve multiple addresses if
 * needed.
 *
 * @note Requires the calling process to have read/write permission on the
 * I2C device node.  On Raspberry Pi / Ubuntu, add the user to the `i2c`
 * group or run as root.
 */
class I2CHardwareTransport : public HardwareTransport
{
public:
  /**
   * @brief Opens the I2C character device described by @p transport_config.
   *
   * @param transport_config  Must have `type == I2C` and a valid `device`
   *                          path (e.g. `/dev/i2c-1`).
   * @return  A valid file descriptor on success.
   * @throws  std::runtime_error if the device cannot be opened.
   */
  int initialize_trans(std::shared_ptr<TransportConfig> transport_config) override;

  /**
   * @brief Reads @p len bytes from register @p reg_addr into @p data.
   *
   * Issues `ioctl(I2C_SLAVE, dev_addr)` to select the target device, writes
   * the register address, then reads the response bytes.
   *
   * @return 0 on success, -1 on any I2C error.
   */
  int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) override;

  /**
   * @brief Writes @p len bytes from @p data to register @p reg_addr.
   *
   * Issues `ioctl(I2C_SLAVE, dev_addr)` to select the target device, then
   * writes the register address followed by the data bytes in a single call.
   *
   * @return 0 on success, -1 on any I2C error.
   */
  int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) override;
};
}  // namespace rr_bno055