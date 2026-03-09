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

#include "bno055.h"
#include <thread>
#include <cstdint>
#include <string>
#include "rr_bno055/transport_factory.hpp"
#include <stdexcept>
#include <atomic>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

namespace rr_bno055
{

/**
 * @brief Low-level I2C/UART transport adapter for the Bosch BNO055 IMU.
 *
 * Implements the bus read/write function pointers required by the Bosch
 * BNO055 SensorAPI (`bno055_t::bus_read` / `bno055_t::bus_write`).
 *
 * A single instance is tracked via `instance_` so that the static template
 * callbacks required by the C API can dispatch to the correct object.
 *
 * ## Hardware connection (I2C, default)
 *
 * | BNO055 pin | Raspberry Pi pin       | Notes                          |
 * |------------|------------------------|--------------------------------|
 * | VCC        | 3.3 V  (pin 1)         | Do **not** use 5 V             |
 * | GND        | GND    (pin 6)         |                                |
 * | SDA        | GPIO 2 / SDA (pin 3)   |                                |
 * | SCL        | GPIO 3 / SCL (pin 5)   |                                |
 * | PS0        | GND                    | Protocol select: PS1=0, PS0=0 → I2C |
 * | PS1        | GND                    |                                |
 * | ADR / COM3 | GND (addr 0x28)        | Pull high for addr 0x29        |
 *
 * The default transport opens `/dev/i2c-1` at address `0x28`.
 * Enable I2C on the Pi with `sudo raspi-config` → Interface Options → I2C.
 *
 * Note that this class should only be called using hardware_transport,  it is not
 * threadsafe.
 */
class HardwareTransport
{
public:
  /**
   * @brief Constructs the transport and wires up the Bosch SensorAPI callbacks.
   *
   * Sets the singleton `instance_` pointer and assigns `bus_read_tmpl` /
   * `bus_write_tmpl` into `device_`.  Call `initialize()` before performing
   * any I/O.
   */
  HardwareTransport();

  ~HardwareTransport();

  /**
   * @brief Blocking delay used by the Bosch SensorAPI during sensor init.
   * @param msec Duration in milliseconds.
   */
  static void delay_msec(uint32_t msec);

  /**
   * @brief Opens the underlying transport (I2C or UART) described by @p config.
   *
   * Must be called before any bus_read / bus_write operations.
   * Throws `std::runtime_error` if the transport cannot be opened.
   *
   * @param transport_config  Device path, type, and I2C address to use.
   */
  void initialize(const TransportConfig& transport_config);

  /**
   * @brief Closes the transport file descriptor.
   */
  void deinitialize();

  /**
   * @brief Reads @p len bytes from register @p reg_addr into @p data.
   *
   * Performs a combined I2C write (register address) followed by a read.
   * @p dev_addr is ignored because the address is embedded in the open fd.
   *
   * @return 0 on success, -1 on error.
   */
  int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

  /**
   * @brief Writes @p len bytes from @p data to register @p reg_addr.
   *
   * Prepends the register address to the payload in a local 256-byte stack
   * buffer (safe because `len` is `uint8_t`, max 255).
   * @p dev_addr is ignored because the address is embedded in the open fd.
   *
   * @return 0 on success, -1 on error.
   */
  int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

private:
  /// Static trampoline into `instance_->bus_read()` for the Bosch C API.
  static int8_t bus_read_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

  /// Static trampoline into `instance_->bus_write()` for the Bosch C API.
  static int8_t bus_write_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

  std::atomic<bool> is_initialized_{ false };
  int transport_;    ///< Open file descriptor for the I2C or UART device.
  bno055_t device_;  ///< Bosch SensorAPI struct holding function pointers.

  inline static HardwareTransport* instance_ = nullptr;  ///< Singleton pointer set in constructor.

  uint8_t config_address_;
};

}  // namespace rr_bno055