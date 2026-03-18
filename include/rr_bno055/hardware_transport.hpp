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

#include <thread>
#include <string>
#include <stdexcept>
#include <iostream>
#include <atomic>
#include <unistd.h>
#include <mutex>
#include <memory>
#include "rr_bno055/transport_config.hpp"

namespace rr_bno055
{

/**
 * @brief Sensor-agnostic I2C/UART transport adapter for Bosch IMU sensors.
 *
 * Provides the bus read/write function pointers required by Bosch SensorAPI
 * C libraries (e.g. `bno055_t::bus_read` / `bno055_t::bus_write`).  The
 * transport carries no sensor-specific knowledge; all sensor validation
 * (addresses, chip IDs, etc.) lives in the sensor device class.
 *
 * A single instance is tracked via `instance_` so that the static trampoline
 * callbacks required by C APIs can dispatch to the correct object.
 *
 * Only one instance may exist at a time.  Copy and move are deleted to
 * enforce this constraint.
 */
class HardwareTransport
{
public:
  // Force singleton.
  HardwareTransport(const HardwareTransport&) = delete;
  HardwareTransport& operator=(const HardwareTransport&) = delete;
  HardwareTransport(HardwareTransport&&) = delete;
  HardwareTransport& operator=(HardwareTransport&&) = delete;

  /**
   * @brief Constructs the transport and wires up the Bosch SensorAPI callbacks.
   *
   * Assigns `bus_read_tmpl` / `bus_write_tmpl` into `device_` so that the
   * Bosch SensorAPI can dispatch reads and writes through this object.
   * Call `initialize()` before performing any I/O.
   */
  HardwareTransport();

  virtual ~HardwareTransport();

  /**
   * @brief Blocking delay used by the Bosch SensorAPI during sensor init.
   * @param msec Duration in milliseconds.
   */
  static void delay_msec(uint32_t msec);

  /**
   * @brief Opens the transport, initialises the Bosch SensorAPI, and sets the
   * device to normal power mode.
   *
   * Calls `initialize_trans()` to open the physical bus, then `bno055_init()`
   * and `bno055_set_power_mode()`.  Throws `std::runtime_error` on any failure.
   * Must be called exactly once before any I/O.
   *
   * @param transport_config  Device path, type, and I2C address to use.
   */
  void initialize(std::shared_ptr<TransportConfig> transport_config);

  /**
   * @brief Opens the underlying transport (I2C or UART) described by @p config.
   *
   * Must be called before any bus_read / bus_write operations.
   * Throws `std::runtime_error` if the transport cannot be opened.
   *
   * @param transport_config  Device path, type, and I2C address to use.
   */
  virtual int initialize_trans(std::shared_ptr<TransportConfig> transport_config) = 0;

  /**
   * @brief Sets the device to low-power mode and closes the transport file descriptor.
   *
   * Subclasses that need additional teardown should override this method and
   * call `HardwareTransport::deinitialize()` at the end of their override.
   */
  virtual void deinitialize();

  /**
   * @brief Reads @p len bytes from register @p reg_addr into @p data.
   *
   * @p dev_addr is the Bosch SensorAPI device address; implementations may
   * ignore it if the address is already embedded in the open file descriptor.
   *
   * @return 0 on success, -1 on error.
   */
  virtual int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) = 0;

  /**
   * @brief Writes @p len bytes from @p data to register @p reg_addr.
   *
   * @p dev_addr is the Bosch SensorAPI device address; implementations may
   * ignore it if the address is already embedded in the open file descriptor.
   *
   * @return 0 on success, -1 on error.
   */
  virtual int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) = 0;

  bool is_initilized()
  {
    return is_initialized_.load(std::memory_order_acquire);
  }

  /// Function pointer types matching the Bosch SensorAPI C callback signatures.
  using BusReadFn  = int8_t (*)(uint8_t, uint8_t, uint8_t*, uint8_t);
  using BusWriteFn = int8_t (*)(uint8_t, uint8_t, uint8_t*, uint8_t);

  /// Returns the bus-read trampoline suitable for registration with a C sensor API.
  static BusReadFn  get_bus_read_fn()  { return bus_read_tmpl; }

  /// Returns the bus-write trampoline suitable for registration with a C sensor API.
  static BusWriteFn get_bus_write_fn() { return bus_write_tmpl; }

protected:
  std::atomic<bool> is_initialized_{ false };
  std::mutex bus_mutex_;

  int transport_;  ///< Open file descriptor for the I2C or UART device.

private:
  /// Static trampoline into `instance_->bus_read()` for the Bosch C API.
  static int8_t bus_read_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

  /// Static trampoline into `instance_->bus_write()` for the Bosch C API.
  static int8_t bus_write_tmpl(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len);

  inline static HardwareTransport* instance_ = nullptr;  ///< Singleton pointer set in constructor.
};

}  // namespace rr_bno055