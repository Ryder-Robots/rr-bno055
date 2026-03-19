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

#include <memory>
#include <array>
#include <mutex>
#include "rr_bno055/hardware_transport.hpp"
#include "rr_bno055/i2c_hardware_transport.hpp"
#include "rr_bno055/uart_hardware_transport.hpp"
#include "rr_bno055/bno055_transport_config.hpp"

namespace rr_bno055
{
/**
 * @brief Sensor-agnostic factory for creating and caching HardwareTransport instances.
 *
 * Creates an I2C or UART transport from a `TransportConfig` and caches it
 * as a `std::weak_ptr` so that subsequent calls with the same transport type
 * return the existing instance if it is still alive.
 *
 * The factory carries no sensor-specific knowledge; it can be shared across
 * multiple sensor device classes (e.g. `Bno055Device` and future sensors)
 * without modification.
 */
class TransportFactory
{
public:
  TransportFactory() = default;
  virtual ~TransportFactory() = default;

  /**
   * @brief Returns an initialised transport for the bus type in @p config,
   * creating one if none is currently alive.
   *
   * The factory caches each transport as a `std::weak_ptr` keyed by
   * `TransportType`.  If the cached instance is still alive (i.e. at least
   * one `shared_ptr` to it exists elsewhere), it is returned directly.
   * Otherwise a new instance is created, initialised, and cached.
   *
   * Thread-safe: guarded by an internal mutex.
   *
   * @param config  Bus configuration.  Only `config->type` and `config->device`
   *                are used by the factory itself; the rest is forwarded to
   *                the transport's `initialize()`.
   * @return        Shared pointer to the initialised transport.
   * @throws std::runtime_error if the transport cannot be opened.
   */
  std::shared_ptr<HardwareTransport> get_or_create_transport(std::shared_ptr<RrBNO055Config> config);

  /**
   * @brief Releases all cached weak_ptr references.
   *
   * Does not forcibly close any transport that is still held by a
   * `shared_ptr` elsewhere — those will close when their last owner
   * releases them.  Call this during orderly shutdown after
   * `Bno055Device::deinitialize()`.
   */
  void cleanup();

private:
  std::array<std::weak_ptr<HardwareTransport>, 2> hws_;
  std::mutex mutex_;

};

}  // namespace rr_bno055
