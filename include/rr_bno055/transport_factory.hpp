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
   * @brief Opens the transport described by the stored config.
   * @return Returns pointer to Hardware Transport.
   */
  std::shared_ptr<HardwareTransport> get_or_create_transport(std::shared_ptr<RrBNO055Config> config);
  void cleanup();

private:
  std::array<std::weak_ptr<HardwareTransport>, 2> hws_;
  std::mutex mutex_;

};

}  // namespace rr_bno055
