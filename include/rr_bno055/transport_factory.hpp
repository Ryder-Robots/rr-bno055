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
#include "rr_bno055/hardware_transport.hpp"
#include "rr_bno055/i2c_hardware_transport.hpp"
#include "rr_bno055/uart_hardware_transport.hpp"

namespace rr_bno055
{

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
  TransportFactory() = default;

  ~TransportFactory() = default;

  /**
   * @brief Opens the transport described by the stored config.
   * @return A valid file descriptor on success, or -1 on failure.
   */
  std::unique_ptr<HardwareTransport> create_transport(const TransportConfig& config);
};

}  // namespace rr_bno055
