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

#include "rr_bno055/transport_factory.hpp"

using namespace rr_bno055;

std::shared_ptr<HardwareTransport> TransportFactory::get_or_create_transport(const TransportConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::shared_ptr<HardwareTransport> hw;
  // attempt to get shared object first.
  if (hw = hws_[config.type].lock())
  {
    if (hw->is_initilized()) {
      return hw;
    }
    // else: transport exists but uninitialized, fall through and recreate
  }

  // initialize new object, keep them on separate buses.
  switch (config.type)
  {
    case I2C:
      hw = std::make_shared<I2CHardwareTransport>();
      hw->initialize(config);
      hws_[I2C] = hw;
      return hw;
    case UART:
      hw = std::make_shared<UARTHardwareTransport>();
      hw->initialize(config);
      hws_[UART] = hw;
      return hw;
    default:
      throw std::runtime_error("[TransportFactory] non supported transport");
  }
}

void TransportFactory::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& h : hws_) {
    if (auto hw = h.lock()) {
      hw->deinitialize();
    }
    h.reset();
  }
}
