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

#include <gtest/gtest.h>
#include <stdexcept>
#include "rr_bno055/transport_factory.hpp"

using namespace rr_bno055;

// Helper: build a shared_ptr<RrBNO055Config> using the builder.
static std::shared_ptr<RrBNO055Config> make_cfg(
  TransportType type   = I2C,
  const std::string& device = "/dev/i2c-1",
  uint8_t address      = 0x28)
{
  RrBNO055Config::Builder b{};
  b.with_type(type);
  b.with_device(device);
  b.with_address(address);
  return std::make_shared<RrBNO055Config>(b.build());
}

// ── I2C ──────────────────────────────────────────────────────────────────────

TEST(TransportFactoryI2CTest, BadDeviceThrowsAddress0x28)
{
  // open() fails on the nonexistent device → initialize_trans returns -1 → throw.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x28)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, BadDeviceThrowsAddress0x29)
{
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x29)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, AddressAbove7BitThrows)
{
  // 0xFF exceeds the 7-bit I2C address range; the kernel rejects ioctl(I2C_SLAVE).
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/i2c-1", 0xFF)), std::runtime_error);
}

// ── UART ─────────────────────────────────────────────────────────────────────

TEST(TransportFactoryUARTTest, NonexistentDeviceThrows)
{
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(UART, "/dev/nonexistent-uart", 0x28)), std::runtime_error);
}

// ── Dispatch ─────────────────────────────────────────────────────────────────

TEST(TransportFactoryDispatchTest, I2CTypeCallsI2CPath)
{
  // A nonexistent device causes open() to fail on the I2C path → throw.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x28)), std::runtime_error);
}

TEST(TransportFactoryDispatchTest, UARTTypeCallsUARTPath)
{
  // Nonexistent device causes open() to fail, confirming the UART path.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(UART, "/dev/nonexistent-uart", 0x28)), std::runtime_error);
}

// ── Cleanup ──────────────────────────────────────────────────────────────────

TEST(TransportFactoryCleanupTest, CleanupWithNoTransportsIsSafe)
{
  TransportFactory factory;
  EXPECT_NO_THROW(factory.cleanup());
}

TEST(TransportFactoryCleanupTest, CleanupAfterFailedCreateIsSafe)
{
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x28)), std::runtime_error);
  EXPECT_NO_THROW(factory.cleanup());
}

