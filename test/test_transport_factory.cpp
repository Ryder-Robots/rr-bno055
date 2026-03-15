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

TEST(TransportFactoryI2CTest, InvalidAddressThrows)
{
  // Address 0x30 is not a valid BNO055 address — initialize_trans returns -1,
  // so create_transport must throw.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/i2c-1", 0x30)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, Address0x28WithBadDeviceThrows)
{
  // 0x28 passes address validation; open() fails on the nonexistent device,
  // so create_transport must still throw.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x28)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, Address0x29WithBadDeviceThrows)
{
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/nonexistent-i2c", 0x29)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, AddressBelowRangeThrows)
{
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/i2c-1", 0x00)), std::runtime_error);
}

TEST(TransportFactoryI2CTest, AddressAboveRangeThrows)
{
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
  // Invalid address triggers the I2C address guard, confirming the I2C path.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(I2C, "/dev/i2c-1", 0x01)), std::runtime_error);
}

TEST(TransportFactoryDispatchTest, UARTTypeCallsUARTPath)
{
  // Nonexistent device causes open() to fail, confirming the UART path.
  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(make_cfg(UART, "/dev/nonexistent-uart", 0x28)), std::runtime_error);
}
