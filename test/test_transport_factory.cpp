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

// ── I2C ──────────────────────────────────────────────────────────────────────

TEST(TransportFactoryI2CTest, InvalidAddressThrows)
{
  // Address 0x30 is not a valid BNO055 address — initialize_trans returns -1,
  // so create_transport must throw.
  TransportConfig cfg;
  cfg.address = 0x30;

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

TEST(TransportFactoryI2CTest, Address0x28WithBadDeviceThrows)
{
  // 0x28 passes address validation; open() fails on the nonexistent device,
  // so create_transport must still throw.
  TransportConfig cfg;
  cfg.address = 0x28;
  cfg.device  = "/dev/nonexistent-i2c";

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

TEST(TransportFactoryI2CTest, Address0x29WithBadDeviceThrows)
{
  TransportConfig cfg;
  cfg.address = 0x29;
  cfg.device  = "/dev/nonexistent-i2c";

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

TEST(TransportFactoryI2CTest, AddressBelowRangeThrows)
{
  TransportConfig cfg;
  cfg.address = 0x00;

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

TEST(TransportFactoryI2CTest, AddressAboveRangeThrows)
{
  TransportConfig cfg;
  cfg.address = 0xFF;

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

// ── UART ─────────────────────────────────────────────────────────────────────

TEST(TransportFactoryUARTTest, NonexistentDeviceThrows)
{
  TransportConfig cfg;
  cfg.type   = UART;
  cfg.device = "/dev/nonexistent-uart";

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

// ── Dispatch ─────────────────────────────────────────────────────────────────

TEST(TransportFactoryDispatchTest, I2CTypeCallsI2CPath)
{
  // Invalid address triggers the I2C address guard, confirming the I2C path.
  TransportConfig cfg;
  cfg.type    = I2C;
  cfg.address = 0x01;  // invalid

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}

TEST(TransportFactoryDispatchTest, UARTTypeCallsUARTPath)
{
  // Nonexistent device causes open() to fail, confirming the UART path.
  TransportConfig cfg;
  cfg.type   = UART;
  cfg.device = "/dev/nonexistent-uart";

  TransportFactory factory;
  EXPECT_THROW(factory.get_or_create_transport(cfg), std::runtime_error);
}
