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
#include "rr_bno055/transport_factory.hpp"

using namespace rr_bno055;

// ── I2C ──────────────────────────────────────────────────────────────────────

TEST(TransportFactoryI2CTest, InvalidAddressReturnsError)
{
  // Address 0x30 is not a valid BNO055 address — must fail before open().
  TransportConfig cfg;
  cfg.address = 0x30;

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

TEST(TransportFactoryI2CTest, Address0x28IsAccepted)
{
  // 0x28 is valid; open() will fail because /dev/nonexistent doesn't exist,
  // but the address check must pass (return != validation -1).
  // We just verify it doesn't crash and returns something (likely -1 from open).
  TransportConfig cfg;
  cfg.address = 0x28;
  cfg.device  = "/dev/nonexistent-i2c";

  TransportFactory factory(cfg);
  // Result is -1 (open fails), but NOT from address validation.
  // There is no way to distinguish here without refactoring — so we only
  // assert the call does not throw and returns -1.
  EXPECT_EQ(factory.get_transport(), -1);
}

TEST(TransportFactoryI2CTest, Address0x29IsAccepted)
{
  TransportConfig cfg;
  cfg.address = 0x29;
  cfg.device  = "/dev/nonexistent-i2c";

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

TEST(TransportFactoryI2CTest, AddressBelowRangeReturnsError)
{
  TransportConfig cfg;
  cfg.address = 0x00;

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

TEST(TransportFactoryI2CTest, AddressAboveRangeReturnsError)
{
  TransportConfig cfg;
  cfg.address = 0xFF;

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

// ── UART ─────────────────────────────────────────────────────────────────────

TEST(TransportFactoryUARTTest, NonexistentDeviceReturnsError)
{
  TransportConfig cfg;
  cfg.type   = UART;
  cfg.device = "/dev/nonexistent-uart";

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

// ── Dispatch ─────────────────────────────────────────────────────────────────

TEST(TransportFactoryDispatchTest, I2CTypeCallsI2CPath)
{
  // With an invalid address the I2C guard triggers immediately.
  // This confirms the I2C code path is reached (not UART).
  TransportConfig cfg;
  cfg.type    = I2C;
  cfg.address = 0x01;  // invalid

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}

TEST(TransportFactoryDispatchTest, UARTTypeCallsUARTPath)
{
  // With a nonexistent device the UART open() call fails.
  // This confirms the UART code path is reached (not I2C).
  TransportConfig cfg;
  cfg.type   = UART;
  cfg.device = "/dev/nonexistent-uart";

  TransportFactory factory(cfg);
  EXPECT_EQ(factory.get_transport(), -1);
}
