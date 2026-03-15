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

TEST(TransportConfigTest, DefaultTypeIsI2C)
{
  auto cfg = TransportConfig::Builder{}.build();
  EXPECT_EQ(cfg.type, I2C);
}

TEST(TransportConfigTest, DefaultDeviceIsI2c1)
{
  auto cfg = TransportConfig::Builder{}.build();
  EXPECT_EQ(cfg.device, "/dev/i2c-1");
}

TEST(TransportConfigTest, DefaultAddressIs0x28)
{
  auto cfg = TransportConfig::Builder{}.build();
  EXPECT_EQ(cfg.address, 0x28);
}

TEST(TransportConfigTest, CustomValuesAreStored)
{
  auto cfg = TransportConfig::Builder{}
    .with_type(UART)
    .with_device("/dev/ttyAMA0")
    .with_address(0x29)
    .build();

  EXPECT_EQ(cfg.type,    UART);
  EXPECT_EQ(cfg.device,  "/dev/ttyAMA0");
  EXPECT_EQ(cfg.address, 0x29);
}
