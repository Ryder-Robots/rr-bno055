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
#include "rr_bno055/bno055_transport_config.hpp"

using namespace rr_bno055;

// ── Defaults ─────────────────────────────────────────────────────────────────

TEST(RrBNO055ConfigTest, DefaultAxisRemapIsXY)
{
  auto cfg = RrBNO055Config::Builder{}.build();
  EXPECT_EQ(cfg.axis_remap, RRBNO055_REMAP_X_Y);
}

TEST(RrBNO055ConfigTest, DefaultAxisSignIsNegative)
{
  auto cfg = RrBNO055Config::Builder{}.build();
  EXPECT_EQ(cfg.axis_sign, RRBNO055_REMAP_AXIS_NEGATIVE);
}

TEST(RrBNO055ConfigTest, InheritsDefaultTransportFields)
{
  auto cfg = RrBNO055Config::Builder{}.build();
  EXPECT_EQ(cfg.type, I2C);
  EXPECT_EQ(cfg.device, "/dev/i2c-1");
  EXPECT_EQ(cfg.address, 0x28);
}

// ── Custom values ─────────────────────────────────────────────────────────────

TEST(RrBNO055ConfigTest, CustomAxisRemapIsStored)
{
  auto cfg = RrBNO055Config::Builder{}.with_axis_remap(RRBNO055_DEFAULT_AXIS).build();
  EXPECT_EQ(cfg.axis_remap, RRBNO055_DEFAULT_AXIS);
}

TEST(RrBNO055ConfigTest, CustomAxisSignIsStored)
{
  auto cfg = RrBNO055Config::Builder{}.with_axis_sign(RRBNO055_REMAP_AXIS_POSITIVE).build();
  EXPECT_EQ(cfg.axis_sign, RRBNO055_REMAP_AXIS_POSITIVE);
}

TEST(RrBNO055ConfigTest, AllCustomFieldsStoredTogether)
{
  // Build with non-chained calls: parent methods return TransportConfig::Builder&,
  // so chaining would lose access to with_axis_remap / with_axis_sign.
  RrBNO055Config::Builder b{};
  b.with_type(UART);
  b.with_device("/dev/ttyAMA0");
  b.with_address(0x29);
  b.with_axis_remap(RRBNO055_REMAP_Y_Z);
  b.with_axis_sign(RRBNO055_REMAP_AXIS_POSITIVE);
  auto cfg = b.build();

  EXPECT_EQ(cfg.type, UART);
  EXPECT_EQ(cfg.device, "/dev/ttyAMA0");
  EXPECT_EQ(cfg.address, 0x29);
  EXPECT_EQ(cfg.axis_remap, RRBNO055_REMAP_Y_Z);
  EXPECT_EQ(cfg.axis_sign, RRBNO055_REMAP_AXIS_POSITIVE);
}
