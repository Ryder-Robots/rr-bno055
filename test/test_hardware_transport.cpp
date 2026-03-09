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
#include <chrono>
#include <stdexcept>
#include "rr_bno055/hardware_transport.hpp"

using namespace rr_bno055;

// ── Construction ─────────────────────────────────────────────────────────────

TEST(HardwareTransportTest, ConstructsWithoutThrowing)
{
  EXPECT_NO_THROW(HardwareTransport transport);
}

// ── delay_msec ───────────────────────────────────────────────────────────────

TEST(HardwareTransportTest, DelayMsecDoesNotThrow)
{
  EXPECT_NO_THROW(HardwareTransport::delay_msec(1));
}

TEST(HardwareTransportTest, DelayMsecTakesAtLeastRequestedTime)
{
  const uint32_t delay_ms = 10;
  auto start = std::chrono::steady_clock::now();
  HardwareTransport::delay_msec(delay_ms);
  auto elapsed = std::chrono::steady_clock::now() - start;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

  EXPECT_GE(elapsed_ms, static_cast<long>(delay_ms));
}

// ── initialize / deinitialize ────────────────────────────────────────────────

TEST(HardwareTransportTest, InitializeThrowsOnBadDevice)
{
  HardwareTransport transport;

  TransportConfig cfg;
  cfg.device  = "/dev/nonexistent-i2c";
  cfg.address = 0x28;

  // open() will fail; initialize() must throw.
  EXPECT_THROW(transport.initialize(cfg), std::runtime_error);
}

TEST(HardwareTransportTest, InitializeThrowsOnInvalidAddress)
{
  HardwareTransport transport;

  TransportConfig cfg;
  cfg.device  = "/dev/i2c-1";
  cfg.address = 0x30;  // not a valid BNO055 address

  EXPECT_THROW(transport.initialize(cfg), std::runtime_error);
}
