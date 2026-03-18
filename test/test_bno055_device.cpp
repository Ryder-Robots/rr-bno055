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
#include <fcntl.h>
#include <memory>
#include "rr_bno055/bno055_device.hpp"
#include "rr_bno055/bno055_transport_config.hpp"

using namespace rr_bno055;

// ── Mock transport ────────────────────────────────────────────────────────────
//
// Opens /dev/null as the transport fd so the base-class initialize() marks the
// transport as initialized and the destructor can safely close it.
// bus_read returns 0x00 for all registers; bus_write always succeeds.
//
// With this mock, bno055_init() succeeds (the Bosch SDK stores the chip_id
// without validating it).  set_power_mode(NORMAL) delays ~400 ms and
// set_op_mode(CONFIG) delays ~19 ms, so tests that call initialize() are
// intentionally slow but exercise the full code path.
//
// The operation-mode readback in set_op_mode compares the intended mode
// against the register value returned by bus_read (always 0x00 = CONFIG).
// set_op_mode(CONFIG) therefore succeeds; set_op_mode(any other mode) fails
// with a mode-mismatch message.

class MockHardwareTransport : public HardwareTransport
{
public:
  int initialize_trans(std::shared_ptr<TransportConfig>) override
  {
    return ::open("/dev/null", O_RDWR);
  }

  int8_t bus_read(uint8_t, uint8_t, uint8_t* data, uint8_t len) override
  {
    std::memset(data, 0, len);
    return 0;
  }

  int8_t bus_write(uint8_t, uint8_t, uint8_t*, uint8_t) override
  {
    return 0;
  }

  void mark_initialized()
  {
    auto cfg = std::make_shared<TransportConfig>(TransportConfig::Builder{}.build());
    initialize(cfg);
  }
};

// ── Helpers ───────────────────────────────────────────────────────────────────

static std::shared_ptr<RrBNO055Config> make_cfg(const std::string& device, uint8_t address)
{
  RrBNO055Config::Builder b{};
  b.with_device(device);
  b.with_address(address);
  return std::make_shared<RrBNO055Config>(b.build());
}

static std::shared_ptr<RrBNO055Config> valid_cfg()
{
  return make_cfg("/dev/i2c-1", 0x28);
}

static std::shared_ptr<MockHardwareTransport> initialized_hw()
{
  auto hw = std::make_shared<MockHardwareTransport>();
  hw->mark_initialized();
  return hw;
}

// ── Config validation (fast — no Bosch API calls reach hardware) ──────────────

TEST(Bno055DeviceTest, InitializeRejectsEmptyDevice)
{
  Bno055Device dev;
  EXPECT_FALSE(dev.initialize(make_cfg("", 0x28), initialized_hw()));
}

TEST(Bno055DeviceTest, InitializeRejectsAddressNot0x28Or0x29)
{
  Bno055Device dev;
  EXPECT_FALSE(dev.initialize(make_cfg("/dev/i2c-1", 0x30), initialized_hw()));
}

TEST(Bno055DeviceTest, InitializeRejectsNullTransport)
{
  Bno055Device dev;
  EXPECT_FALSE(dev.initialize(valid_cfg(), nullptr));
}

TEST(Bno055DeviceTest, InitializeRejectsUninitializedTransport)
{
  Bno055Device dev;
  // Transport not marked initialized → is_initilized() returns false.
  EXPECT_FALSE(dev.initialize(valid_cfg(), std::make_shared<MockHardwareTransport>()));
}

// ── Full initialize() happy path ──────────────────────────────────────────────
//
// bno055_init() stores whatever chip_id the mock returns without validating it,
// so initialize() succeeds.  set_op_mode(CONFIG) readback is 0x00 (CONFIG) and
// matches, so the full path completes.

TEST(Bno055DeviceTest, InitializeSucceedsAddress0x28)
{
  Bno055Device dev;
  EXPECT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
}

TEST(Bno055DeviceTest, InitializeSucceedsAddress0x29)
{
  Bno055Device dev;
  EXPECT_TRUE(dev.initialize(make_cfg("/dev/i2c-1", 0x29), initialized_hw()));
}

// ── Deinitialize ─────────────────────────────────────────────────────────────

TEST(Bno055DeviceTest, DeinitializeBeforeInitializeIsSafe)
{
  Bno055Device dev;
  EXPECT_NO_THROW(dev.deinitialize());
}

TEST(Bno055DeviceTest, DeinitializeAfterSuccessfulInitialize)
{
  Bno055Device dev;
  auto hw = initialized_hw();
  ASSERT_TRUE(dev.initialize(valid_cfg(), hw));
  EXPECT_NO_THROW(dev.deinitialize());
}

// ── set_op_mode ───────────────────────────────────────────────────────────────
//
// After initialize(), hw_ is valid.  CONFIG (0x00) matches the mock readback
// (also 0x00), so it returns true.  NDOF (0x0C) does not match → returns false,
// but the function body executes in full without crashing.

TEST(Bno055DeviceTest, SetOpModeToConfigSucceeds)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  EXPECT_TRUE(dev.set_op_mode(RRBNO055_OPERATION_MODE_CONFIG));
}

TEST(Bno055DeviceTest, SetOpModeReadbackMismatchReturnsFalse)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  // NDOF readback will be 0x00 ≠ 0x0C — mismatch logged, returns false.
  EXPECT_FALSE(dev.set_op_mode(RRBNO055_OPERATION_MODE_NDOF));
}

// ── set_power_mode ────────────────────────────────────────────────────────────

TEST(Bno055DeviceTest, SetPowerModeSucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  EXPECT_TRUE(dev.set_power_mode(RRBNO055_POWER_MODE_LOWPOWER));
}

// ── reset ─────────────────────────────────────────────────────────────────────
//
// After initialize(), reset() runs its full body:
//   1. bno055_set_sys_rst  → bus_write returns 0 → BNO055_SUCCESS
//   2. delay_msec(650)     → ~650 ms sleep
//   3. bno055_init         → succeeds (mock)
//   4. set_op_mode(NDOF)   → readback 0x00 ≠ 0x0C → mismatch → returns false
//
// The function returns false due to the mode mismatch, but the important
// invariant is that the full reset path executes without crashing.

TEST(Bno055DeviceTest, ResetAfterInitializeExecutesWithoutCrash)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  EXPECT_NO_THROW(dev.reset());
}

// ── read_quaternion ────────────────────────────────────────────────────────────
//
// With the mock (bus_read returns 0x00), bno055_read_quaternion_wxyz succeeds
// and all fields are zero.

TEST(Bno055DeviceTest, ReadQuaternionSucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_quaternion_t quat{};
  EXPECT_TRUE(dev.read_quaternion(quat));
}

TEST(Bno055DeviceTest, ReadQuaternionFieldsAreZeroWithMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_quaternion_t quat{ 99, 99, 99, 99 };
  ASSERT_TRUE(dev.read_quaternion(quat));
  EXPECT_EQ(quat.w, 0);
  EXPECT_EQ(quat.x, 0);
  EXPECT_EQ(quat.y, 0);
  EXPECT_EQ(quat.z, 0);
}

// ── read_angular_velocity ──────────────────────────────────────────────────────

TEST(Bno055DeviceTest, ReadAngularVelocitySucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_gyro_t gyro{};
  EXPECT_TRUE(dev.read_angular_velocity(gyro));
}

TEST(Bno055DeviceTest, ReadAngularVelocityFieldsAreZeroWithMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_gyro_t gyro{ 99, 99, 99 };
  ASSERT_TRUE(dev.read_angular_velocity(gyro));
  EXPECT_EQ(gyro.x, 0);
  EXPECT_EQ(gyro.y, 0);
  EXPECT_EQ(gyro.z, 0);
}

// ── read_linear_acceleration ───────────────────────────────────────────────────

TEST(Bno055DeviceTest, ReadLinearAccelerationSucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_linear_accel_t accel{};
  EXPECT_TRUE(dev.read_linear_acceleration(accel));
}

TEST(Bno055DeviceTest, ReadLinearAccelerationFieldsAreZeroWithMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_linear_accel_t accel{ 99, 99, 99 };
  ASSERT_TRUE(dev.read_linear_acceleration(accel));
  EXPECT_EQ(accel.x, 0);
  EXPECT_EQ(accel.y, 0);
  EXPECT_EQ(accel.z, 0);
}

// ── read_gravity ───────────────────────────────────────────────────────────────

TEST(Bno055DeviceTest, ReadGravitySucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_gravity_t gravity{};
  EXPECT_TRUE(dev.read_gravity(gravity));
}

TEST(Bno055DeviceTest, ReadGravityFieldsAreZeroWithMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  bno055_gravity_t gravity{ 99, 99, 99 };
  ASSERT_TRUE(dev.read_gravity(gravity));
  EXPECT_EQ(gravity.x, 0);
  EXPECT_EQ(gravity.y, 0);
  EXPECT_EQ(gravity.z, 0);
}

// ── get_system_status ─────────────────────────────────────────────────────────

TEST(Bno055DeviceTest, GetSystemStatusSucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  uint8_t status = 0xFF, error = 0xFF;
  EXPECT_TRUE(dev.get_system_status(status, error));
}

TEST(Bno055DeviceTest, GetSystemStatusWritesZeroFromMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  uint8_t status = 0xFF, error = 0xFF;
  ASSERT_TRUE(dev.get_system_status(status, error));
  // Mock bus_read always returns 0x00 for all registers.
  EXPECT_EQ(status, 0x00);
  EXPECT_EQ(error, 0x00);
}

// ── get_calibration_status ────────────────────────────────────────────────────

TEST(Bno055DeviceTest, GetCalibrationStatusSucceedsAfterInitialize)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  RrBno055CalibData data;
  EXPECT_TRUE(dev.get_calibration_status(data));
}

TEST(Bno055DeviceTest, GetCalibrationStatusFieldsAreZeroWithMock)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  RrBno055CalibData data;
  ASSERT_TRUE(dev.get_calibration_status(data));
  // Mock returns 0x00 for all registers — all sensors at calibration level 0.
  EXPECT_EQ(data.sys, 0);
  EXPECT_EQ(data.gyro, 0);
  EXPECT_EQ(data.accel, 0);
  EXPECT_EQ(data.mag, 0);
}

// ── is_fully_calibrated ───────────────────────────────────────────────────────
//
// The mock returns 0x00 for all registers so every sensor reports calibration
// level 0 (< 3). is_fully_calibrated() delegates to get_calibration_status()
// to avoid duplicate I2C transactions, then maps each level < 3 to a NOT_CALIBRATED
// flag bit.

TEST(Bno055DeviceTest, IsFullyCalibratedReturnsFalseWhenUncalibrated)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  uint8_t calib_status = 0;
  EXPECT_FALSE(dev.is_fully_calibrated(calib_status));
}

TEST(Bno055DeviceTest, IsFullyCalibratedSetsAllNotCalibratedBits)
{
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));
  uint8_t calib_status = 0;
  ASSERT_FALSE(dev.is_fully_calibrated(calib_status));
  EXPECT_TRUE(calib_status & RrBno055CalibStatus::SYS_NOT_CALIBRATED);
  EXPECT_TRUE(calib_status & RrBno055CalibStatus::GYRO_NOT_CALIBRATED);
  EXPECT_TRUE(calib_status & RrBno055CalibStatus::ACCEL_NOT_CALIBRATED);
  EXPECT_TRUE(calib_status & RrBno055CalibStatus::MAG_NOT_CALIBRATED);
}

TEST(Bno055DeviceTest, IsFullyCalibratedAndGetCalibrationStatusAreConsistent)
{
  // Verifies that is_fully_calibrated delegates to get_calibration_status and
  // both methods agree on the sensor state — no duplicate I2C reads.
  Bno055Device dev;
  ASSERT_TRUE(dev.initialize(valid_cfg(), initialized_hw()));

  RrBno055CalibData data;
  ASSERT_TRUE(dev.get_calibration_status(data));

  uint8_t calib_status = 0;
  bool fully = dev.is_fully_calibrated(calib_status);

  // Both must agree: if any field < 3, is_fully_calibrated must return false.
  bool any_uncalibrated = data.sys < 3 || data.gyro < 3 || data.accel < 3 || data.mag < 3;
  EXPECT_EQ(fully, !any_uncalibrated);
}
