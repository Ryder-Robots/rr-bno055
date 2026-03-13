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

#include <array>
#include <memory>
extern "C" {
#include "bno055.h"
}
#include "rr_bno055/hardware_transport.hpp"
#include "rr_bno055/transport_factory.hpp"

namespace rr_bno055
{

enum RrBno055OpMode : uint8_t
{
  RRBNO055_OPERATION_MODE_CONFIG = BNO055_OPERATION_MODE_CONFIG,
  RRBNO055_OPERATION_MODE_ACCONLY = BNO055_OPERATION_MODE_ACCONLY,
  RRBNO055_OPERATION_MODE_MAGONLY = BNO055_OPERATION_MODE_MAGONLY,
  RRBNO055_OPERATION_MODE_GYRONLY = BNO055_OPERATION_MODE_GYRONLY,
  RRBNO055_OPERATION_MODE_ACCMAG = BNO055_OPERATION_MODE_ACCMAG,
  RRBNO055_OPERATION_MODE_ACCGYRO = BNO055_OPERATION_MODE_ACCGYRO,
  RRBNO055_OPERATION_MODE_MAGGYRO = BNO055_OPERATION_MODE_MAGGYRO,
  RRBNO055_OPERATION_MODE_AMG = BNO055_OPERATION_MODE_AMG,
  RRBNO055_OPERATION_MODE_IMUPLUS = BNO055_OPERATION_MODE_IMUPLUS,
  RRBNO055_OPERATION_MODE_COMPASS = BNO055_OPERATION_MODE_COMPASS,
  RRBNO055_OPERATION_MODE_M4G = BNO055_OPERATION_MODE_M4G,
  RRBNO055_OPERATION_MODE_NDOF_FMC_OFF = BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  RRBNO055_OPERATION_MODE_NDOF = BNO055_OPERATION_MODE_NDOF,
};

class Bno055Device
{
public:

  // Probally does not have to be a singleton strictly speaking,
  // but something feels right about ensuring that it is.
  Bno055Device(const Bno055Device&) = delete;
  Bno055Device& operator=(const Bno055Device&) = delete;
  Bno055Device(Bno055Device&&) = delete;
  Bno055Device& operator=(Bno055Device&&) = delete;
  virtual ~Bno055Device() = default;

  // Lifecycle:
  bool initialize(const TransportConfig& conf, std::shared_ptr<HardwareTransport> hw);
  void deinitialize();
  bool reset();

  // Configuration: (reference bno055_set_operation_mode), s
  bool set_mode(RrBno055OpMode mode);

  void setAxisRemap();

  // Data Reading:

  void readQuaternion();
  void readAngularVelocity();
  void readLinearAcceleration();
  void readGravity();

  // Status:

  void getCalibrationStatus();
  void isCalibrated();
  void getSystemStatus();

private:
  
  std::shared_ptr<HardwareTransport> hw_;

    /**
   * @brief Returns a copy of the Bosch SensorAPI device struct.
   *
   * The returned struct contains the function pointers wired to this transport
   * and can be passed to higher-level Bosch API calls (e.g. reading sensor data).
   * Only valid after `initialize()` has succeeded.
   */
  bno055_t device_;

  // defines the delay between mode changes.
  static constexpr std::array<uint8_t, 12> OP_MODE_SWITCH_DELAY = {
    19,  // BNO055_OPERATION_MODE_CONFIG   = 0x00
    3,   // BNO055_OPERATION_MODE_ACCONLY  = 0x01
    3,   // BNO055_OPERATION_MODE_MAGONLY  = 0x02
    3,   // BNO055_OPERATION_MODE_GYRONLY  = 0x03
    3,   // BNO055_OPERATION_MODE_ACCMAG   = 0x04
    3,   // BNO055_OPERATION_MODE_ACCGYRO  = 0x05
    3,   // BNO055_OPERATION_MODE_MAGGYRO  = 0x06
    3,   // BNO055_OPERATION_MODE_AMG      = 0x07
    7,   // BNO055_OPERATION_MODE_IMUPLUS  = 0x08
    7,   // BNO055_OPERATION_MODE_COMPASS  = 0x09
    7,   // BNO055_OPERATION_MODE_M4G      = 0x0A
    7,   // BNO055_OPERATION_MODE_NDOF     = 0x0B
  };
};
}  // namespace rr_bno055