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

#include "bno055.h"
#include "rr_bno055/transport_config.hpp"

namespace rr_bno055
{
/**
 * @brief Physical-axis remapping values for the BNO055.
 *
 * Controls which sensor axis is mapped to which output axis, allowing the
 * driver to compensate for the sensor being mounted in a non-standard
 * orientation.  Pass to `Bno055Device::set_axis_remap()` or
 * `RrBNO055Config::Builder::with_axis_remap()`.
 *
 * | Constant                    | Value | Mapping             |
 * | --------------------------- | ----- | ------------------- |
 * | `RRBNO055_DEFAULT_AXIS`     | 0x24  | X=X; Y=Y; Z=Z       |
 * | `RRBNO055_REMAP_X_Y`        | 0x21  | X=Y; Y=X; Z=Z       |
 * | `RRBNO055_REMAP_Y_Z`        | 0x18  | X=X; Y=Z; Z=Y       |
 * | `RRBNO055_REMAP_Z_X`        | 0x06  | X=Z; Y=Y; Z=X       |
 * | `RRBNO055_REMAP_X_Y_Z_TYPE0`| 0x12  | X=Z; Y=X; Z=Y       |
 * | `RRBNO055_REMAP_X_Y_Z_TYPE1`| 0x09  | X=Y; Y=Z; Z=X       |
 */
enum RrBno055AxisRemap : uint8_t
{
  RRBNO055_REMAP_X_Y         = BNO055_REMAP_X_Y,         ///< 0x21: X=Y; Y=X; Z=Z
  RRBNO055_REMAP_Y_Z         = BNO055_REMAP_Y_Z,         ///< 0x18: X=X; Y=Z; Z=Y
  RRBNO055_REMAP_Z_X         = BNO055_REMAP_Z_X,         ///< 0x06: X=Z; Y=Y; Z=X
  RRBNO055_REMAP_X_Y_Z_TYPE0 = BNO055_REMAP_X_Y_Z_TYPE0, ///< 0x12: X=Z; Y=X; Z=Y
  RRBNO055_REMAP_X_Y_Z_TYPE1 = BNO055_REMAP_X_Y_Z_TYPE1, ///< 0x09: X=Y; Y=Z; Z=X
  RRBNO055_DEFAULT_AXIS      = BNO055_DEFAULT_AXIS,       ///< 0x24: X=X; Y=Y; Z=Z (identity)
};

/**
 * @brief Sign (polarity) for a single sensor axis output.
 *
 * A negative sign inverts the reported value for that axis.  Use
 * `RrBno055AxisSignXYZ` to specify signs for all three axes together.
 */
enum RrBno055AxisSign : uint8_t
{
  RRBNO055_REMAP_AXIS_POSITIVE = BNO055_REMAP_AXIS_POSITIVE, ///< Axis output is not inverted.
  RRBNO055_REMAP_AXIS_NEGATIVE = BNO055_REMAP_AXIS_NEGATIVE, ///< Axis output is inverted.
};

/**
 * @brief Per-axis sign configuration for the BNO055.
 *
 * Specifies the polarity of each output axis independently.  The defaults
 * (x=NEGATIVE, y=POSITIVE, z=NEGATIVE) match the physical mounting used
 * by Ryder Robots; adjust to suit your own mounting orientation using
 * `imu_orientation_check --sign-x/y/z` to find the correct combination.
 */
struct RrBno055AxisSignXYZ
{
  RrBno055AxisSign x_sign = RRBNO055_REMAP_AXIS_NEGATIVE; ///< X-axis polarity. Default: negative.
  RrBno055AxisSign y_sign = RRBNO055_REMAP_AXIS_POSITIVE; ///< Y-axis polarity. Default: positive.
  RrBno055AxisSign z_sign = RRBNO055_REMAP_AXIS_NEGATIVE; ///< Z-axis polarity. Default: negative.
};

/**
 * @brief BNO055-specific transport configuration.
 *
 * Extends `TransportConfig` with the axis remapping and per-axis sign
 * settings needed to compensate for the sensor's physical mounting orientation.
 * Constructed exclusively through `RrBNO055Config::Builder`.
 *
 * @see RrBno055AxisRemap
 * @see RrBno055AxisSignXYZ
 */
class RrBNO055Config : public TransportConfig
{
public:
  const RrBno055AxisRemap   axis_remap;    ///< Physical-to-output axis mapping.
  const RrBno055AxisSignXYZ axis_sign_xyz; ///< Per-axis output polarity.

  /**
   * @brief Fluent builder for `RrBNO055Config`.
   *
   * Inherits `with_type()`, `with_device()`, and `with_address()` from
   * `TransportConfig::Builder`.  Because those methods return
   * `TransportConfig::Builder&`, use non-chained calls when mixing parent
   * and child methods (see `TransportConfig::Builder` for an example).
   */
  class Builder : public TransportConfig::Builder
  {
  public:
    /// Sets the axis remap. Default: `RRBNO055_REMAP_X_Y`.
    Builder& with_axis_remap(RrBno055AxisRemap axis_remap)
    {
      axis_remap_ = axis_remap;
      return *this;
    }

    /**
     * @brief Sets per-axis output signs. Default: x=neg, y=pos, z=neg.
     *
     * Use `imu_orientation_check --sign-x/y/z` to determine the correct
     * values for your mounting orientation, then set them here so they
     * are applied automatically during `Bno055Device::initialize()`.
     */
    Builder& with_axis_sign_xyz(RrBno055AxisSignXYZ axis_sign_xyz)
    {
      axis_sign_xyz_ = axis_sign_xyz;
      return *this;
    }

    RrBNO055Config build() const
    {
      return RrBNO055Config(type_, device_, address_, axis_remap_, axis_sign_xyz_);
    }

  private:
    RrBno055AxisRemap   axis_remap_    = RRBNO055_REMAP_X_Y;
    RrBno055AxisSignXYZ axis_sign_xyz_;
  };

private:
  RrBNO055Config(TransportType type, std::string device, uint8_t address, RrBno055AxisRemap remap,
                 RrBno055AxisSignXYZ axis_sign_xyz_in)
    : TransportConfig(type, std::move(device), address)
    , axis_remap(remap)  // parameter named remap, member named axis_remap
    , axis_sign_xyz(axis_sign_xyz_in)
  {
  }
};
}  // namespace rr_bno055
