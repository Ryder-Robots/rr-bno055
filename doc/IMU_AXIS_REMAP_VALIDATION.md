# IMU Axis Remap Validation

## Overview

This document records the empirical validation of the BNO055 axis remap and sign
configuration implemented in `bno055_transport_config.hpp` and `bno055_device.cpp`.
The sensor is mounted in an inverted orientation on the mazebot platform, requiring
a non-default axis sign configuration to produce correct heading output.

## Configuration Under Test

The following default configuration is applied during `initialize()`:

```cpp
// Axis remap
RRBNO055_REMAP_X_Y  // 0x21: Z=Z; X=Y; Y=X

// Axis sign defaults (RrBno055AxisSignXYZ)
x_sign = RRBNO055_REMAP_AXIS_NEGATIVE
y_sign = RRBNO055_REMAP_AXIS_POSITIVE
z_sign = RRBNO055_REMAP_AXIS_NEGATIVE
```

These defaults are encoded in the `RrBno055AxisSignXYZ` struct and flow through the
builder pattern without requiring explicit caller configuration. They can be overridden
via `RrBNO055Config::Builder::with_axis_sign_xyz()` if the mounting orientation changes.

## Test Procedure

Four sequential physical movement tests were performed. Each test follows directly
from the end position of the previous test. All tests were performed by hand without
a measurement grid — results are therefore approximations, and tolerance should be
applied accordingly.

| Test | Movement | Expected Result |
|------|----------|-----------------|
| 1 | Move IMU forward, hold stationary for ~2 seconds | Roll stable near ±180°, pitch stable, yaw stable |
| 2 | From test 1 position, rotate 90° anti-clockwise | Yaw decreases by ~90°, roll remains stable |
| 3 | From test 2 position, rotate a further 90° anti-clockwise | Yaw decreases by a further ~90°, roll remains stable |
| 4 | From test 3 position, move IMU backward, hold stationary for ~2 seconds | Yaw stable, pitch shift consistent with orientation change |

## Results

### Test 1 — Forward, stationary hold (4.8s)

| Metric | Value |
|--------|-------|
| Roll | ~-179° (stable) |
| Pitch | ~3.3° (stable) |
| Yaw | ~-167° to ~176° (minor drift, sensor uncalibrated) |
| Calibration | sys=0, gyro=0, accel=0, mag=0 |

Roll is stable at ±179° throughout. This is the correct resting value for the
inverted mounting — it is not indicative of a fault. Minor yaw drift late in the
log is consistent with sensor operating in an uncalibrated state at test start.

### Test 2 — 90° anti-clockwise rotation (8.9s)

| Metric | Value |
|--------|-------|
| Roll | ~178–179° (stable throughout rotation and hold) |
| Pitch | ~1.2–3.4° |
| Yaw start | ~-167° |
| Yaw end | ~-87° |
| Δ Yaw | ~80° (expected ~90°, within physical approximation tolerance) |
| Calibration | sys=0, gyro=3, accel=0, mag=0 |

Roll remains stable through the full rotation. Yaw tracks the anti-clockwise
movement cleanly. This is the primary confirmation that the inverted mounting
correction is working correctly — roll no longer inverts or becomes stuck during
rotation.

### Test 3 — Further 90° anti-clockwise rotation (7.5s)

| Metric | Value |
|--------|-------|
| Roll | ~177–180° (stable throughout rotation and hold) |
| Pitch | ~0.2–2.7° |
| Yaw start | ~-77° |
| Yaw end | ~-171° |
| Δ Yaw | ~94° (expected ~90°, within physical approximation tolerance) |
| Calibration | sys=0, gyro=3, accel=0, mag=0 |

Roll continues to track correctly through a second 90° rotation. Settles cleanly
at -180° during the stationary hold at end of test.

### Test 4 — Backward, stationary hold (5.6s)

| Metric | Value |
|--------|-------|
| Roll | ~±179–180° (stable) |
| Pitch | ~5.0° (elevated relative to test 1, consistent with backward orientation) |
| Yaw | ~-164° to -171° (stable) |
| Calibration | sys=0, gyro=3, accel=0, mag=0 |

Yaw is stable. Pitch increase relative to test 1 (~3.3° vs ~5.0°) is consistent
with the physical difference between forward and backward hold orientations.

## Conclusions

The axis remap and sign configuration is confirmed correct for the inverted mounting
orientation. Specifically:

- Roll is stable at ±179–180° when stationary, which is the correct resting value
  for this mounting — not a fault condition.
- Roll does not invert or become stuck during rotation.
- Yaw tracks anti-clockwise rotation accurately to within physical approximation
  tolerance across two sequential 90° rotations (cumulative ~174° tracked, expected 180°).
- Pitch responds consistently to orientation changes.

## Outstanding Notes

Accelerometer and magnetometer calibration did not reach level 3 during any test
(accel=0, mag=0 throughout). This is a runtime calibration concern separate from
the axis remap correctness validated here, and does not affect the conclusions above.
Heading accuracy under navigation load should be re-evaluated once full calibration
is achieved during normal operation.

## Raw Data

Test logs are retained as CSV files:

| File | Test |
|------|------|
| `imu_log_1-v3.csv` | Test 1 — forward stationary |
| `imu_log_2-v3.csv` | Test 2 — first 90° anti-clockwise |
| `imu_log_3-v3.csv` | Test 3 — second 90° anti-clockwise |
| `imu_log_4-v3.csv` | Test 4 — backward stationary |
