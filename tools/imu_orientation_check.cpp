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

// ─── imu_orientation_check ───────────────────────────────────────────────────
//
// Reads orientation data from a BNO055 over I2C or UART and prints a live
// display to stdout so the sensor's physical axis mapping can be verified
// without a full ROS stack.
//
// Usage:
//   imu_orientation_check [OPTIONS]
//
// Options:
//   --device PATH    I2C or UART device (default: /dev/i2c-1)
//   --address ADDR   I2C address in hex, 0x28 or 0x29 (default: 0x28)
//   --uart           Use UART transport instead of I2C
//   --no-wait        Skip calibration wait; start reading immediately
//   --rate HZ        Refresh rate in Hz (default: 10, max: 50)
//
// Orientation convention (BNO055 NDOF mode, default axis mapping):
//   Roll  — rotation about X axis (positive = right side down)
//   Pitch — rotation about Y axis (positive = nose up)
//   Yaw   — rotation about Z axis (positive = nose left, i.e. CCW from above)
//
// Press Ctrl+C to exit cleanly.

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "rr_bno055/bno055_device.hpp"
#include "rr_bno055/bno055_transport_config.hpp"
#include "rr_bno055/transport_factory.hpp"

using namespace rr_bno055;

// ─── Signal handling ─────────────────────────────────────────────────────────

static volatile bool g_running = true;

static void on_sigint(int)
{
  g_running = false;
}

// ─── ANSI helpers ────────────────────────────────────────────────────────────

static constexpr const char* RESET  = "\033[0m";
static constexpr const char* BOLD   = "\033[1m";
static constexpr const char* RED    = "\033[31m";
static constexpr const char* GREEN  = "\033[32m";
static constexpr const char* YELLOW = "\033[33m";
static constexpr const char* CYAN   = "\033[36m";
static constexpr const char* CLEAR_LINE = "\033[2K\r";

// Move cursor up N lines.
static void cursor_up(int n)
{
  std::cout << "\033[" << n << "A";
}

// ─── Quaternion → roll / pitch / yaw ────────────────────────────────────────
//
// BNO055 raw quaternion LSB = 1/2^14.  After normalisation the conversion
// follows the standard ZYX aerospace convention:
//
//   yaw   = atan2( 2(wz + xy),  1 - 2(y² + z²) )
//   pitch = asin(  2(wy - zx) )
//   roll  = atan2( 2(wx + yz),  1 - 2(x² + y²) )

struct Euler
{
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
};

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double BNO055_QUAT_SCALE = 1.0 / 16384.0;  // 2^14

static Euler quat_to_euler(const bno055_quaternion_t& q)
{
  const double w = q.w * BNO055_QUAT_SCALE;
  const double x = q.x * BNO055_QUAT_SCALE;
  const double y = q.y * BNO055_QUAT_SCALE;
  const double z = q.z * BNO055_QUAT_SCALE;

  const double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin( std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  const double yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  return { roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG };
}

// ─── Calibration display ─────────────────────────────────────────────────────

static const char* calib_bar(uint8_t level)
{
  switch (level)
  {
    case 0: return "[    ]";
    case 1: return "[=   ]";
    case 2: return "[==  ]";
    case 3: return "[====]";
    default: return "[????]";
  }
}

static const char* calib_colour(uint8_t level)
{
  return level == 3 ? GREEN : (level >= 1 ? YELLOW : RED);
}

// ─── Live display ─────────────────────────────────────────────────────────────
//
// First call prints N lines; subsequent calls move the cursor back to the top
// of the block and overwrite, giving an in-place update effect.

static bool g_first_frame = true;
static constexpr int DISPLAY_LINES = 17;

static void print_frame(const Euler& euler,
                        const bno055_quaternion_t& quat,
                        const bno055_gyro_t& gyro,
                        const RrBno055CalibData& calib)
{
  if (!g_first_frame)
    cursor_up(DISPLAY_LINES);
  g_first_frame = false;

  auto print_angle = [](const char* label, double val) {
    const char* colour = (std::abs(val) < 5.0) ? GREEN : (std::abs(val) < 30.0 ? YELLOW : RED);
    std::cout << CLEAR_LINE
              << "  " << BOLD << label << RESET
              << "  " << colour << std::showpos;
    std::cout.precision(2);
    std::cout << std::fixed << val << "°" << RESET << '\n';
    std::cout << std::noshowpos;
  };

  std::cout << CLEAR_LINE << BOLD << CYAN
            << "─── BNO055 Orientation Check ───────────────────" << RESET << '\n';
  std::cout << CLEAR_LINE << '\n';

  std::cout << CLEAR_LINE << BOLD << "  Euler Angles (ZYX / aerospace)" << RESET << '\n';
  print_angle("Roll  (X):", euler.roll_deg);
  print_angle("Pitch (Y):", euler.pitch_deg);
  print_angle("Yaw   (Z):", euler.yaw_deg);
  std::cout << CLEAR_LINE << '\n';

  std::cout << CLEAR_LINE << BOLD << "  Raw Quaternion  (normalised)" << RESET << '\n';
  std::cout << CLEAR_LINE << std::fixed;
  std::cout.precision(4);
  std::cout << "  w=" << quat.w * BNO055_QUAT_SCALE
            << "  x=" << quat.x * BNO055_QUAT_SCALE
            << "  y=" << quat.y * BNO055_QUAT_SCALE
            << "  z=" << quat.z * BNO055_QUAT_SCALE << '\n';
  std::cout << CLEAR_LINE << '\n';

  std::cout << CLEAR_LINE << BOLD << "  Angular Velocity (°/s)" << RESET << '\n';
  std::cout << CLEAR_LINE << std::showpos << std::fixed;
  std::cout.precision(1);
  // BNO055 gyro LSB = 1/16 dps in default range
  constexpr double GYRO_SCALE = 1.0 / 16.0;
  std::cout << "  x=" << gyro.x * GYRO_SCALE
            << "  y=" << gyro.y * GYRO_SCALE
            << "  z=" << gyro.z * GYRO_SCALE << '\n';
  std::cout << std::noshowpos;
  std::cout << CLEAR_LINE << '\n';

  std::cout << CLEAR_LINE << BOLD << "  Calibration  (0=none  3=full)" << RESET << '\n';
  std::cout << CLEAR_LINE
            << "  Sys  " << calib_colour(calib.sys)  << calib_bar(calib.sys)  << " " << (int)calib.sys  << RESET << '\n';
  std::cout << CLEAR_LINE
            << "  Gyro " << calib_colour(calib.gyro) << calib_bar(calib.gyro) << " " << (int)calib.gyro << RESET << '\n';
  std::cout << CLEAR_LINE
            << "  Accel" << calib_colour(calib.accel)<< calib_bar(calib.accel)<< " " << (int)calib.accel<< RESET << '\n';
  std::cout << CLEAR_LINE
            << "  Mag  " << calib_colour(calib.mag)  << calib_bar(calib.mag)  << " " << (int)calib.mag  << RESET << '\n';
  std::cout << CLEAR_LINE << '\n';
  std::cout << CLEAR_LINE
            << "  " << YELLOW << "Ctrl+C to exit" << RESET << '\n';

  std::cout.flush();
}

// ─── CLI parsing ─────────────────────────────────────────────────────────────

struct Options
{
  std::string device  = "/dev/i2c-1";
  uint8_t     address = 0x28;
  bool        uart    = false;
  bool        no_wait = false;
  int         rate_hz = 10;
};

static void print_usage(const char* prog)
{
  std::cout << "Usage: " << prog << " [OPTIONS]\n"
            << "\n"
            << "Options:\n"
            << "  --device PATH    I2C or UART device node  (default: /dev/i2c-1)\n"
            << "  --address ADDR   I2C hex address 0x28|0x29 (default: 0x28)\n"
            << "  --uart           Use UART transport instead of I2C\n"
            << "  --no-wait        Skip calibration wait; start reading immediately\n"
            << "  --rate HZ        Refresh rate in Hz, 1-50  (default: 10)\n"
            << "  --help           Show this message\n";
}

static Options parse_args(int argc, char** argv)
{
  Options opts;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help")
    {
      print_usage(argv[0]);
      std::exit(0);
    }
    else if (arg == "--uart")
    {
      opts.uart = true;
      opts.device = "/dev/ttyAMA0";  // sensible UART default on Pi
    }
    else if (arg == "--no-wait")
    {
      opts.no_wait = true;
    }
    else if (arg == "--device" && i + 1 < argc)
    {
      opts.device = argv[++i];
    }
    else if (arg == "--address" && i + 1 < argc)
    {
      opts.address = static_cast<uint8_t>(std::stoul(argv[++i], nullptr, 16));
    }
    else if (arg == "--rate" && i + 1 < argc)
    {
      opts.rate_hz = std::clamp(std::stoi(argv[++i]), 1, 50);
    }
    else
    {
      std::cerr << "Unknown option: " << arg << '\n';
      print_usage(argv[0]);
      std::exit(1);
    }
  }
  return opts;
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  std::signal(SIGINT, on_sigint);

  const Options opts = parse_args(argc, argv);

  // Build config.
  RrBNO055Config::Builder b{};
  b.with_device(opts.device);
  b.with_address(opts.address);
  if (opts.uart)
    b.with_type(TransportType::UART);
  auto conf = std::make_shared<RrBNO055Config>(b.build());

  // Open transport.
  TransportFactory factory;
  std::shared_ptr<HardwareTransport> hw;
  try
  {
    hw = factory.get_or_create_transport(conf);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to open transport on " << opts.device << ": " << e.what() << '\n';
    return 1;
  }

  // Initialise sensor.
  Bno055Device imu;
  std::cout << "Initialising BNO055 on " << opts.device
            << " (addr 0x" << std::hex << (int)opts.address << std::dec << ") …\n";

  if (!imu.initialize(conf, hw))
  {
    std::cerr << "Sensor initialisation failed. Check wiring and device path.\n";
    return 1;
  }

  if (!imu.set_op_mode(RRBNO055_OPERATION_MODE_NDOF))
  {
    std::cerr << "Warning: could not set NDOF mode — orientation data may be unreliable.\n";
  }

  // Calibration wait.
  if (!opts.no_wait)
  {
    std::cout << "\nWaiting for full calibration. Move the sensor:\n"
              << "  Gyro  — hold still\n"
              << "  Accel — place in 6 orientations\n"
              << "  Mag   — figure-8 in the air\n"
              << "\nPress Ctrl+C to skip and start reading immediately.\n\n";

    uint8_t calib_status;
    while (g_running && !imu.is_fully_calibrated(calib_status))
    {
      RrBno055CalibData data;
      imu.get_calibration_status(data);
      std::cout << CLEAR_LINE
                << "  Sys "  << calib_colour(data.sys)  << (int)data.sys  << RESET
                << "  Gyro " << calib_colour(data.gyro) << (int)data.gyro << RESET
                << "  Accel "<< calib_colour(data.accel)<< (int)data.accel<< RESET
                << "  Mag "  << calib_colour(data.mag)  << (int)data.mag  << RESET
                << "  (3=full)";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (g_running)
      std::cout << CLEAR_LINE << GREEN << "  Fully calibrated!" << RESET << "\n\n";
    else
      std::cout << CLEAR_LINE << YELLOW << "  Skipping calibration wait." << RESET << "\n\n";

    // Reset the signal — Ctrl+C during calibration just skips the wait.
    g_running = true;
  }

  // Live read loop.
  const auto period = std::chrono::milliseconds(1000 / opts.rate_hz);
  bno055_quaternion_t quat{};
  bno055_gyro_t gyro{};
  RrBno055CalibData calib{};

  while (g_running)
  {
    auto tick = std::chrono::steady_clock::now();

    imu.read_quaternion(quat);
    imu.read_angular_velocity(gyro);
    imu.get_calibration_status(calib);

    print_frame(quat_to_euler(quat), quat, gyro, calib);

    std::this_thread::sleep_until(tick + period);
  }

  // Clean up.
  std::cout << '\n' << BOLD << "Shutting down…" << RESET << '\n';
  imu.deinitialize();
  factory.cleanup();

  return 0;
}
