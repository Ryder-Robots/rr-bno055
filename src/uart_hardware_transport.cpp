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

#include "rr_bno055/uart_hardware_transport.hpp"

using namespace rr_bno055;

int UARTHardwareTransport::initialize_trans(const TransportConfig& transport_config)
{
  int fd = open(transport_config.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    std::cerr << "[TransportFactory] Failed to open UART device " << transport_config.device << ": " << strerror(errno)
              << std::endl;
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) < 0)
  {
    std::cerr << "[TransportFactory] tcgetattr failed: " << strerror(errno) << std::endl;
    close(fd);
    return -1;
  }

  // Baud rate - BNO055 UART default is 115200
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  // 8N1 - 8 data bits, no parity, 1 stop bit
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CLOCAL | CREAD;

  // Raw input - no echo, no signals
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Blocking read with timeout
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;  // 0.5 second timeout

  if (tcsetattr(fd, TCSANOW, &tty) < 0)
  {
    std::cerr << "[TransportFactory] tcsetattr failed: " << strerror(errno) << std::endl;
    close(fd);
    return -1;
  }

  return fd;
}

int8_t UARTHardwareTransport::bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
}

int8_t UARTHardwareTransport::bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
}
