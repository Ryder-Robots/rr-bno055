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
    std::cerr << "[UARTHardwareTransport] Failed to open UART device " << transport_config.device << ": " << strerror(errno)
              << std::endl;
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) < 0)
  {
    std::cerr << "[UARTHardwareTransport] tcgetattr failed: " << strerror(errno) << std::endl;
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
    std::cerr << "[UARTHardwareTransport] tcsetattr failed: " << strerror(errno) << std::endl;
    close(fd);
    return -1;
  }

  return fd;
}

// read may not return exactly len bytes,  this will usually mean it will
// need more than one pass, read_exact attempts exactly len bytes.
ssize_t UARTHardwareTransport::read_exact(int fd, uint8_t* buf, size_t len)
{
  size_t bytes_read_total = 0;
  while (len - bytes_read_total > 0)
  {
    ssize_t bytes_read = read(fd, buf, len - bytes_read_total);
    if (bytes_read == -1)
    {
      // Retry on signal interrupt
      if (errno == EINTR)
      {
        continue;
      }
      return -1;
    }

    // EOF / device closed
    else if (bytes_read == 0)
    {
      errno = EIO;
      return -1;
    }

    bytes_read_total += bytes_read;
    buf += bytes_read;
  }
  return static_cast<ssize_t>(bytes_read_total);
}

ssize_t UARTHardwareTransport::write_all(int fildes, const void* buf, size_t nbyte)
{
  const uint8_t* b = static_cast<const uint8_t*>(buf);
  size_t remainder = nbyte;
  size_t total_written = 0;
  while (remainder > 0)
  {
    ssize_t written = write(fildes, b, remainder);
    if (written == -1)
    {
      // Retry on signal interrupt
      if (errno == EINTR)
      {
        continue;
      }
      return written;
    }
    else if (written == 0)
    {
      errno = EIO;
      return -1;
    }
    b += written;
    remainder -= written;
    total_written += written;
  }
 return static_cast<ssize_t>(total_written);
}

int8_t UARTHardwareTransport::bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;
  if (!is_initialized_.load(std::memory_order_acquire))
  {
    std::cerr << "[UARTHardwareTransport] serial communication is not initialised";
    return -1;
  }
  if (transport_ == -1)
  {
    std::cerr << "[UARTHardwareTransport] no IO to serial port";
    return -1;
  }

  uint8_t packet[] = { UART_START_BYTE, UART_READ, reg_addr, len };
  if (write_all(transport_, packet, 4) == -1)
  {
    std::cerr << "[UARTHardwareTransport] unable to send read request to IMU: " << strerror(errno) << std::endl;
    return -1;
  }

  uint8_t header[2];
  if (read_exact(transport_, header, 2) == -1)
  {
    std::cerr << "[UARTHardwareTransport] could not read serial header: " << strerror(errno) << std::endl;
    return -1;
  }

  // this should never happen
  if (header[1] > len) {
    std::cerr << "[UARTHardwareTransport] header length exceeds requested length" << std::endl;
    return -1; 
  }

  if (header[0] != UART_RESP_READ)
  {
    std::cerr << "[UARTHardwareTransport] invalid header received" << std::endl;
    return -1;
  }

  if (read_exact(transport_, data, header[1]) == -1)
  {
    std::cerr << "[UARTHardwareTransport] could not read serial data: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

int8_t UARTHardwareTransport::bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  (void)dev_addr;

  if (!is_initialized_.load(std::memory_order_acquire))
  {
    std::cerr << "[UARTHardwareTransport] serial communication is not initialised";
    return -1;
  }

  if (transport_ == -1)
  {
    std::cerr << "[UARTHardwareTransport] no IO to serial port";
    return -1;
  }
  std::vector<uint8_t> packet;
  packet.reserve(4 + len);

  packet.push_back(UART_START_BYTE);
  packet.push_back(UART_WRITE);
  packet.push_back(reg_addr);
  packet.push_back(len);
  packet.insert(packet.end(), data, data + len);
  if (write_all(transport_, packet.data(), packet.size()) == -1)
  {
    std::cerr << "[UARTHardwareTransport] serial write error: " << strerror(errno) << std::endl;
    return -1;
  }

  uint8_t ack[2];
  if (read_exact(transport_, ack, 2) < 0)
  {
    std::cerr << "[UARTHardwareTransport] serial read ACK error: " << strerror(errno) << std::endl;
    return -1;
  }
  if (ack[0] != UART_RESP_WRITE)
  {
    std::cerr << "[UARTHardwareTransport] unexpected ACK header: " << std::hex << (int)ack[0] << std::endl;
    return -1;
  }
  if (ack[1] != 0x01)
  {
    std::cerr << "[UARTHardwareTransport] write failed, status: " << std::hex << (int)ack[1] << std::endl;
    return -1;
  }
  return 0;
}
