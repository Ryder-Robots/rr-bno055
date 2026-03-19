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

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include "bno055.h"
#include "rr_bno055/hardware_transport.hpp"

namespace rr_bno055
{

/**
 * @brief UART transport for Bosch IMU sensors using the BNO055 serial protocol.
 *
 * Opens a UART device (e.g. `/dev/ttyAMA0`) and communicates using the
 * BNO055 register-access serial protocol.  Each transaction is framed with
 * a start byte, command byte, register address, length, and data payload.
 *
 * Wire the BNO055 protocol-select pins for UART mode: PS1=GND, PS0=3.3 V.
 * Cross the TX/RX lines: BNO055 TX → Pi RX, BNO055 RX → Pi TX.
 *
 * Protocol frame format (read request):
 * @code
 *   → AA 01 <reg> <len>
 *   ← BB <len> <data...>
 * @endcode
 *
 * Protocol frame format (write request):
 * @code
 *   → AA 00 <reg> <len> <data...>
 *   ← EE <status>
 * @endcode
 */
class UARTHardwareTransport : public HardwareTransport
{
public:
  /**
   * @brief Opens and configures the UART device described by @p transport_config.
   *
   * Configures the port to 115200 baud, 8N1, no flow control.
   *
   * @param transport_config  Must have `type == UART` and a valid `device`
   *                          path (e.g. `/dev/ttyAMA0`).
   * @return  A valid file descriptor on success.
   * @throws  std::runtime_error if the device cannot be opened or configured.
   */
  int initialize_trans(std::shared_ptr<TransportConfig> transport_config) override;

  /**
   * @brief Reads @p len bytes from register @p reg_addr into @p data.
   *
   * Sends a read-request frame and blocks until the response frame is received.
   *
   * @return 0 on success, -1 on framing error or I/O failure.
   */
  int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) override;

  /**
   * @brief Writes @p len bytes from @p data to register @p reg_addr.
   *
   * Sends a write-request frame and blocks until the acknowledgement byte
   * is received.
   *
   * @return 0 on success, -1 on framing error or I/O failure.
   */
  int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len) override;

  static constexpr uint8_t UART_START_BYTE  = 0xAA; ///< Start of every request frame.
  static constexpr uint8_t UART_WRITE       = 0x00; ///< Command byte for write requests.
  static constexpr uint8_t UART_READ        = 0x01; ///< Command byte for read requests.
  static constexpr uint8_t UART_RESP_WRITE  = 0xEE; ///< First byte of a write-response frame.
  static constexpr uint8_t UART_RESP_READ   = 0xBB; ///< First byte of a read-response frame.

private:
  /// Reads exactly @p len bytes from @p fd, retrying on short reads.
  ssize_t read_exact(int fd, uint8_t* buf, size_t len);

  /// Writes exactly @p nbyte bytes to @p fildes, retrying on short writes.
  ssize_t write_all(int fildes, const void* buf, size_t nbyte);
};
}  // namespace rr_bno055