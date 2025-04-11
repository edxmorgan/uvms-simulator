// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ros2_control_blue_reach_5/serial_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "ros2_control_blue_reach_5/packet.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace alpha::driver
{

void SerialClient::connect(const std::string & device, const int polling_timeout_ms)
{
  if (device.empty()) {
    throw std::invalid_argument("Attempted to open file using an unassigned file path.");
  }

  handle_ = open(device.c_str(), O_RDWR);

  if (handle_ == -1) {
    throw std::runtime_error(
      "Could not open specified serial device. Please verify that the device is not already being "
      "used and that you have configured read/write permissions.");
  }

  port_status_ = PortState::kOpen;

  struct termios tty;

  // Get the terminal existing settings
  if (tcgetattr(handle_, &tty) < 0) {
    throw std::runtime_error("Unable to get the current terminal configurations.");
  }

  // Set the baudrate to 115200 as defined by the Alpha Reach specification
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);         // Enable the receiver and set the mode to local mode
  tty.c_cflag &= ~CSIZE;                   // Mask the character size bits
  tty.c_cflag |= CS8;                      // Use 8 data bits per byte
  tty.c_cflag &= ~PARENB;                  // Disable parity
  tty.c_cflag &= ~CSTOPB;                  // Only one stop bit is used
  tty.c_cflag &= ~CRTSCTS;                 // Disable RTS/CTS hardware flow control
  tty.c_lflag &= ~ICANON;                  // Disable canonical input
  tty.c_lflag &= ~ECHO;                    // Disable echo
  tty.c_lflag &= ~ECHOE;                   // Disable erasure
  tty.c_lflag &= ~ECHONL;                  // Disable new-line echo
  tty.c_lflag &= ~ISIG;                    // Disable signals
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off IO control
  tty.c_iflag &=
    ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
      ICRNL);  // Disable special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Disable output post-processing
  tty.c_oflag &= ~ONLCR;  // Disable conversion of newline to carriage return

  tty.c_cc[VTIME] =
    static_cast<cc_t>(polling_timeout_ms / 100);  // Set the timeout; convert to deciseconds
  tty.c_cc[VMIN] = 0;  // We can't set this in a thread otherwise it may block indefinitely

  // Save the configurations
  if (tcsetattr(handle_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Unable to save the terminal configurations.");
  }

  // Indicate that we are now running
  // Used by the rx_worker_ main loop
  running_.store(true);

  // Start reading data from the serial port
  rx_worker_ = std::thread(&SerialClient::poll, this);

  // Give the RX thread a few ms to start up
  std::this_thread::sleep_for(10ms);

  // Final check to make sure that the client was properly connected
  if (!active()) {
    throw std::runtime_error(
      "An error occurred while attempting to establish a serial connection.");
  }
}

void SerialClient::disconnect()
{
  running_.store(false);
  rx_worker_.join();
  close(handle_);
}

bool SerialClient::send(const Packet & packet) const
{
  std::vector<unsigned char> encoded_data = packet.encode();

  if (write(handle_, encoded_data.data(), encoded_data.size()) < 0) {
    RCLCPP_WARN(  // NOLINT
      rclcpp::get_logger("SerialClient"), "Failed to write the encoded packet to the serial port.");

    return false;
  }

  return true;
}

void SerialClient::registerCallback(
  PacketId packet_type, const std::function<void(Packet)> & callback)
{
  callbacks_[packet_type].push_back(callback);
}

bool SerialClient::active() const { return (running_.load() && port_status_ == PortState::kOpen); }

void SerialClient::poll() const
{
  std::array<unsigned char, 1> data;

  // Start by waiting for the end of a packet
  // Once we have reached the end of the packet then we can start processing data like normal
  while (running_.load()) {
    const int size = read(handle_, &data, 1);

    if (size > 0 && data[0] == 0) {
      break;
    }
  }

  // Create a buffer to store incoming data
  std::vector<unsigned char> buffer;

  // Now we can start processing data
  while (running_.load()) {
    // Note that we have to read byte-by-byte. This is because the Reach protocol doesn't include
    // a header which defines the size of the packet. Instead we have to read until there is a
    // packet delimiter (0x00) and process that data.
    const int size = read(handle_, &data, 1);

    if (size < 0) {
      RCLCPP_WARN(  // NOLINT
        rclcpp::get_logger("SerialClient"),
        "An error occurred while attempting to read a message from the serial port.");
    } else if (size > 0) {
      buffer.push_back(data[0]);

      // We have reached the end of the packet; now try to decode it
      if (data[0] == 0) {
        try {
          const Packet packet = Packet::decode(buffer);

          // We need to use the find method here instead of a normal [] indexing operation because
          // the [] does not have const overloading which we want for this to be thread-safe
          auto it = callbacks_.find(packet.getPacketId());

          // If a callback exists for the message, execute it
          if (it != callbacks_.end()) {
            for (const auto & callback : it->second) {
              callback(packet);
            }
          }
        }
        catch (const std::exception & e) {
          RCLCPP_DEBUG(rclcpp::get_logger("SerialClient"), e.what());  // NOLINT
        }

        // Empty the buffer before we start reading the next packet
        buffer.clear();
      }
    }
  }
}

}  // namespace alpha::driver
