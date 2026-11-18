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
#include <poll.h>
#include <termios.h>
#include <sys/ioctl.h>
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

SerialClient::~SerialClient() { disconnect(); }

void SerialClient::connect(const std::string & device, const int polling_timeout_ms)
{
  if (rx_worker_.joinable() || handle_ >= 0) {
    throw std::logic_error("Serial client must be disconnected before reconnecting");
  }
  if (polling_timeout_ms < 100 || polling_timeout_ms > 25500) {
    throw std::invalid_argument("Serial polling timeout must be between 100 and 25500 ms");
  }
  if (device.empty()) {
    throw std::invalid_argument("Attempted to open file using an unassigned file path.");
  }

  handle_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);

  if (handle_ == -1) {
    throw std::runtime_error(
      "Could not open specified serial device. Please verify that the device is not already being "
      "used and that you have configured read/write permissions.");
  }

  try {
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
    read_failed_.store(false, std::memory_order_relaxed);
    write_failed_.store(false, std::memory_order_relaxed);
    running_.store(true);

    // Start reading data from the serial port
    rx_worker_ = std::thread(&SerialClient::poll, this);

  } catch (...) {
    disconnect();
    throw;
  }

  // Give the RX thread a few ms to start up
  std::this_thread::sleep_for(10ms);

  // Final check to make sure that the client was properly connected
  if (!active()) {
    disconnect();
    throw std::runtime_error(
      "An error occurred while attempting to establish a serial connection.");
  }
}

void SerialClient::disconnect()
{
  running_.store(false);
  if (rx_worker_.joinable()) rx_worker_.join();
  const std::lock_guard<std::mutex> lock(tx_mutex_);
  if (handle_ >= 0) close(handle_);
  handle_ = -1;
}

bool SerialClient::send(const Packet & packet) const
{
  std::vector<unsigned char> encoded_data = packet.encode();
  // Polling, command, and heartbeat threads share this port. Complete each
  // packet under one lock so partial writes cannot interleave packet bytes.
  const std::lock_guard<std::mutex> lock(tx_mutex_);
  if (!active() || handle_ < 0) {
    write_failed_.store(true, std::memory_order_relaxed);
    write_errors_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  std::size_t sent = 0;
  while (sent < encoded_data.size()) {
    const auto count = write(handle_, encoded_data.data() + sent, encoded_data.size() - sent);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      write_failed_.store(true, std::memory_order_relaxed);
      write_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN(rclcpp::get_logger("SerialClient"),
        "Failed to complete packet write to the serial port.");
      return false;
    }
    sent += static_cast<std::size_t>(count);
  }

  if (packet.getPacketId() == PacketId::PacketID_REQUEST_PACKET) {
    for (const auto field : packet.getData())
      trace_.record(static_cast<unsigned>(packet.getDeviceId()),
        static_cast<PacketId>(field), SerialTrace::Stage::requested);
  }
  return true;
}

void SerialClient::registerCallback(
  PacketId packet_type, const std::function<void(Packet)> & callback)
{
  const std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_[packet_type].push_back(callback);
}

bool SerialClient::active() const { return running_.load(); }

std::string SerialClient::diagnostics() const
{
  const std::lock_guard<std::mutex> lock(tx_mutex_);
  int input=-1, output=-1;
  if (handle_>=0) {
    if (ioctl(handle_, TIOCINQ, &input)<0) input=-1;
    if (ioctl(handle_, TIOCOUTQ, &output)<0) output=-1;
  }
  std::ostringstream out;
  out << "rx_bytes=" << received_bytes_.load()
      << " rejected_frames_or_callbacks=" << rejected_frames_.load()
      << " read_errors=" << read_errors_.load() << " write_errors=" << write_errors_.load()
      << " kernel_rx_pending=" << input << " kernel_tx_pending=" << output
      << trace_.report();
  return out.str();
}

void SerialClient::poll()
{
  rclcpp::Clock diagnostic_clock(RCL_STEADY_TIME);
  std::array<unsigned char, 1> data;

  // Create a buffer to store incoming data
  std::vector<unsigned char> buffer;
  buffer.reserve(Packet::kMaxEncodedSize);
  bool discard_until_delimiter = false;

  // Now we can start processing data
  while (running_.load()) {
    // Accumulate this implementation's byte reads until the COBS delimiter.
    // poll also detects hangup, which a VMIN=0 read cannot distinguish from timeout.
    pollfd descriptor{handle_, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 100);
    if (ready < 0 && errno == EINTR) continue;
    if (ready == 0) continue;
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      read_failed_.store(true, std::memory_order_relaxed);
      read_errors_.fetch_add(1, std::memory_order_relaxed);
      running_.store(false);
      RCLCPP_WARN(rclcpp::get_logger("SerialClient"), "Serial receive disconnected; connection stopped.");
      break;
    }
    const int size = read(handle_, &data, 1);

    if (size < 0) {
      if (errno == EINTR) continue;
      read_failed_.store(true, std::memory_order_relaxed);
      running_.store(false);
      read_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN(  // NOLINT
        rclcpp::get_logger("SerialClient"),
        "Serial receive failed; connection stopped.");
      break;
    } else if (size > 0) {
      received_bytes_.fetch_add(size, std::memory_order_relaxed);
      // Reach packets contain at most 254 decoded bytes. This encoder can
      // add three framing bytes for a complete 254-byte nonzero block.
      // A lost delimiter must not grow this buffer indefinitely.
      if (discard_until_delimiter) {
        if (data[0] == 0) discard_until_delimiter = false;
        continue;
      }
      if (buffer.size() >= Packet::kMaxEncodedSize) {
        rejected_frames_.fetch_add(1, std::memory_order_relaxed);
        buffer.clear();
        discard_until_delimiter = data[0] != 0;
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SerialClient"), diagnostic_clock, 1000,
          "Oversized serial frame discarded; resynchronizing at delimiter");
        continue;
      }
      if (buffer.empty() && data[0] == 0) continue;
      buffer.push_back(data[0]);

      // We have reached the end of the packet; now try to decode it
      if (data[0] == 0) {
        try {
          const Packet packet = Packet::decode(buffer);
          trace_.record(static_cast<unsigned>(packet.getDeviceId()), packet.getPacketId(),
            SerialTrace::Stage::decoded);

          // Registration can occur after connect() starts RX. Snapshot under
          // the registry lock; never hold it while executing user callbacks.
          std::vector<std::function<void(Packet)>> callbacks;
          {
            const std::lock_guard<std::mutex> lock(callbacks_mutex_);
            auto it = callbacks_.find(packet.getPacketId());
            if (it != callbacks_.end()) callbacks = it->second;
          }
          {
            for (const auto & callback : callbacks) {
              const auto started = std::chrono::steady_clock::now();
              callback(packet);
              const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
              if (elapsed_ms > 50.0) {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SerialClient"), diagnostic_clock, 1000,
                  "Receive callback blocked serial polling: device=%u packet=%u elapsed=%.1fms",
                  static_cast<unsigned>(packet.getDeviceId()),
                  static_cast<unsigned>(packet.getPacketId()), elapsed_ms);
              }
            }
          }
          trace_.record(static_cast<unsigned>(packet.getDeviceId()), packet.getPacketId(),
            SerialTrace::Stage::dispatched);
        }
        catch (const std::exception & e) {
          rejected_frames_.fetch_add(1, std::memory_order_relaxed);
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SerialClient"), diagnostic_clock, 1000,
            "Receive packet rejected (%zu bytes): %s", buffer.size(), e.what());
        }

        // Empty the buffer before we start reading the next packet
        buffer.clear();
      }
    }
  }
}

}  // namespace alpha::driver
