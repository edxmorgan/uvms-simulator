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

#include "ros2_control_blue_reach_5/driver.hpp"

#include <vector>

#include "ros2_control_blue_reach_5/device_id.hpp"
#include "ros2_control_blue_reach_5/packet_id.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace alpha::driver
{

Driver::Driver()
{
  subscribe(PacketId::PacketID_MODEL_NUMBER, [this](const Packet &) -> void {
    const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);
    last_heartbeat_ = std::chrono::steady_clock::now();
  });
}

void Driver::start(const std::string & serial_port, int heartbeat_timeout, bool verbose)
{
  if (heartbeat_timeout < 1) {
    throw std::invalid_argument("The heartbeat timeout must be greater than 1 second.");
  }

  // Attempt to connect the serial client
  // We don't expose the VTIME timeout to the user API to avoid usability concerns
  client_.connect(serial_port);

  // Disable any previous heartbeat configurations
  disableHeartbeat();

  // Configure the new heartbeat request
  // The heartbeat has a minimum frequency of one message per second, so we set it to that
  enableHeartbeat(1);

  {
    const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);
    last_heartbeat_ = std::chrono::steady_clock::now();
  }

  running_.store(true);

  // Start the thread that monitors heartbeats from the manipulator
  heartbeat_worker_ = std::thread(&Driver::monitorHeartbeat, this, heartbeat_timeout, verbose);
}

void Driver::stop()
{
  disableHeartbeat();
  running_.store(false);
  heartbeat_worker_.join();
  client_.disconnect();
}

void Driver::setMode(Mode mode, DeviceId device) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a message to the manipulator without an active connection!");
  }

  const std::vector<unsigned char> mode_setting = {static_cast<unsigned char>(mode)};

  const Packet packet(PacketId::PacketID_MODE, device, mode_setting);

  client_.send(packet);
}

void Driver::setVelocity(float velocity, DeviceId device) const
{
  sendFloat(velocity, PacketId::PacketID_VELOCITY, device);
}

void Driver::setPosition(float position, DeviceId device) const
{
  sendFloat(position, PacketId::PacketID_POSITION, device);
}

void Driver::setRelativePosition(float relative_position, DeviceId device) const
{
  sendFloat(relative_position, PacketId::PacketID_RELATIVE_POSITION, device);
}

void Driver::setCurrent(float current, DeviceId device) const
{
  sendFloat(current, PacketId::PacketID_CURRENT, device);
}

void Driver::request(PacketId packet_type, DeviceId device) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a request to the manipulator without an active connection!");
  }

  const std::vector<unsigned char> request_type = {static_cast<unsigned char>(packet_type)};

  const Packet packet(PacketId::PacketID_REQUEST_PACKET, device, request_type);

  client_.send(packet);
}

void Driver::request(std::vector<PacketId> & packet_types, DeviceId device) const
{
  if (packet_types.size() > 10) {
    throw std::invalid_argument("Cannot request more than 10 packets from a device at once.");
  }

  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a request to the manipulator without an active connection!");
  }

  std::vector<unsigned char> request_types(packet_types.size());

  // Cast to unsigned char
  for (auto type : packet_types) {
    request_types.push_back(static_cast<unsigned char>(type));
  }

  const Packet packet(PacketId::PacketID_REQUEST_PACKET, device, request_types);

  client_.send(packet);
}

void Driver::subscribe(PacketId packet_type, const std::function<void(Packet)> & callback)
{
  client_.registerCallback(packet_type, callback);
}

void Driver::sendFloat(float value, PacketId packet_type, DeviceId device_id) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a message to the manipulator without an active connection!");
  }

  // Convert the float into a vector of bytes
  std::vector<unsigned char> reinterpretted_vector(sizeof(value));
  std::memcpy(reinterpretted_vector.data(), &value, sizeof(value));

  const Packet packet(packet_type, device_id, reinterpretted_vector);

  client_.send(packet);
}

void Driver::enableHeartbeat(int freq)
{
  // We request the model number as the heartbeat because there isn't an official heartbeat message
  const std::vector<unsigned char> heartbeat_config = {
    static_cast<unsigned char>(alpha::driver::PacketId::PacketID_MODEL_NUMBER)};

  const Packet packet(PacketId::PacketID_HEARTBEAT_SET, DeviceId::kAllJoints, heartbeat_config);

  client_.send(packet);

  setHeartbeatFreq(freq);
}

void Driver::disableHeartbeat() { setHeartbeatFreq(0); }

void Driver::setHeartbeatFreq(int freq)
{
  const std::vector<unsigned char> heartbeat_frequency = {static_cast<unsigned char>(freq)};
  const Packet packet(PacketId::PacketID_HEARTBEAT_FREQUENCY_SET, DeviceId::kAllJoints, heartbeat_frequency);
  client_.send(packet);
}

void Driver::monitorHeartbeat(int heartbeat_timeout, bool verbose) const
{
  while (running_.load()) {
    // Make sure that the lock is properly scoped so that we don't accidentally keep the lock
    // forever
    {
      const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);

      if (
        std::chrono::steady_clock::now() - last_heartbeat_ >
        std::chrono::seconds(heartbeat_timeout)) {

        if (verbose) {
                  RCLCPP_WARN(  // NOLINT
          rclcpp::get_logger("AlphaDriver"),
          "Timeout occurred; the system has not received a heartbeat message in the last %d "
          "seconds",
          heartbeat_timeout);
        }
      }
    }

    // We don't need to check more often than our timeout requires us to
    std::this_thread::sleep_for(std::chrono::seconds(heartbeat_timeout));
  }
}

}  // namespace alpha::driver
