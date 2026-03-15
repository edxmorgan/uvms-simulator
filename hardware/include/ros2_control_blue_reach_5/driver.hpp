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

#pragma once
#ifndef ROS2_CONTROL_BLUE_REACH_5__DRIVER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ros2_control_blue_reach_5/mode.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
#include "ros2_control_blue_reach_5/packet_id.hpp"
#include "ros2_control_blue_reach_5/serial_client.hpp"

namespace alpha::driver
{

class Driver
{
public:
  /**
   * @brief Construct a new Driver object.
   */
  Driver();

  /**
   * @brief Start the driver.
   *
   * @note This will attempt to connect the serial client and start the heartbeat.
   *
   * @param serial_port The serial port that the manipulator is available at.
   * @param heartbeat_timeout The maximum time (s) between heartbeat messages before the connection
   * is considered timed out. This must be greater than 1 second; defaults to 3 seconds.
   */
  void start(const std::string & serial_port, int heartbeat_timeout = 3, bool verbose = true);

  /**
   * @brief Stop the driver.
   *
   * @note This will stop the heartbeat and disconnect the serial client.
   */
  void stop();

  /**
   * @brief Set the mode of a device.
   *
   * @param mode The desired operating mode.
   * @param device The device whose mode should be configured.
   */
  void setMode(Mode mode, DeviceId device) const;

  /**
   * @brief Set the velocity of a device.
   *
   * @note If the velocity setpoint is set above the velocity limit, the velocity will be set to the
   * limit.
   *
   * @param velocity The velocity setpoint. For rotational devices, the velocity should be provided
   * in rad/s. For linear devices, the velocity should be provided in mm/s.
   * @param device The device whose velocity should be set.
   */
  void setVelocity(float velocity, DeviceId device) const;

  /**
   * @brief Set the position of a device.
   *
   * @note If the position setpoint is outside of the configured limits, the command is ignored.
   *
   * @param position The position setpoint. For rotational devices, this should be an angle in the
   * range [0, 2pi]. For linear devices, the velocity should be a distance in mm.
   * @param device device whose position should be set
   */
  void setPosition(float position, DeviceId device) const;

  /**
   * @brief Set the relative position of a device.
   *
   * @note The actuator will move from its current position by the amount specified.
   *
   * @param relative_position The relative position setpoint. For rotational devices, this should be
   * an angle in the range [0, 2pi]. For linear devices, the velocity should be a distance in mm.
   * @param device device whose relative position should be set.
   */
  void setRelativePosition(float relative_position, DeviceId device) const;

  /**
   * @brief Set the current setpoint of the motor windings.
   *
   * @note Demanding current that is out of range will set the current to its maximum.
   *
   * @param current The current setpoint (mAh).
   * @param device The device whose current should be set.
   */
  void setCurrent(float current, DeviceId device) const;

  /**
   * @brief Request a packet from the Alpha manipulator.
   *
   * @remark This intended to be used in conjunction with the @ref subscribe method.
   *
   * @param packet_type The packet type that the manipulator should send.
   * @param device The device that should send the packet.
   */
  void request(PacketId packet_type, DeviceId device) const;

  /**
   * @brief Request multiple packets from the Alpha manipulator.
   *
   * @note Up to to 10 packets may be requested at once.
   *
   * @param packet_types The vector of packet types that the manipulator should send.
   * @param device The device that should send the packets.
   */
  void request(std::vector<PacketId> & packet_types, DeviceId device) const;

  /**
   * @brief Register a callback function to be executed when a packet of the specified type is
   * received from the manipulator.
   *
   * @note This is intended to be used in conjunction with the @ref request method.
   *
   * @param packet_type The packet type that should signal this function.
   * @param callback The function to execute when a packet of the given type is received from the
   * manipulator.
   */
  void subscribe(PacketId packet_type, const std::function<void(Packet)> & callback);

private:
  /**
   * @brief Send a float message.
   *
   * @note This is a helper method used to cast a float value to a vector of unsigned chars and
   * send the resulting data.
   *
   * @param value The float value to send.
   * @param packet_type The type of packet to construct.
   * @param device The device that should receive the message.
   */
  void sendFloat(float value, PacketId packet_type, DeviceId device) const;

  /**
   * @brief Enable heartbeat messages from the Alpha manipulator.
   *
   * @note There isn't a single dedicated message available for heartbeat messages. Because of this,
   * we specify the Model Number packet as the heartbeat message.
   *
   * @remark While a more helpful message could be requested as the heartbeat (e.g., position,
   * velocity, mode, etc.), the interface would become less usable and a bit more confusing if some
   * state messages are broadcasted but others aren't. Furthermore, those messages may not be
   * desired by users. Therefore, we leave it up to the users to request state information.
   *
   * @param freq The frequency (Hz) that the heartbeat packets should be sent at.
   */
  void enableHeartbeat(int freq);

  /**
   * @brief Disable heartbeat messages from the Alpha manipulator.
   *
   * @note This is equivalent to setting the heartbeat frequency to 0.
   */
  void disableHeartbeat();

  /**
   * @brief Set the frequency that heartbeat messages should be sent.
   *
   * @param freq The frequency (Hz) that the heartbeat messages should be sent at.
   */
  void setHeartbeatFreq(int freq);

  /**
   * @brief Monitor the latest heartbeat timestamp to determine whether or not the connection has
   * timed out.
   *
   * @note This method is not designed to be a watchdog. If a timeout occurs, it is the user's
   * responsibility to respond.
   *
   * @param heartbeat_timeout_ms maximum allowable time between heartbeat messages before notifying
   * users that a timeout may have occurred
   */
  void monitorHeartbeat(int heartbeat_timeout_ms, bool verbose) const;

  SerialClient client_;

  // Heartbeat monitor
  std::thread heartbeat_worker_;
  std::atomic<bool> running_{false};
  mutable std::mutex last_heartbeat_lock_;
  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;
};

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__DRIVER_HPP_
