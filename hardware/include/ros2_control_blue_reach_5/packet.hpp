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
#ifndef ROS2_CONTROL_BLUE_REACH_5__PACKET_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__PACKET_HPP_

#include <vector>

#include "ros2_control_blue_reach_5/device_id.hpp"
#include "ros2_control_blue_reach_5/packet_id.hpp"

namespace alpha::driver
{

class Packet
{
public:
  /**
   * @brief Create a new Packet.
   *
   * @param packet_id The packet type.
   * @param device_id The ID of the device that the data targets.
   * @param data Unencoded serial data.
   */
  Packet(PacketId packet_id, DeviceId device_id, std::vector<unsigned char> data);

  /**
   * @brief Encode the packet's data using the Reach packet structure specification.
   *
   * @return Encoded serial data.
   */
  std::vector<unsigned char> encode() const;

  /**
   * @brief Decode a packet that has been encoded using the Reach communication specification.
   *
   * @param data The encoded serial data.
   * @return The packet obtained from the decoded serial data.
   */
  static Packet decode(const std::vector<unsigned char> & data);

  /**
   * @brief Get the unique packet identifier.
   *
   * @return The packet type.
   */
  PacketId getPacketId() const;

  /**
   * @brief Get the unique device identifier.
   *
   * @return The ID of the device that the packet targets.
   */
  DeviceId getDeviceId() const;

  /**
   * @brief Get the packet data.
   *
   * @note The packet serial data will never be encoded during the lifetime of the object unless
   * the packet is instantiated with the serial data already encoded.
   *
   * @return The packet serial data.
   */
  std::vector<unsigned char> getData() const;

private:
  PacketId packet_id_;
  DeviceId device_id_;
  std::vector<unsigned char> data_;
};

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__PACKET_HPP_

