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

#include "ros2_control_blue_reach_5/packet.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ros2_control_blue_reach_5/cobs.hpp"
#include "ros2_control_blue_reach_5/crc.hpp"

namespace alpha::driver
{

Packet::Packet(PacketId packet_id, DeviceId device_id, std::vector<unsigned char> data)
: packet_id_(packet_id),
  device_id_(device_id),
  data_(std::move(data))
{
  if (data_.empty()) {
    throw std::invalid_argument("Cannot create a packet with no data.");
  }
}

std::vector<unsigned char> Packet::encode() const
{
  std::vector<unsigned char> data(data_);

  // Add the packet ID and the device ID
  // Note that we need to type cast to the underlying type because enum classes
  // don't implicitly cast to ints (which is a good thing)
  data.push_back(static_cast<unsigned char>(packet_id_));
  data.push_back(static_cast<unsigned char>(device_id_));

  // Length is the current buffer size plus two (length and CRC)
  data.push_back(data.size() + 2);

  // Calculate the CRC from the data and add it to the buffer
  data.push_back(calculateReachCrc8(data));

  // Encode the data using COBS encoding
  std::vector<unsigned char> encoded_data = cobsEncode(data);

  return encoded_data;
}

Packet Packet::decode(const std::vector<unsigned char> & data)
{
  if (data.empty()) {
    throw std::invalid_argument("An empty data packet was received for decoding.");
  }

  // Note that an exception will be raised if the decoding fails
  std::vector<unsigned char> decoded_data = cobsDecode(data);

  if (decoded_data.empty()) {
    throw std::runtime_error("Decoded data is empty");
  }

  // Pop the CRC and make sure that it is defined correctly
  const unsigned char actual_crc = decoded_data.back();
  decoded_data.pop_back();

  const unsigned char expected_crc = calculateReachCrc8(decoded_data);

  if (actual_crc != expected_crc) {
    throw std::runtime_error("The expected and actual CRC values do not match.");
  }

  // Pop the packet length to ensure that a packet of the correct size was
  // provided
  const auto length = static_cast<std::vector<unsigned char>::size_type>(decoded_data.back());
  decoded_data.pop_back();

  if ((decoded_data.size() + 2) != length) {
    throw std::runtime_error("The specified payload size is not equal to the actual payload size.");
  }

  // Get the device ID
  const unsigned char device_id = decoded_data.back();
  decoded_data.pop_back();

  // Get the packet ID
  const unsigned char packet_id = decoded_data.back();
  decoded_data.pop_back();

  return Packet(static_cast<PacketId>(packet_id), static_cast<DeviceId>(device_id), decoded_data);
}

PacketId Packet::getPacketId() const { return packet_id_; }

DeviceId Packet::getDeviceId() const { return device_id_; }

std::vector<unsigned char> Packet::getData() const { return data_; }

}  // namespace alpha::driver
