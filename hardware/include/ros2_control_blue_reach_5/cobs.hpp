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
#ifndef ROS2_CONTROL_BLUE_REACH_5__COBS_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__COBS_HPP_

#include <vector>

namespace alpha::driver
{

/**
 * @brief Encode serial data using the COBS algorithm.
 *
 * @remark This implementation has been inspired by the following source:
 * https://github.com/gbmhunter/SerialFiller/blob/d678acbf6d29de7042d48c6be8ecef556bb6d857/src/CobsTranscoder.cpp#L19
 *
 * @param data The serial data to encode.
 * @return The serial data that has been encoded with the COBS algorithm.
 */
std::vector<unsigned char> cobsEncode(const std::vector<unsigned char> & data);

/**
 * @brief Decode serial data that has been encoded with the COBS algorithm.
 *
 * @remark This implementation has been inspired by the following source:
 * https://github.com/gbmhunter/SerialFiller/blob/d678acbf6d29de7042d48c6be8ecef556bb6d857/src/CobsTranscoder.cpp#L74
 *
 * @param data The serial data to decode.
 * @return The decoded serial data.
 */
std::vector<unsigned char> cobsDecode(const std::vector<unsigned char> & data);

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__CRC_HPP_
