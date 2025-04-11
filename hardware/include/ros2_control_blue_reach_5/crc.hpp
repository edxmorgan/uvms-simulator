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
#ifndef ROS2_CONTROL_BLUE_REACH_5__CRC_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__CRC_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace alpha::driver
{

/**
 * @brief The CRC initial value specified by the Reach protocol.
 */
const unsigned char kInitialValue = 0x00;

/**
 * @brief The CRC final XOR value specified by the Reach protocol.
 */
const unsigned char kFinalXorValue = 0xFF;

/**
 * @brief The Reach protocol specifies that the CRC input should be reflected.
 */
const bool kInputReflected = true;

/**
 * @brief The Reach protocol specifies that the CRC result should be reflected.
 */
const bool kResultReflected = true;

/**
 * @brief Lookup table used for the CRC calculation.
 *
 * @note This is pre-computed to speed up the CRC calculation.
 *
 * @remarks To recompute by hand, use the following parameters:
 *   - Polynomial: 0x4D
 *   - Initial Value: 0x00
 *   - Final XOR Value: 0xFF
 *   - Input Reflected: True
 *   - Result Reflected: True
 *   - Use Reflected Lookup Table: False \n
 *   \n
 *   For help computing the lookup table, you can use the handy-dandy calculator available
 *   at this link below: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 */
const std::array<unsigned char, 256> kCrc8LookupTable = {
  0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
  0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
  0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
  0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
  0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
  0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
  0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
  0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
  0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
  0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
  0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
  0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
  0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
  0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
  0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
  0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8};

/**
 * @brief Reflect data about the center bit.
 *
 * @remark This implementation has been inspired by Micheal Barr's "CRC Series,
 * Part 3: CRC Implementation Code in C/C++" (2000), retrieved Jan 6, 2023:
 * https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 * @param data The data to reflect.
 * @param size The size of the data being reflected.
 * @return The reflected data.
 */
unsigned char reflect(std::uint64_t data, int size);

/**
 * @brief Calculate the CRC value for a message using the CRC8 algorithm.
 *
 * @remark This implementation has been inspired by Micheal Barr's "CRC Series,
 * Part 3: CRC Implementation Code in C/C++" (2000), retrieved Jan 6, 2023:
 * https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 * @param data The serial data whose CRC should be calculated.
 * @param initial_value The CRC initial value.
 * @param final_xor_value The CRC final XOR value.
 * @param input_reflected Reflect the input.
 * @param result_reflected Reflect the result.
 * @param lookup_table The pre-computed lookup table used for CRC calculation.
 * @return The calculated CRC8 value for the provided serial data.
 */
unsigned char calculateCrc8(
  const std::vector<unsigned char> & data, unsigned char initial_value,
  unsigned char final_xor_value, bool input_reflected, bool result_reflected,
  const std::array<unsigned char, 256> & lookup_table);

/**
 * @brief Calculate the CRC value for a message using the Reach serial protocol specification.
 *
 * @remark This is a wrapper for the @ref calculateCrc8 method defined using the Reach
 * protocol.
 *
 * @param data The serial data whose CRC should be calculated using the Reach specification.
 * @return The CRC8 value calculated using the Reach serial protocol.
 */
unsigned char calculateReachCrc8(const std::vector<unsigned char> & data);

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__CRC_HPP_
