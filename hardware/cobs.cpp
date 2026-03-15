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

#include "ros2_control_blue_reach_5/cobs.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace alpha::driver
{

std::vector<unsigned char> cobsEncode(const std::vector<unsigned char> & data)
{
  // Initialize the encoded data with 0x00 prepended
  // this will be overwritten once the count to the next 0x00 is determined
  std::vector<unsigned char> encoded_data = {0x00};

  int block_start = 0;
  int current_block_size = 0;

  for (const unsigned char it : data) {
    if (it == 0x00) {
      // Save the total number of elements before the next 0x00
      encoded_data[block_start] = static_cast<uint8_t>(current_block_size + 1);

      // Add a placeholder
      encoded_data.push_back(0x00);

      // Reset the counters
      block_start = encoded_data.size() - 1;
      current_block_size = 0;
    } else {
      // Copy over the data
      encoded_data.push_back(it);
      current_block_size++;

      // Handle the case where the block size is 254 or greater
      // Note that the Reach specification dictates that packets may not be larger
      // than 254 bytes including the footer; however, we handle this case as a sanity check
      if (current_block_size >= 254) {
        encoded_data[block_start] = static_cast<uint8_t>(current_block_size + 1);

        // Add placeholder
        encoded_data.push_back(0x00);

        // Reset counters
        block_start = encoded_data.size() - 1;
        current_block_size = 0;
      }
    }
  }

  encoded_data[block_start] = static_cast<uint8_t>(current_block_size + 1);
  encoded_data.push_back(0x00);

  return encoded_data;
}

std::vector<unsigned char> cobsDecode(const std::vector<unsigned char> & data)
{
  std::vector<unsigned char> decoded_data;
  std::vector<unsigned char>::size_type encoded_data_pos = 0;

  while (encoded_data_pos < data.size()) {
    const int block_size = data[encoded_data_pos] - 1;
    encoded_data_pos++;

    for (int i = 0; i < block_size; ++i) {
      const unsigned char byte = data[encoded_data_pos];

      if (byte == 0x00) {
        throw std::runtime_error("Failed to decode the encoded data.");
      }

      decoded_data.push_back(data[encoded_data_pos]);
      encoded_data_pos++;
    }

    if (data[encoded_data_pos] == 0x00) {
      break;
    }

    if (block_size < 0xFE) {
      decoded_data.push_back(0x00);
    }
  }

  return decoded_data;
}

}  // namespace alpha::driver
