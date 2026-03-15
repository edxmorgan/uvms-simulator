// Copyright (C) 2024 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#pragma once
#ifndef ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include "json.hpp"  // nlohmann JSON

namespace a50dvl
{
namespace driver
{

/**
 * @brief A lightweight TCP-based driver that connects to a DVL and
 * reads JSON lines at runtime, invoking a callback when new data arrives.
 */
class DVLDriver
{
public:
  /**
   * @brief Construct a new DVLDriver object
   */
  DVLDriver();

  /**
   * @brief Destroy the DVLDriver object
   *
   * Automatically stops the driver thread if it is running.
   */
  ~DVLDriver();

  /**
   * @brief Start the driver by connecting to the DVL's TCP port
   * and spinning an internal thread that reads JSON lines.
   *
   * @param host The hostname or IP of the DVL
   * @param port The port on which the DVL is streaming data
   * @throws std::runtime_error if the driver is already running or if connection fails
   */
  void start(const std::string & host, int port);

  /**
   * @brief Stop the driver by closing the TCP connection
   * and terminating the reading thread.
   */
  void stop();

  /**
   * @brief Subscribe to new JSON messages read from the DVL.
   *
   * @param callback A function that will be invoked whenever the driver parses
   *        a new JSON line from the DVL.
   */
  void subscribe(const std::function<void(const nlohmann::json &)> & callback);

private:
  /**
   * @brief The internal thread routine that continuously blocks on TCP reads,
   *        parses JSON lines, and invokes the callback (if set).
   */
  void pollData();

private:
  std::atomic<bool> running_;
  std::thread poll_thread_;

  // Connection details
  std::string host_;
  int port_;

  // Boost ASIO objects
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  // Optional callback for incoming JSON
  std::function<void(const nlohmann::json &)> callback_;
};

}  // namespace driver
}  // namespace a50dvl

#endif // ROS2_CONTROL_BLUE_REACH_5__DVLDRIVER_HPP_