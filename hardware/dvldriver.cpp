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


#include "ros2_control_blue_reach_5/dvldriver.hpp"

#include <iostream> // For logging errors, etc.
#include <sys/socket.h>
#include <boost/system/error_code.hpp>

namespace a50dvl
{
  namespace driver
  {

    DVLDriver::DVLDriver()
        : running_(false)
    {
    }

    DVLDriver::~DVLDriver()
    {
      stop();
    }

    void DVLDriver::start(const std::string &host, int port)
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      if (running_.load())
        throw std::runtime_error("DVLDriver is already running!");
      if (port <= 0 || port > 65535)
        throw std::invalid_argument("DVL port must be in [1, 65535]");

      // EOF leaves a completed, joinable worker. Reap it before replacing transport.
      if (poll_thread_.joinable()) poll_thread_.join();
      host_ = host;
      port_ = port;
      try
      {
        boost::asio::ip::tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(host_, std::to_string(port_));
        socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        boost::asio::connect(*socket_, endpoints);
        running_.store(true);
        poll_thread_ = std::thread(&DVLDriver::pollData, this);
      }
      catch (const std::exception &e)
      {
        running_.store(false);
        socket_.reset();
        throw std::runtime_error(std::string("Failed to start DVL: ") + e.what());
      }
    }

    void DVLDriver::stop()
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      running_.store(false);
      // Shutdown unblocks the reader; close only after it no longer uses the socket.
      if (socket_ && socket_->is_open())
      {
        // Native shutdown does not mutate Boost.Asio's shared socket object.
        ::shutdown(socket_->native_handle(), SHUT_RDWR);
      }
      if (poll_thread_.joinable()) poll_thread_.join();
      socket_.reset();
    }

    void DVLDriver::subscribe(const std::function<void(const nlohmann::json &)> &callback)
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback_ = callback;
    }

    void DVLDriver::pollData()
    {
      boost::asio::streambuf buffer(max_message_bytes);
      while (running_.load())
      {
        try
        {
          // Read until newline
          boost::system::error_code error;
          std::size_t bytes_transferred = boost::asio::read_until(*socket_, buffer, '\n', error);
          (void)bytes_transferred; // Prevents the unused-variable warning

          if (error)
          {
            if (error == boost::asio::error::eof)
            {
              // Remote closed connection
              std::cerr << "[DVLDriver] Remote closed connection.\n";
              break;
            }
            // Otherwise log or handle error
            std::cerr << "[DVLDriver] Error while reading: " << error.message() << std::endl;
            break;
          }

          // Convert buffer to string
          std::istream is(&buffer);
          std::string line;
          std::getline(is, line);

          if (!line.empty())
          {
            try
            {
              // Parse the line as JSON
              auto j = nlohmann::json::parse(line);

              // Dispatch to callback if available
              std::function<void(const nlohmann::json &)> callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback = callback_;
              }
              if (callback) callback(j);
            }
            catch (const std::exception &e)
            {
              // A bad schema/callback must not terminate reception of later frames.
              std::cerr << "[DVLDriver] Rejected message: " << e.what() << "\n";
            }
          }
        }
        catch (const std::exception &e)
        {
          std::cerr << "[DVLDriver] Exception in pollData: " << e.what() << std::endl;
          break;
        }
      }
      running_.store(false);
    }

  } // namespace driver
} // namespace a50dvl