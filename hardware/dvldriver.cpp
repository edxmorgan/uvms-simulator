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
      // Ensure we stop cleanly if the destructor is called unexpectedly
      if (running_.load())
      {
        stop();
      }
    }

    void DVLDriver::start(const std::string &host, int port)
    {
      if (running_.load())
      {
        throw std::runtime_error("DVLDriver is already running!");
      }

      host_ = host;
      port_ = port;
      running_.store(true);

      // Attempt to connect to the DVL
      try
      {
        boost::asio::ip::tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(host_, std::to_string(port_));
        socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        boost::asio::connect(*socket_, endpoints);
      }
      catch (const std::exception &e)
      {
        running_.store(false);
        throw std::runtime_error(std::string("Failed to connect to DVL: ") + e.what());
      }

      // Spin the poll thread for reading data
      poll_thread_ = std::thread(&DVLDriver::pollData, this);
    }

    void DVLDriver::stop()
    {
      if (!running_.load())
      {
        return;
      }
      running_.store(false);

      // Close the socket so the poll thread is unblocked
      if (socket_ && socket_->is_open())
      {
        boost::system::error_code ec;
        socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        socket_->close(ec);
      }

      // Join the polling thread
      if (poll_thread_.joinable())
      {
        poll_thread_.join();
      }
    }

    void DVLDriver::subscribe(const std::function<void(const nlohmann::json &)> &callback)
    {
      callback_ = callback;
    }

    void DVLDriver::pollData()
    {
      boost::asio::streambuf buffer;
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
              if (callback_)
              {
                callback_(j);
              }
            }
            catch (nlohmann::json::parse_error &e)
            {
              std::cerr << "[DVLDriver] JSON parse error: " << e.what() << "\n";
              std::cerr << "[DVLDriver] Offending line: " << line << "\n";
            }
          }
        }
        catch (const std::exception &e)
        {
          std::cerr << "[DVLDriver] Exception in pollData: " << e.what() << std::endl;
          break;
        }
      }
    }

  } // namespace driver
} // namespace a50dvl