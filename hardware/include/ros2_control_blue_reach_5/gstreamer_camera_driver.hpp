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

#ifndef ROS2_CONTROL_BLUE_REACH_5__GSTREAMER_CAMERA_DRIVER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__GSTREAMER_CAMERA_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/node.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef struct _GstElement GstElement;
typedef struct _GstAppSink GstAppSink;

namespace ros2_control_blue_reach_5
{

class GStreamerCameraDriver
{
public:
  GStreamerCameraDriver() = default;
  ~GStreamerCameraDriver();

  GStreamerCameraDriver(const GStreamerCameraDriver &) = delete;
  GStreamerCameraDriver & operator=(const GStreamerCameraDriver &) = delete;

  void configure(
    rclcpp::Node * node,
    const std::string & image_topic,
    const std::string & frame_id,
    const std::string & pipeline = default_pipeline());

  void start();
  void stop();
  bool running() const;

  static std::string default_pipeline();

private:
  void loop();
  static void initialize_gstreamer_once();

  rclcpp::Node * node_{nullptr};
  std::string image_topic_;
  std::string frame_id_;
  std::string pipeline_;

  GstElement * gst_pipeline_{nullptr};
  GstAppSink * gst_appsink_{nullptr};

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>
    realtime_image_pub_;

  std::thread camera_thread_;
  std::atomic<bool> camera_thread_running_{false};
};

}  // namespace ros2_control_blue_reach_5

#endif  // ROS2_CONTROL_BLUE_REACH_5__GSTREAMER_CAMERA_DRIVER_HPP_
