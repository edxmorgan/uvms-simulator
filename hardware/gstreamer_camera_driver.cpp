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

#include "ros2_control_blue_reach_5/gstreamer_camera_driver.hpp"

extern "C"
{
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
}
#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif

#include <cstring>
#include <mutex>
#include <stdexcept>

#include "rclcpp/qos.hpp"

namespace ros2_control_blue_reach_5
{
namespace
{
constexpr const char * kLoggerName = "GStreamerCameraDriver";
}

GStreamerCameraDriver::~GStreamerCameraDriver()
{
  stop();
}

void GStreamerCameraDriver::configure(
  rclcpp::Node * node,
  const std::string & image_topic,
  const std::string & camera_info_topic,
  const std::string & frame_id,
  const std::string & pipeline,
  const std::string & mirror_image_topic,
  const std::string & mirror_camera_info_topic)
{
  if (!node) {
    throw std::invalid_argument("GStreamerCameraDriver requires a valid rclcpp node");
  }

  stop();

  node_ = node;
  image_topic_ = image_topic;
  camera_info_topic_ = camera_info_topic;
  mirror_image_topic_ = mirror_image_topic;
  mirror_camera_info_topic_ = mirror_camera_info_topic;
  set_frame_id(frame_id);
  pipeline_ = pipeline;

  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SystemDefaultsQoS());
  camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SystemDefaultsQoS());
  if (!mirror_image_topic_.empty()) {
    mirror_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      mirror_image_topic_, rclcpp::SystemDefaultsQoS());
  }
  if (!mirror_camera_info_topic_.empty()) {
    mirror_camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      mirror_camera_info_topic_, rclcpp::SystemDefaultsQoS());
  }
  realtime_image_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>(image_pub_);
}

void GStreamerCameraDriver::start()
{
  if (camera_thread_running_) {
    return;
  }
  if (!node_ || !image_pub_ || !camera_info_pub_ || !realtime_image_pub_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Camera driver was not configured before start");
    return;
  }

  initialize_gstreamer_once();

  gst_pipeline_ = gst_parse_launch(pipeline_.c_str(), nullptr);
  if (!gst_pipeline_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to create GStreamer camera pipeline");
    return;
  }

  gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

  GstElement * appsink_elem = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "camera_sink");
  if (!appsink_elem) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to get appsink element 'camera_sink' from camera pipeline");
    gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
    gst_object_unref(gst_pipeline_);
    gst_pipeline_ = nullptr;
    return;
  }

  gst_appsink_ = GST_APP_SINK(appsink_elem);
  camera_thread_running_ = true;
  camera_thread_ = std::thread(&GStreamerCameraDriver::loop, this);
}

void GStreamerCameraDriver::stop()
{
  camera_thread_running_ = false;

  if (gst_pipeline_) {
    gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
  }

  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }

  if (gst_appsink_) {
    gst_object_unref(GST_OBJECT(gst_appsink_));
    gst_appsink_ = nullptr;
  }

  if (gst_pipeline_) {
    gst_object_unref(gst_pipeline_);
    gst_pipeline_ = nullptr;
  }
}

bool GStreamerCameraDriver::running() const
{
  return camera_thread_running_;
}

void GStreamerCameraDriver::set_frame_id(const std::string & frame_id)
{
  std::lock_guard<std::mutex> lock(frame_id_mutex_);
  frame_id_ = frame_id;
}

std::string GStreamerCameraDriver::default_pipeline()
{
  return
    "udpsrc port=5600 ! application/x-rtp, payload=96 "
    "! rtpjitterbuffer latency=0 drop-on-latency=true faststart-min-packets=1 "
    "! rtph264depay "
    "! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream "
    "! avdec_h264 max-threads=2 "
    "! videoconvert ! video/x-raw,format=(string)BGR "
    "! appsink name=camera_sink emit-signals=true sync=false async=false max-buffers=1 drop=true";
}

void GStreamerCameraDriver::loop()
{
  while (camera_thread_running_) {
    GstSample * sample = gst_app_sink_try_pull_sample(gst_appsink_, 100 * GST_MSECOND);
    if (!sample) {
      continue;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    if (!caps) {
      gst_sample_unref(sample);
      continue;
    }

    GstStructure * structure = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (
      !gst_structure_get_int(structure, "width", &width) ||
      !gst_structure_get_int(structure, "height", &height))
    {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      continue;
    }

    std::string frame_id;
    {
      std::lock_guard<std::mutex> lock(frame_id_mutex_);
      frame_id = frame_id_;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id;
    msg.height = height;
    msg.width = width;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = width * 3;

    const std::size_t image_size =
      static_cast<std::size_t>(msg.step) * static_cast<std::size_t>(height);
    if (map.size < image_size) {
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Camera frame smaller than expected: got %zu bytes, expected %zu bytes",
        static_cast<std::size_t>(map.size), image_size);
      continue;
    }

    msg.data.resize(image_size);
    std::memcpy(msg.data.data(), map.data, msg.data.size());

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = msg.header;
    camera_info.height = msg.height;
    camera_info.width = msg.width;
    camera_info.distortion_model = "plumb_bob";
    camera_info.d.assign(5, 0.0);
    const double fx = static_cast<double>(width);
    const double fy = static_cast<double>(width);
    const double cx = 0.5 * static_cast<double>(width - 1);
    const double cy = 0.5 * static_cast<double>(height - 1);
    camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    camera_info_pub_->publish(camera_info);
    if (mirror_camera_info_pub_) {
      mirror_camera_info_pub_->publish(camera_info);
    }
    if (mirror_image_pub_) {
      mirror_image_pub_->publish(msg);
    }
    if (realtime_image_pub_ && realtime_image_pub_->trylock()) {
      realtime_image_pub_->msg_ = msg;
      realtime_image_pub_->unlockAndPublish();
    } else {
      image_pub_->publish(std::move(msg));
    }
  }
}

void GStreamerCameraDriver::initialize_gstreamer_once()
{
  static std::once_flag gst_init_flag;
  std::call_once(gst_init_flag, []() { gst_init(nullptr, nullptr); });
}

}  // namespace ros2_control_blue_reach_5
