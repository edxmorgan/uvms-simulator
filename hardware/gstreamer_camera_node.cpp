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

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_blue_reach_5/gstreamer_camera_driver.hpp"

namespace ros2_control_blue_reach_5
{

class GStreamerCameraNode : public rclcpp::Node
{
public:
  GStreamerCameraNode()
  : Node("gstreamer_camera_node")
  {
    const std::string image_topic =
      this->declare_parameter<std::string>("image_topic", "/alpha/image_raw");
    const std::string camera_info_topic =
      this->declare_parameter<std::string>("camera_info_topic", "/alpha/camera_info");
    const std::string mirror_image_topic =
      this->declare_parameter<std::string>("mirror_image_topic", "");
    const std::string mirror_camera_info_topic =
      this->declare_parameter<std::string>("mirror_camera_info_topic", "");
    const std::string frame_id =
      this->declare_parameter<std::string>("frame_id", "camera_link");
    const std::string pipeline =
      this->declare_parameter<std::string>(
        "pipeline", GStreamerCameraDriver::default_pipeline());

    camera_driver_ = std::make_unique<GStreamerCameraDriver>();
    camera_driver_->configure(
      this,
      image_topic,
      camera_info_topic,
      frame_id,
      pipeline,
      mirror_image_topic,
      mirror_camera_info_topic);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "frame_id") {
            continue;
          }
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            result.successful = false;
            result.reason = "frame_id must be a string";
            return result;
          }
          const auto frame_id_value = parameter.as_string();
          if (frame_id_value.empty()) {
            result.successful = false;
            result.reason = "frame_id must not be empty";
            return result;
          }
          camera_driver_->set_frame_id(frame_id_value);
          RCLCPP_INFO(get_logger(), "GStreamer camera frame_id set to %s", frame_id_value.c_str());
        }
        return result;
      });
    camera_driver_->start();

    RCLCPP_INFO(
      get_logger(),
      "GStreamer camera publishing %s and %s with frame_id %s",
      image_topic.c_str(),
      camera_info_topic.c_str(),
      frame_id.c_str());
    if (!mirror_image_topic.empty() || !mirror_camera_info_topic.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "GStreamer camera mirroring to %s and %s",
        mirror_image_topic.empty() ? "<none>" : mirror_image_topic.c_str(),
        mirror_camera_info_topic.empty() ? "<none>" : mirror_camera_info_topic.c_str());
    }
  }

  ~GStreamerCameraNode() override
  {
    if (camera_driver_) {
      camera_driver_->stop();
    }
  }

private:
  std::unique_ptr<GStreamerCameraDriver> camera_driver_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace ros2_control_blue_reach_5

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_control_blue_reach_5::GStreamerCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
