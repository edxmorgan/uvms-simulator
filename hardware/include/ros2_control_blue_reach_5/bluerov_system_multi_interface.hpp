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

#ifndef ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>

#include "rclcpp/subscription.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/utils.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "ros2_control_blue_reach_5/dvldriver.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <mutex>
#include "tf2_ros/static_transform_broadcaster.h"
#include <casadi/casadi.hpp>

#include <mavlink/v2.0/common/mavlink.h>
#include "mavros_msgs/msg/mavlink.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ros2_control_blue_reach_5
{

    class BlueRovSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        ~BlueRovSystemMultiInterfaceHardware();
        RCLCPP_SHARED_PTR_DEFINITIONS(BlueRovSystemMultiInterfaceHardware);

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        // ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        // hardware_interface::return_type perform_command_mode_switch(
        //     const std::vector<std::string> &start_interfaces,
        //     const std::vector<std::string> &stop_interfaces) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        void light_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void cameraMountPitch_callback(const std_msgs::msg::Float32::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr light_subscriber_;
        realtime_tools::RealtimeBuffer<std_msgs::msg::Float32::SharedPtr> light_msg_buffer_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr camera_mount_pitch_subscriber_;
        realtime_tools::RealtimeBuffer<std_msgs::msg::Float32::SharedPtr> camera_mount_pitch_msg_buffer_;

        // GStreamer objects for the camera stream
        GstElement *gst_pipeline_{nullptr};
        GstAppSink *gst_appsink_{nullptr};

        // Publisher for sensor_msgs::Image messages
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>
            realtime_image_pub_;

        // Thread for handling the camera stream
        std::thread camera_thread_;
        std::atomic<bool> camera_thread_running_{false};

        // Methods to start, run, and stop the camera stream processing
        void startCameraStream();
        void cameraLoop();
        void stopCameraStream();

        // --- Declaration of the helper function to rebuild the full MAVLink packet ---
        static std::vector<uint8_t> convertToBytes(const mavros_msgs::msg::Mavlink::SharedPtr &msg);

        // Callback for MAVLink subscription.
        void mavlink_data_handler(const mavros_msgs::msg::Mavlink::SharedPtr mavros_data);

        // (Other private members omitted for brevity)

        // Example: subscription for MAVLink messages.
        rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_sub_;

        inline double pressureToDepth(double press_abs_hpa, double water_density);

        // Store the utils function for the robot joints
        casadi_reach_alpha_5::Utils utils_service;

        // Store the state & commands for the robot vehicle
        blue::dynamics::Vehicle hw_vehicle_struct;
        std::string system_name;

        double map_position_x, map_position_y, map_position_z;
        double map_orientaion_w, map_orientaion_x, map_orientaion_y, map_orientaion_z;

        using tf = tf2_msgs::msg::TFMessage;

        tf2::Quaternion q_orig, q_rot, q_new, q_rot_dvl;

        std::shared_ptr<rclcpp::Publisher<tf>> transform_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf>>
            realtime_transform_publisher_;

        std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>> override_rc_pub_;
        std::unique_ptr<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>> rt_override_rc_pub_;

        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> set_params_client_;

        void publishStaticPoseTransform();
        void publishRealtimePoseTransform(
            const rclcpp::Time &time);

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        void stop_thrusters();
        void publishDVLVelocity(const rclcpp::Time &time);
        std::array<double, 36> convert3x3To6x6Covariance(const blue::dynamics::Covariance &linear_cov);

        double delta_seconds = 0.0;
        double time_seconds = 0.0;

        // --- Kalman Filter state variables ---
        casadi::DM x_est_; // State vector: e.g. [px, py, pz, roll, pitch, yaw, u, v, w, p, q, r]
        casadi::DM P_est_; // Covariance: either 12x1 diag or 12x12, depending on your ekf_step function
        casadi::DM Q_;     // Process noise
        casadi::DM R_;     // Measurement noise

        // DVL driver instance
        a50dvl::driver::DVLDriver dvl_driver_;

        // Mutex to protect DVL data
        std::mutex dvl_data_mutex_;

        blue::dynamics::DVLMessage dvl_msg;
        blue::dynamics::DVLVelocityMessage dv_vel;
        blue::dynamics::DVLPoseMessage dvl_pose;

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> dvl_velocity_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>
            realtime_dvl_velocity_publisher_;

        // Add a flag for data readiness (in the header file)
        bool new_dvl_data_available_ = false;

        // Subscriber for MAVROS IMU data
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        bool imu_new_msg_ = false;

        // Use the base class
        std::shared_ptr<rclcpp::Executor> executor_;

        std::thread spin_thread_;
        std::shared_ptr<rclcpp::Node> node_topics_interface_;

        // Mutex for thread-safe IMU data access
        std::mutex imu_mutex_;
        std::mutex mavlink_mutex_;

        std::mutex activate_mutex_;
        std::condition_variable activate_cv_;
        bool activation_complete_ = false;
        bool activation_successful_ = false;

        std::mutex deactivate_mutex_;
        std::condition_variable deactivate_cv_;
        bool deactivation_complete_ = false;
        bool deactivation_successful_ = false;

        std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfListener_;

        bool initial_body_received_ = false;
        tf2::Transform initial_body_transform_;
        casadi::DM last_wrapped_roll;
        casadi::DM unwrap_roll_rt;

        casadi::DM last_wrapped_pitch;
        casadi::DM unwrap_pitch_rt;

        casadi::DM last_wrapped_yaw;
        casadi::DM unwrap_yaw_rt;
        bool first_imu_read = true;
        std::vector<casadi::DM> vehicle_parameters;
    };

} // namespace ros2_control_blue
#endif // ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_
