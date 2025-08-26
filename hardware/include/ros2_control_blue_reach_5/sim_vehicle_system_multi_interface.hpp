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

#ifndef ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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

#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/utils.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <casadi/casadi.hpp>

namespace ros2_control_blue_reach_5
{

    class SimVehicleSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SimVehicleSystemMultiInterfaceHardware);

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        // hardware_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State &previous_state) override;

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

        void publishRealtimePoseTransform(const rclcpp::Time &time);
        void publishStaticPoseTransform();
        double delta_seconds;
        double time_seconds;

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread spin_thread_;
        std::shared_ptr<rclcpp::Node> node_topics_interface_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

        // --- Kalman Filter state variables ---
        casadi::DM x_est_; // State vector: e.g. [px, py, pz, roll, pitch, yaw, u, v, w, p, q, r]
        casadi::DM P_est_; // Covariance: either 12x1 diag or 12x12, depending on your ekf_step function
        casadi::DM Q_;     // Process noise
        casadi::DM R_;     // Measurement noise
        std::vector<casadi::DM> vehicle_parameters;
        double P_diag_[12];

        std::vector<casadi::DM> uv_state;
        std::vector<casadi::DM> uv_input;
        std::vector<casadi::DM> vehicle_simulate_argument;
        std::vector<casadi::DM> vehicle_sim;
        std::vector<double> vehicle_next_states;
        std::vector<casadi::DM> vehicle_parameters_new;
        std::vector<double> arm_base_f_ext;
    };

} // namespace ros2_control_blue
#endif // ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_
