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

#ifndef ROS2_CONTROL_BLUE_REACH_5__SIM_REACH_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__SIM_REACH_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"

#include "ros2_control_blue_reach_5/driver.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/utils.hpp"

#include <casadi/casadi.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace ros2_control_blue_reach_5
{
    using tf = tf2_msgs::msg::TFMessage;
    class SimReachSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SimReachSystemMultiInterfaceHardware);

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
        double payload_mass = 0;
        double payload_Ixx = 0;
        double payload_Iyy = 0;
        double payload_Izz = 0;

        std::string system_name;

        // Store the state & commands for the robot joints
        std::vector<Joint> hw_joint_struct_;

        std::string robot_prefix;
        
         // One EKF per joint
        std::vector<casadi::DM> x_est_list_;             // each is 3x1
        std::vector<casadi::DM> P_est_list_;             // each is 3x3
        std::vector<casadi::DM> Q_list_;                 // each is 3x3
        std::vector<casadi::DM> R_list_;                 // each is 2x2
        std::vector<std::array<double, 3>> P_diag_list_; // cached diagonals for quick inspection

        std::shared_ptr<rclcpp::Node> node_frames_interface_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread spin_thread_;

        rclcpp::Publisher<tf>::SharedPtr frame_transform_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf>> realtime_frame_transform_publisher_;

        double delta_seconds;
        double time_seconds;

        std::vector<casadi::DM> arm_state;
        std::vector<casadi::DM> arm_torques;

        std::vector<DM> arm_simulate_argument;
        std::vector<DM> arm_sim;
        std::vector<double> arm_next_states;

        // cache FK outputs computed in read
        std::vector<casadi::DM> T_i_;
        std::vector<casadi::DM> T_com_i_;

        // Store the utils function for the robot joints
        casadi_reach_alpha_5::Utils utils_service;
    };

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__SIM_REACH_SYSTEM_MULTI_INTERFACE_HPP_
