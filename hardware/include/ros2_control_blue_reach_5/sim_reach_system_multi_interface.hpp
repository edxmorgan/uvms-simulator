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

namespace ros2_control_blue_reach_5
{
    class SimReachSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SimReachSystemMultiInterfaceHardware);

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
        // Store the state & commands for the robot joints
        std::vector<Joint> hw_joint_struct_;

        double delta_seconds;
        double time_seconds;
    };

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__SIM_REACH_SYSTEM_MULTI_INTERFACE_HPP_
