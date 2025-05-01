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

#include "ros2_control_blue_reach_5/sim_reach_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{
    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "CasADi computer from manipulator system: %s", casadi_version.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robots has %lu joints ", info_.joints.size());

        hw_joint_struct_.reserve(info_.joints.size());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            std::string device_id_value = joint.parameters.at("device_id");
            double default_position = stod(joint.parameters.at("home"));

            uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

            Joint::State initialState(default_position);
            hw_joint_struct_.emplace_back(joint.name, device_id, initialState);
            // RRBotSystemMultiInterface has exactly 19 state interfaces
            // and 6 command interfaces on each joint
            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "***********************robots now has %lu joints ", hw_joint_struct_.size());
            if (joint.command_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Joint '%s' has %zu command interfaces. 6 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 27)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Joint '%s'has %zu state interfaces. 27 expected.",
                    joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // SimReachSystemMultiInterfaceHardware has exactly 4 gpio state interfaces
            if (gpio.state_interfaces.size() != 4)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 4 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // SimReachSystemMultiInterfaceHardware has exactly 0 gpio command interfaces
            if (gpio.command_interfaces.size() != 0)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu command interfaces. 0 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "Successfully configured the SimReachSystemMultiInterfaceHardware system interface for serial communication!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    SimReachSystemMultiInterfaceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &hw_joint_struct_[i].current_state_.filtered_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &hw_joint_struct_[i].current_state_.filtered_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].current_state_.acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &hw_joint_struct_[i].current_state_.estimated_acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].current_state_.current));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].current_state_.effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].current_state_.computed_effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT_UNCERTAINTY, &hw_joint_struct_[i].current_state_.computed_effort_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.adaptive_predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &hw_joint_struct_[i].current_state_.state_id));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_joint_struct_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_joint_struct_[i].current_state_.sim_period));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[21].name, &hw_joint_struct_[i].current_state_.gravityF_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[22].name, &hw_joint_struct_[i].current_state_.gravityF_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[23].name, &hw_joint_struct_[i].current_state_.gravityF_z));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[24].name, &hw_joint_struct_[i].current_state_.gravityT_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[25].name, &hw_joint_struct_[i].current_state_.gravityT_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[26].name, &hw_joint_struct_[i].current_state_.gravityT_z));
        };

        // 0-3: payload
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &payload_mass));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &payload_Ixx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &payload_Iyy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &payload_Izz));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SimReachSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].command_state_.position));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].command_state_.velocity));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].command_state_.acceleration));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].command_state_.current));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].command_state_.effort));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].command_state_.computed_effort));
        };
        return command_interfaces;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::prepare_command_mode_switch(
        const std::vector<std::string> & /*start_interfaces*/,
        const std::vector<std::string> & /*stop_interfaces*/)
    {
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "preparing command mode switch");

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Command Mode Switch successful");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Activating... please wait...");
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 0 %f", hw_joint_struct_[0].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 1  %f", hw_joint_struct_[1].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 2  %f", hw_joint_struct_[2].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 3  %f", hw_joint_struct_[3].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Deactivating... please wait...");

        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            //   RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "logging sim %f", hw_joint_struct_[i].command_state_.position);

            hw_joint_struct_[i].current_state_.sim_time = time_seconds;
            hw_joint_struct_[i].current_state_.sim_period = delta_seconds;

            hw_joint_struct_[i].current_state_.filtered_position = hw_joint_struct_[i].command_state_.position;

            hw_joint_struct_[i].current_state_.position = hw_joint_struct_[i].command_state_.position;

            hw_joint_struct_[i].current_state_.velocity = hw_joint_struct_[i].command_state_.velocity;

            hw_joint_struct_[i].current_state_.filtered_velocity = hw_joint_struct_[i].command_state_.velocity;

            hw_joint_struct_[i].current_state_.effort = hw_joint_struct_[i].command_state_.effort;

            hw_joint_struct_[i].current_state_.computed_effort = hw_joint_struct_[i].command_state_.effort;
        };
        payload_mass = 0.0;
        payload_Ixx = 0.0;
        payload_Iyy = 0.0;
        payload_Izz = 0.0;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
