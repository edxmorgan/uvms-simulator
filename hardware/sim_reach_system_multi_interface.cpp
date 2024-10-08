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

    std::size_t number_of_robot_arms = 1;
    // Resize the outer vector to hold the required number of inner vectors
    hw_robot_arm_struct_.resize(number_of_robot_arms); // Replace number_of_robot_arms with the actual number

    // Resize each inner vector
    for (auto &joint_vector : hw_robot_arm_struct_)
    {
      joint_vector.resize(info_.joints.size());
    };

    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));

      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

      RCLCPP_INFO(
          rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
      RCLCPP_INFO(
          rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

      Joint::State initialState{default_position, 0.0, 0.0};
      hw_robot_arm_struct_[0].emplace_back(joint.name, device_id, initialState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint

      if (joint.command_interfaces.size() != 5)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %zu command interfaces. 5 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 19)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "Joint '%s'has %zu state interfaces. 19 expected.",
            joint.name.c_str(),
            joint.state_interfaces.size());
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
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_robot_arm_struct_[0][i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &hw_robot_arm_struct_[0][i].current_state_.filtered_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_robot_arm_struct_[0][i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &hw_robot_arm_struct_[0][i].current_state_.filtered_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_robot_arm_struct_[0][i].current_state_.acceleration));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &hw_robot_arm_struct_[0][i].current_state_.estimated_acceleration));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_robot_arm_struct_[0][i].current_state_.current));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_robot_arm_struct_[0][i].current_state_.effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_robot_arm_struct_[0][i].current_state_.computed_effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT_UNCERTAINTY, &hw_robot_arm_struct_[0][i].current_state_.computed_effort_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION, &hw_robot_arm_struct_[0][i].current_state_.predicted_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION_UNCERTAINTY, &hw_robot_arm_struct_[0][i].current_state_.predicted_position_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY, &hw_robot_arm_struct_[0][i].current_state_.predicted_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY_UNCERTAINTY, &hw_robot_arm_struct_[0][i].current_state_.predicted_velocity_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION, &hw_robot_arm_struct_[0][i].current_state_.adaptive_predicted_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY, &hw_robot_arm_struct_[0][i].current_state_.adaptive_predicted_position_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY, &hw_robot_arm_struct_[0][i].current_state_.adaptive_predicted_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY, &hw_robot_arm_struct_[0][i].current_state_.adaptive_predicted_velocity_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &hw_robot_arm_struct_[0][i].current_state_.state_id));
    };
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  SimReachSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_robot_arm_struct_[0][i].command_state_.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_robot_arm_struct_[0][i].command_state_.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_robot_arm_struct_[0][i].command_state_.acceleration));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_robot_arm_struct_[0][i].command_state_.current));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_robot_arm_struct_[0][i].command_state_.effort));
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

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

      if (std::isnan(hw_robot_arm_struct_[0][i].current_state_.position) || hw_robot_arm_struct_[0][i].current_state_.position == 0)
      {
        hw_robot_arm_struct_[0][i].current_state_.position = hw_robot_arm_struct_[0][i].default_state_.position;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].current_state_.velocity))
      {
        hw_robot_arm_struct_[0][i].current_state_.velocity = 0;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].current_state_.current))
      {
        hw_robot_arm_struct_[0][i].current_state_.current = 0;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].current_state_.acceleration))
      {
        hw_robot_arm_struct_[0][i].current_state_.acceleration = 0;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].command_state_.position))
      {
        hw_robot_arm_struct_[0][i].command_state_.position = 0;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].command_state_.velocity))
      {
        hw_robot_arm_struct_[0][i].command_state_.velocity = 0;
      }
      if (std::isnan(hw_robot_arm_struct_[0][i].command_state_.current))
      {
        hw_robot_arm_struct_[0][i].command_state_.current = 0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

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
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type SimReachSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    hw_robot_arm_struct_[0][0].current_state_.filtered_position = hw_robot_arm_struct_[0][0].command_state_.position;
    hw_robot_arm_struct_[0][1].current_state_.filtered_position = hw_robot_arm_struct_[0][1].command_state_.position;
    hw_robot_arm_struct_[0][2].current_state_.filtered_position = hw_robot_arm_struct_[0][2].command_state_.position;
    hw_robot_arm_struct_[0][3].current_state_.filtered_position = hw_robot_arm_struct_[0][3].command_state_.position;

    hw_robot_arm_struct_[0][0].current_state_.position = hw_robot_arm_struct_[0][0].command_state_.position;
    hw_robot_arm_struct_[0][1].current_state_.position = hw_robot_arm_struct_[0][1].command_state_.position;
    hw_robot_arm_struct_[0][2].current_state_.position = hw_robot_arm_struct_[0][2].command_state_.position;
    hw_robot_arm_struct_[0][3].current_state_.position = hw_robot_arm_struct_[0][3].command_state_.position;

    hw_robot_arm_struct_[0][0].current_state_.velocity = hw_robot_arm_struct_[0][0].command_state_.velocity;
    hw_robot_arm_struct_[0][1].current_state_.velocity = hw_robot_arm_struct_[0][1].command_state_.velocity;
    hw_robot_arm_struct_[0][2].current_state_.velocity = hw_robot_arm_struct_[0][2].command_state_.velocity;
    hw_robot_arm_struct_[0][3].current_state_.velocity = hw_robot_arm_struct_[0][3].command_state_.velocity;

    hw_robot_arm_struct_[0][0].current_state_.filtered_velocity = hw_robot_arm_struct_[0][0].command_state_.velocity;
    hw_robot_arm_struct_[0][1].current_state_.filtered_velocity = hw_robot_arm_struct_[0][1].command_state_.velocity;
    hw_robot_arm_struct_[0][2].current_state_.filtered_velocity = hw_robot_arm_struct_[0][2].command_state_.velocity;
    hw_robot_arm_struct_[0][3].current_state_.filtered_velocity = hw_robot_arm_struct_[0][3].command_state_.velocity;

    hw_robot_arm_struct_[0][0].current_state_.effort = hw_robot_arm_struct_[0][0].command_state_.effort;
    hw_robot_arm_struct_[0][1].current_state_.effort = hw_robot_arm_struct_[0][1].command_state_.effort;
    hw_robot_arm_struct_[0][2].current_state_.effort = hw_robot_arm_struct_[0][2].command_state_.effort;
    hw_robot_arm_struct_[0][3].current_state_.effort = hw_robot_arm_struct_[0][3].command_state_.effort;

    hw_robot_arm_struct_[0][0].current_state_.computed_effort = hw_robot_arm_struct_[0][0].command_state_.effort;
    hw_robot_arm_struct_[0][1].current_state_.computed_effort = hw_robot_arm_struct_[0][1].command_state_.effort;
    hw_robot_arm_struct_[0][2].current_state_.computed_effort = hw_robot_arm_struct_[0][2].command_state_.effort;
    hw_robot_arm_struct_[0][3].current_state_.computed_effort = hw_robot_arm_struct_[0][3].command_state_.effort;

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
