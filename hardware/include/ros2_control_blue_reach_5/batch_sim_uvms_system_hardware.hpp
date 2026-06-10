// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef ROS2_CONTROL_BLUE_REACH_5__BATCH_SIM_UVMS_SYSTEM_HARDWARE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__BATCH_SIM_UVMS_SYSTEM_HARDWARE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/msg/batch_command.hpp"
#include "ros2_control_blue_reach_5/msg/batch_observation.hpp"
#include "ros2_control_blue_reach_5/srv/reset_batch_sim.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"

namespace ros2_control_blue_reach_5
{

class BatchSimUvmsSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BatchSimUvmsSystemHardware);

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  ~BatchSimUvmsSystemHardware() override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_BLUE_REACH_5_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void stop_ros_interfaces() noexcept;
  void reset_buffers(bool hold_commands);
  void update_selected_interfaces();
  void publish_observation_if_due(const rclcpp::Time & time);
  void command_callback(const msg::BatchCommand::SharedPtr msg);

  static std::string string_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    const std::string & fallback);
  static std::size_t size_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    std::size_t fallback);
  static double double_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    double fallback);

  std::size_t robot_count_{1};
  std::size_t vehicle_state_dim_{BatchUvmsCore::kVehicleStateDim};
  std::size_t arm_joint_count_{BatchUvmsCore::kArmJointCount};
  std::size_t arm_state_dim_{BatchUvmsCore::kArmStateDim};
  std::size_t observation_dim_{BatchUvmsCore::kObservationDim};
  std::size_t vehicle_action_dim_{BatchUvmsCore::kVehicleActionDim};
  std::size_t arm_action_dim_{BatchUvmsCore::kArmActionDim};
  std::size_t action_dim_{BatchUvmsCore::kActionDim};
  std::size_t selected_robot_index_{0};
  double state_update_frequency_{100.0};
  double publish_state_rate_{10.0};
  std::string world_frame_id_{"world"};
  std::string batch_observation_topic_{"/uvms_batch/observations"};
  std::string batch_command_topic_{"/uvms_batch/commands"};
  std::string batch_reset_service_{"/uvms_batch/reset"};

  BatchUvmsCore core_;
  std::vector<double> gpio_state_interfaces_;
  std::vector<double> gpio_command_interfaces_;
  std::vector<double> arm_position_state_interfaces_;
  std::vector<double> arm_velocity_state_interfaces_;
  std::vector<double> arm_effort_state_interfaces_;
  std::vector<double> arm_effort_command_interfaces_;

  std::mutex buffer_mutex_;
  double gpu_step_ms_{0.0};
  double host_to_device_ms_{0.0};
  double device_to_host_ms_{0.0};
  double publish_ms_{0.0};
  double dropped_command_count_{0.0};
  double last_publish_sim_time_{-1.0};
  double last_reset_epoch_command_{0.0};

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<msg::BatchObservation>::SharedPtr observation_pub_;
  rclcpp::Subscription<msg::BatchCommand>::SharedPtr command_sub_;
  rclcpp::Service<srv::ResetBatchSim>::SharedPtr reset_service_;
};

}  // namespace ros2_control_blue_reach_5

#endif  // ROS2_CONTROL_BLUE_REACH_5__BATCH_SIM_UVMS_SYSTEM_HARDWARE_HPP_
