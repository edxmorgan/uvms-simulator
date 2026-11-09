// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_CORE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "ros2_control_blue_reach_5/batch_uvms_task.hpp"

namespace ros2_control_blue_reach_5
{

class BatchUvmsCore
{
public:
  enum VehicleIndex : std::size_t
  {
    kX = 0,
    kY,
    kZ,
    kRoll,
    kPitch,
    kYaw,
    kU,
    kV,
    kW,
    kP,
    kQ,
    kR,
    kVehicleStateCount
  };

  static constexpr std::size_t kVehicleStateDim = kVehicleStateCount;
  static constexpr std::size_t kArmJointCount = 5;
  static constexpr std::size_t kArmStateDim = 2 * kArmJointCount;
  static constexpr std::size_t kObservationDim = kVehicleStateDim + kArmStateDim;
  static constexpr std::size_t kVehicleActionDim = 6;
  static constexpr std::size_t kArmActionDim = kArmJointCount;
  static constexpr std::size_t kActionDim = kVehicleActionDim + kArmActionDim;
  static constexpr std::size_t kVehicleParamDim = 33;
  static constexpr std::size_t kArmParamDim = 81;

  void configure(std::size_t robot_count);
  void set_task(std::shared_ptr<BatchUvmsTask> task);
  void reset(bool hold_commands);
  bool reset(bool hold_commands, const std::vector<double> & observations);
  bool set_observations(const std::vector<double> & observations);
  bool set_vehicle_params(const std::vector<double> & params);
  bool set_arm_params(const std::vector<double> & params);
  void set_arm_environment(
    double endeffector_mass,
    double endeffector_damping,
    double endeffector_stiffness,
    double baumgarte_alpha);
  bool set_actions(const std::vector<double> & actions, std::uint64_t tick_id);
  bool set_selected_action(
    std::size_t selected_robot_index,
    const double * vehicle_command6,
    const std::vector<double> & arm_command);
  void set_hold_commands(bool hold_commands);
  void set_selected_robot_index(std::size_t selected_robot_index);
  void step(double dt);

  bool action_shape_matches(
    std::size_t robot_count,
    std::size_t action_dim,
    std::size_t vehicle_action_dim,
    std::size_t arm_action_dim,
    std::size_t arm_joint_count,
    std::size_t action_count) const;
  bool observation_shape_matches(std::size_t observation_count) const;

  std::size_t robot_count() const { return robot_count_; }
  std::size_t selected_robot_index() const { return selected_robot_index_; }
  std::size_t selected_observation_offset() const;
  std::size_t selected_action_offset() const;
  std::size_t observation_offset(std::size_t robot_index) const;
  std::size_t action_offset(std::size_t robot_index) const;
  std::size_t arm_position_offset(std::size_t robot_index) const;
  std::size_t arm_velocity_offset(std::size_t robot_index) const;

  const std::vector<double> & observations() const { return observations_; }
  const std::vector<double> & actions() const { return actions_; }
  const std::vector<double> & rewards() const { return rewards_; }
  const std::vector<std::uint8_t> & dones() const { return dones_; }
  std::vector<double> selected_state() const;

  bool commands_held() const { return commands_held_; }
  std::uint64_t tick_id() const { return tick_id_; }
  std::uint64_t applied_action_tick_id() const { return applied_action_tick_id_; }
  double sim_time() const { return sim_time_; }
  double sim_period() const { return sim_period_; }
  double step_count() const { return step_count_; }

private:
  std::size_t clamp_robot_index(std::size_t robot_index) const;
  void resize_dynamics_buffers();
  void pack_dynamics_actions();
  void apply_arm_actuator_model();
  void update_arm_lock_mask();
  void clamp_arm_state(std::vector<double> & observations);
  void compute_task();
  void reset_task();

  std::shared_ptr<BatchUvmsTask> task_;
  std::size_t robot_count_{1};
  std::size_t selected_robot_index_{0};
  std::vector<double> observations_;
  std::vector<double> next_observations_;
  std::vector<double> actions_;
  std::vector<double> vehicle_wrench_;
  std::vector<double> arm_torque_;
  std::vector<double> vehicle_params_;
  std::vector<double> arm_params_;
  std::vector<double> dt_;
  std::vector<double> external_wrench_;
  std::vector<double> ee_mass_;
  std::vector<double> ee_damping_;
  std::vector<double> ee_stiffness_;
  std::vector<double> lock_mask_;
  std::vector<std::uint8_t> arm_lock_state_;
  std::vector<double> baumgarte_alpha_;
  std::vector<double> rewards_;
  std::vector<std::uint8_t> dones_;
  bool commands_held_{true};
  std::uint64_t tick_id_{0};
  std::uint64_t applied_action_tick_id_{0};
  double sim_time_{0.0};
  double sim_period_{0.0};
  double step_count_{0.0};
};

}  // namespace ros2_control_blue_reach_5

#endif  // ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_CORE_HPP_
