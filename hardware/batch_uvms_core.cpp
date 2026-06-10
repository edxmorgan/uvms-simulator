// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"

#include <algorithm>
#include <utility>

namespace ros2_control_blue_reach_5
{
namespace
{
constexpr double kMockLinearDamping = 0.15;
constexpr double kMockAngularDamping = 0.25;
constexpr double kMockArmDamping = 0.20;
}  // namespace

void BatchUvmsCore::configure(std::size_t robot_count)
{
  robot_count_ = std::max<std::size_t>(1, robot_count);
  selected_robot_index_ = std::min(selected_robot_index_, robot_count_ - 1);
  reset(true);
}

void BatchUvmsCore::set_task(std::shared_ptr<BatchUvmsTask> task)
{
  task_ = std::move(task);
  reset_task();
}

void BatchUvmsCore::reset(bool hold_commands)
{
  observations_.assign(robot_count_ * kObservationDim, 0.0F);
  actions_.assign(robot_count_ * kActionDim, 0.0F);
  rewards_.assign(robot_count_, 0.0F);
  dones_.assign(robot_count_, 0U);
  commands_held_ = hold_commands;
  tick_id_ = 0;
  applied_action_tick_id_ = 0;
  sim_time_ = 0.0;
  sim_period_ = 0.0;
  step_count_ = 0.0;
  reset_task();
}

bool BatchUvmsCore::reset(bool hold_commands, const std::vector<float> & observations)
{
  if (!observations.empty() && !observation_shape_matches(observations.size())) {
    return false;
  }
  reset(hold_commands);
  if (!observations.empty()) {
    observations_ = observations;
    reset_task();
  }
  ++tick_id_;
  return true;
}

bool BatchUvmsCore::set_actions(const std::vector<float> & actions, std::uint64_t tick_id)
{
  if (!action_shape_matches(
      robot_count_, kActionDim, kVehicleActionDim, kArmActionDim, kArmJointCount, actions.size())) {
    return false;
  }
  actions_ = actions;
  applied_action_tick_id_ = tick_id;
  commands_held_ = false;
  return true;
}

bool BatchUvmsCore::set_selected_action(
  std::size_t selected_robot_index,
  const double * vehicle_command6,
  const std::vector<double> & arm_command)
{
  if (arm_command.size() != kArmJointCount || vehicle_command6 == nullptr) {
    return false;
  }

  selected_robot_index_ = clamp_robot_index(selected_robot_index);
  const std::size_t action = selected_action_offset();
  for (std::size_t i = 0; i < 6; ++i) {
    actions_[action + i] = static_cast<float>(vehicle_command6[i]);
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i) {
    actions_[action + kVehicleActionDim + i] = static_cast<float>(arm_command[i]);
  }
  commands_held_ = false;
  return true;
}

void BatchUvmsCore::set_hold_commands(bool hold_commands)
{
  commands_held_ = hold_commands;
}

void BatchUvmsCore::set_selected_robot_index(std::size_t selected_robot_index)
{
  selected_robot_index_ = clamp_robot_index(selected_robot_index);
}

void BatchUvmsCore::step(double dt)
{
  if (dt <= 0.0) {
    return;
  }

  if (!commands_held_) {
    for (std::size_t robot = 0; robot < robot_count_; ++robot) {
      const std::size_t obs = observation_offset(robot);
      const std::size_t act = action_offset(robot);
      const float ax = actions_[act + 0];
      const float ay = actions_[act + 1];
      const float az = actions_[act + 2];
      const float ap = actions_[act + 3];
      const float aq = actions_[act + 4];
      const float ar = actions_[act + 5];

      observations_[obs + kU] += static_cast<float>((ax - kMockLinearDamping * observations_[obs + kU]) * dt);
      observations_[obs + kV] += static_cast<float>((ay - kMockLinearDamping * observations_[obs + kV]) * dt);
      observations_[obs + kW] += static_cast<float>((az - kMockLinearDamping * observations_[obs + kW]) * dt);
      observations_[obs + kP] += static_cast<float>((ap - kMockAngularDamping * observations_[obs + kP]) * dt);
      observations_[obs + kQ] += static_cast<float>((aq - kMockAngularDamping * observations_[obs + kQ]) * dt);
      observations_[obs + kR] += static_cast<float>((ar - kMockAngularDamping * observations_[obs + kR]) * dt);

      observations_[obs + kX] += static_cast<float>(observations_[obs + kU] * dt);
      observations_[obs + kY] += static_cast<float>(observations_[obs + kV] * dt);
      observations_[obs + kZ] += static_cast<float>(observations_[obs + kW] * dt);
      observations_[obs + kRoll] += static_cast<float>(observations_[obs + kP] * dt);
      observations_[obs + kPitch] += static_cast<float>(observations_[obs + kQ] * dt);
      observations_[obs + kYaw] += static_cast<float>(observations_[obs + kR] * dt);

      const std::size_t q = arm_position_offset(robot);
      const std::size_t qd = arm_velocity_offset(robot);
      for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
        const float qdd = actions_[act + kVehicleActionDim + joint] -
          static_cast<float>(kMockArmDamping) * observations_[qd + joint];
        observations_[qd + joint] += static_cast<float>(qdd * dt);
        observations_[q + joint] += static_cast<float>(observations_[qd + joint] * dt);
      }
    }
  }

  sim_time_ += dt;
  sim_period_ = dt;
  step_count_ += 1.0;
  tick_id_ = std::max(tick_id_ + 1, applied_action_tick_id_);
  compute_task();
}

bool BatchUvmsCore::action_shape_matches(
  std::size_t robot_count,
  std::size_t action_dim,
  std::size_t vehicle_action_dim,
  std::size_t arm_action_dim,
  std::size_t arm_joint_count,
  std::size_t action_count) const
{
  return robot_count == robot_count_ && action_dim == kActionDim &&
         vehicle_action_dim == kVehicleActionDim && arm_action_dim == kArmActionDim &&
         arm_joint_count == kArmJointCount && action_count == actions_.size();
}

bool BatchUvmsCore::observation_shape_matches(std::size_t observation_count) const
{
  return observation_count == observations_.size();
}

std::size_t BatchUvmsCore::selected_observation_offset() const
{
  return observation_offset(selected_robot_index_);
}

std::size_t BatchUvmsCore::selected_action_offset() const
{
  return action_offset(selected_robot_index_);
}

std::size_t BatchUvmsCore::observation_offset(std::size_t robot_index) const
{
  return clamp_robot_index(robot_index) * kObservationDim;
}

std::size_t BatchUvmsCore::action_offset(std::size_t robot_index) const
{
  return clamp_robot_index(robot_index) * kActionDim;
}

std::size_t BatchUvmsCore::arm_position_offset(std::size_t robot_index) const
{
  return observation_offset(robot_index) + kVehicleStateDim;
}

std::size_t BatchUvmsCore::arm_velocity_offset(std::size_t robot_index) const
{
  return observation_offset(robot_index) + kVehicleStateDim + kArmJointCount;
}

std::vector<float> BatchUvmsCore::selected_state() const
{
  const auto begin = observations_.begin() + static_cast<std::ptrdiff_t>(selected_observation_offset());
  return std::vector<float>(begin, begin + static_cast<std::ptrdiff_t>(kObservationDim));
}

std::size_t BatchUvmsCore::clamp_robot_index(std::size_t robot_index) const
{
  return std::min(robot_index, robot_count_ - 1);
}

void BatchUvmsCore::compute_task()
{
  if (task_) {
    task_->compute(
      robot_count_, kObservationDim, kActionDim, observations_, actions_, rewards_, dones_);
    return;
  }

  std::fill(rewards_.begin(), rewards_.end(), 0.0F);
  std::fill(dones_.begin(), dones_.end(), 0U);
}

void BatchUvmsCore::reset_task()
{
  if (task_) {
    task_->reset(robot_count_, kObservationDim, observations_, rewards_, dones_);
    return;
  }

  std::fill(rewards_.begin(), rewards_.end(), 0.0F);
  std::fill(dones_.begin(), dones_.end(), 0U);
}

}  // namespace ros2_control_blue_reach_5
