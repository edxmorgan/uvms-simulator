#include "ros2_control_blue_reach_5/arm_integration_policy.hpp"
// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "ros2_control_blue_reach_5/cpu_dynamics/uvms_cpu_dynamics.h"
#include "ros2_control_blue_reach_5/arm_actuator_model.hpp"

namespace ros2_control_blue_reach_5
{
namespace
{
constexpr std::size_t kVehicleParamDim = 33;
constexpr std::size_t kArmParamDim = 81;
constexpr std::size_t kExternalWrenchDim = 6;
constexpr std::size_t kLockMaskDim = 4;
constexpr double kJointLockOnDeadband = 0.05;
constexpr double kJointLockOffDeadband = 0.10;
constexpr double kTorqueCurrentZeroEpsilon = 1.0e-9;

constexpr double kArmPositionMin[BatchUvmsCore::kArmJointCount] = {
  1.0, 0.01, 0.01, 0.01, 0.0004};
constexpr double kArmPositionMax[BatchUvmsCore::kArmJointCount] = {
  5.5, 3.4, 3.4, 5.7, 0.0137};
constexpr double kArmCurrentMax[BatchUvmsCore::kArmJointCount] = {
  600.0, 400.0, 400.0, 400.0, 400.0};
constexpr double kArmMotorKt[BatchUvmsCore::kArmJointCount] = {
  90.6, 90.6, 90.6, 50.0, 50.0};
constexpr double kArmForwardStaticCurrent[BatchUvmsCore::kArmJointCount] = {
  43.0, 43.0, 43.0, 10.0, 43.0};
constexpr double kArmBackwardStaticCurrent[BatchUvmsCore::kArmJointCount] = {
  43.0, 43.0, 43.0, 20.0, 43.0};

std::vector<double> default_vehicle_params()
{
  return {
    3.72028553e+01, 2.21828075e+01, 6.61734807e+01, 3.389098005e+00,
    6.41362040e-01, 6.41362040e-01, 3.389098005e+00, 1.39646394e+00,
    4.98032205e-01, 2.53118738e+00, 1.05000000e+02, 9.78296453e+01,
    8.27479545e-01, 1.36822559e-01, 4.25841171e+00, -7.36416666e+01,
    -3.36082112e+01, -8.94055107e+01, -2.98736214e+00, -1.57921531e+00,
    -3.39766499e+00, -1.47912104e-04, -5.16373030e-04, -9.85522538e+01,
    -3.05907788e-02, -1.27877517e-01, -1.63514832e+00,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

std::vector<double> default_arm_params()
{
  return {
    1.94000000e-01, 4.29000000e-01, 1.14999999e-01, 3.32999998e-01,
    -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -4.29000003e-02,
    1.96649101e-02, 4.29000003e-02, 2.88077923e-03, 7.23516749e-03,
    9.16434754e-03, 2.16416476e-03, -1.19076924e-03, 8.07346553e-03,
    7.10109586e-01, 7.10109586e-01, 1.99576149e-06, -0.00000000e+00,
    -0.00000000e+00, -0.00000000e+00, 1.10178508e-01, 1.83331277e-01,
    1.04292121e-01, -3.32240937e-02, -8.30350362e-02, -3.83631263e-02,
    1.18956416e-01, 1.22363853e-01, 4.34411664e-03, -3.96112974e-04,
    -2.13904668e-02, -1.77228242e-03, 1.92510932e-02, 2.56548460e-02,
    7.17220917e-03, 1.48789886e-03, 4.53687373e-04, -1.09861913e-03,
    2.39569756e+00, 2.23596482e+00, 8.19671021e-01, 3.57249665e-01,
    0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0,
    0.19, 0.0, -0.12, 3.14159, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.04, 0.0, 0.0, 0.0};
}

void tile_into(const std::vector<double> & row, std::size_t count, std::vector<double> & out)
{
  out.clear();
  out.reserve(row.size() * count);
  for (std::size_t i = 0; i < count; ++i) {
    out.insert(out.end(), row.begin(), row.end());
  }
}

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
  resize_dynamics_buffers();
  observations_.assign(robot_count_ * kObservationDim, 0.0);
  next_observations_.assign(robot_count_ * kObservationDim, 0.0);
  actions_.assign(robot_count_ * kActionDim, 0.0);
  rewards_.assign(robot_count_, 0.0);
  dones_.assign(robot_count_, 0U);
  arm_lock_state_.assign(robot_count_ * kLockMaskDim, 0U);
  commands_held_ = hold_commands;
  tick_id_ = 0;
  applied_action_tick_id_ = 0;
  sim_time_ = 0.0;
  sim_period_ = 0.0;
  step_count_ = 0.0;
  reset_task();
}

bool BatchUvmsCore::reset(bool hold_commands, const std::vector<double> & observations)
{
  if (!observations.empty() && !observation_shape_matches(observations.size())) {
    return false;
  }
  reset(hold_commands);
  if (!observations.empty()) {
    observations_ = observations;
    next_observations_.assign(robot_count_ * kObservationDim, 0.0);
    reset_task();
  }
  ++tick_id_;
  return true;
}

bool BatchUvmsCore::set_observations(const std::vector<double> & observations)
{
  if (!observation_shape_matches(observations.size())) {
    return false;
  }
  observations_ = observations;
  next_observations_.assign(robot_count_ * kObservationDim, 0.0);
  compute_task();
  return true;
}

bool BatchUvmsCore::set_vehicle_params(const std::vector<double> & params)
{
  if (params.size() != kVehicleParamDim) {
    return false;
  }
  tile_into(params, robot_count_, vehicle_params_);
  return true;
}

bool BatchUvmsCore::set_arm_params(const std::vector<double> & params)
{
  if (params.size() != kArmParamDim) {
    return false;
  }
  tile_into(params, robot_count_, arm_params_);
  return true;
}

void BatchUvmsCore::set_arm_environment(
  double endeffector_mass,
  double endeffector_damping,
  double endeffector_stiffness,
  double baumgarte_alpha)
{
  ee_mass_.assign(robot_count_, endeffector_mass);
  ee_damping_.assign(robot_count_, endeffector_damping);
  ee_stiffness_.assign(robot_count_, endeffector_stiffness);
  baumgarte_alpha_.assign(robot_count_, baumgarte_alpha);
}

bool BatchUvmsCore::set_actions(const std::vector<double> & actions, std::uint64_t tick_id)
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
    actions_[action + i] = static_cast<double>(vehicle_command6[i]);
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i) {
    actions_[action + kVehicleActionDim + i] = static_cast<double>(arm_command[i]);
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
  arm_integration_policy::intervals(dt, baumgarte_alpha_.at(0));

  if (!commands_held_) {
    std::fill(dt_.begin(), dt_.end(), static_cast<double>(dt));
    pack_dynamics_actions();
    apply_arm_actuator_model();
    update_arm_lock_mask();
    uvms_cpu::UvmsStepHostPointers ptrs{
      observations_.data(), vehicle_wrench_.data(), arm_torque_.data(), vehicle_params_.data(),
      arm_params_.data(), dt_.data(), external_wrench_.data(), ee_mass_.data(),
      ee_damping_.data(), ee_stiffness_.data(), lock_mask_.data(), baumgarte_alpha_.data(),
      next_observations_.data()};
    uvms_cpu::step_uvms(ptrs, static_cast<int>(robot_count_));
    clamp_arm_state(next_observations_);
    observations_.swap(next_observations_);
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

std::vector<double> BatchUvmsCore::selected_state() const
{
  const auto begin = observations_.begin() + static_cast<std::ptrdiff_t>(selected_observation_offset());
  return std::vector<double>(begin, begin + static_cast<std::ptrdiff_t>(kObservationDim));
}

std::size_t BatchUvmsCore::clamp_robot_index(std::size_t robot_index) const
{
  return std::min(robot_index, robot_count_ - 1);
}

void BatchUvmsCore::resize_dynamics_buffers()
{
  const auto vehicle_defaults = default_vehicle_params();
  const auto arm_defaults = default_arm_params();

  tile_into(vehicle_defaults, robot_count_, vehicle_params_);
  tile_into(arm_defaults, robot_count_, arm_params_);

  vehicle_wrench_.assign(robot_count_ * kVehicleActionDim, 0.0);
  arm_torque_.assign(robot_count_ * kArmActionDim, 0.0);
  dt_.assign(robot_count_, 0.0);
  external_wrench_.assign(robot_count_ * kExternalWrenchDim, 0.0);
  ee_mass_.assign(robot_count_, 300.0);
  ee_damping_.assign(robot_count_, 400.0);
  ee_stiffness_.assign(robot_count_, 0.0);
  lock_mask_.assign(robot_count_ * kLockMaskDim, 0.0);
  arm_lock_state_.assign(robot_count_ * kLockMaskDim, 0U);
  baumgarte_alpha_.assign(robot_count_, 200.0);
}

void BatchUvmsCore::pack_dynamics_actions()
{
  for (std::size_t robot = 0; robot < robot_count_; ++robot) {
    const std::size_t action = action_offset(robot);
    std::copy_n(
      actions_.data() + static_cast<std::ptrdiff_t>(action),
      kVehicleActionDim,
      vehicle_wrench_.data() + static_cast<std::ptrdiff_t>(robot * kVehicleActionDim));
    std::copy_n(
      actions_.data() + static_cast<std::ptrdiff_t>(action + kVehicleActionDim),
      kArmActionDim,
      arm_torque_.data() + static_cast<std::ptrdiff_t>(robot * kArmActionDim));
  }
}

void BatchUvmsCore::apply_arm_actuator_model()
{
  for (std::size_t robot = 0; robot < robot_count_; ++robot) {
    const std::size_t torque_offset = robot * kArmActionDim;
    const std::size_t q_offset = arm_position_offset(robot);
    for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
      const double tau_cmd = arm_torque_[torque_offset + joint];
      const double static_current =
        tau_cmd >= 0.0 ? kArmForwardStaticCurrent[joint] : kArmBackwardStaticCurrent[joint];
      const double i_cmd = arm_actuator_model::torque_to_current(
        kArmMotorKt[joint], static_current, tau_cmd);

      double min_current = -kArmCurrentMax[joint];
      double max_current = kArmCurrentMax[joint];
      const double q = observations_[q_offset + joint];
      if (q < kArmPositionMin[joint]) {
        min_current = 0.0;
      } else if (q > kArmPositionMax[joint]) {
        max_current = 0.0;
      }

      const double i_safe = std::clamp(i_cmd, min_current, max_current);
      if (std::abs(i_safe) <= kTorqueCurrentZeroEpsilon) {
        arm_torque_[torque_offset + joint] = 0.0;
        continue;
      }
      const double safe_static_current =
        i_safe >= 0.0 ? kArmForwardStaticCurrent[joint] : kArmBackwardStaticCurrent[joint];
      arm_torque_[torque_offset + joint] =
        arm_actuator_model::current_to_torque(kArmMotorKt[joint], safe_static_current, i_safe);
    }
  }
}

void BatchUvmsCore::update_arm_lock_mask()
{
  if (arm_lock_state_.size() != robot_count_ * kLockMaskDim) {
    arm_lock_state_.assign(robot_count_ * kLockMaskDim, 0U);
  }

  for (std::size_t robot = 0; robot < robot_count_; ++robot) {
    const std::size_t torque_offset = robot * kArmActionDim;
    const std::size_t mask_offset = robot * kLockMaskDim;
    for (std::size_t joint = 0; joint < kLockMaskDim; ++joint) {
      const double effort = std::abs(arm_torque_[torque_offset + joint]);
      auto & locked = arm_lock_state_[mask_offset + joint];
      if (locked == 0U && effort < kJointLockOnDeadband) {
        locked = 1U;
      } else if (locked != 0U && effort > kJointLockOffDeadband) {
        locked = 0U;
      }
      lock_mask_[mask_offset + joint] = locked != 0U ? 1.0 : 0.0;
    }
  }
}

void BatchUvmsCore::clamp_arm_state(std::vector<double> & observations)
{
  for (std::size_t robot = 0; robot < robot_count_; ++robot) {
    const std::size_t q_offset = arm_position_offset(robot);
    const std::size_t qd_offset = arm_velocity_offset(robot);
    for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
      double & q = observations[q_offset + joint];
      double & qd = observations[qd_offset + joint];
      if (q < kArmPositionMin[joint]) {
        q = kArmPositionMin[joint];
        if (qd < 0.0) {
          qd = 0.0;
        }
      } else if (q > kArmPositionMax[joint]) {
        q = kArmPositionMax[joint];
        if (qd > 0.0) {
          qd = 0.0;
        }
      }
    }
  }
}

void BatchUvmsCore::compute_task()
{
  if (task_) {
    task_->compute(
      robot_count_, kObservationDim, kActionDim, observations_, actions_, rewards_, dones_);
    return;
  }

  std::fill(rewards_.begin(), rewards_.end(), 0.0);
  std::fill(dones_.begin(), dones_.end(), 0U);
}

void BatchUvmsCore::reset_task()
{
  if (task_) {
    task_->reset(robot_count_, kObservationDim, observations_, rewards_, dones_);
    return;
  }

  std::fill(rewards_.begin(), rewards_.end(), 0.0);
  std::fill(dones_.begin(), dones_.end(), 0U);
}

}  // namespace ros2_control_blue_reach_5
