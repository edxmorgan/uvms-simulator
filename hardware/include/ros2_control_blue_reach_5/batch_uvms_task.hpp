// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#ifndef ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_TASK_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_TASK_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ros2_control_blue_reach_5
{

class BatchUvmsTask
{
public:
  virtual ~BatchUvmsTask() = default;

  virtual void reset(
    std::size_t robot_count,
    std::size_t observation_dim,
    std::vector<float> & observations,
    std::vector<float> & rewards,
    std::vector<std::uint8_t> & dones) = 0;

  virtual void compute(
    std::size_t robot_count,
    std::size_t observation_dim,
    std::size_t action_dim,
    const std::vector<float> & observations,
    const std::vector<float> & actions,
    std::vector<float> & rewards,
    std::vector<std::uint8_t> & dones) = 0;
};

class BatchUvmsNullTask final : public BatchUvmsTask
{
public:
  void reset(
    std::size_t,
    std::size_t,
    std::vector<float> &,
    std::vector<float> & rewards,
    std::vector<std::uint8_t> & dones) override
  {
    std::fill(rewards.begin(), rewards.end(), 0.0F);
    std::fill(dones.begin(), dones.end(), 0U);
  }

  void compute(
    std::size_t,
    std::size_t,
    std::size_t,
    const std::vector<float> &,
    const std::vector<float> &,
    std::vector<float> & rewards,
    std::vector<std::uint8_t> & dones) override
  {
    std::fill(rewards.begin(), rewards.end(), 0.0F);
    std::fill(dones.begin(), dones.end(), 0U);
  }
};

}  // namespace ros2_control_blue_reach_5

#endif  // ROS2_CONTROL_BLUE_REACH_5__BATCH_UVMS_TASK_HPP_
