// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/batch_sim_uvms_system_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ros2_control_blue_reach_5
{
namespace
{
constexpr std::size_t kGpioCommandSelectedIndex = 0;
constexpr std::size_t kGpioCommandHold = 1;
constexpr std::size_t kGpioCommandResetEpoch = 2;
constexpr std::size_t kGpioCommandVehicleStart = 3;
constexpr std::size_t kGpioDiagnosticCount = 10;
constexpr std::size_t kSelectedVehicleStateStart = kGpioDiagnosticCount;
constexpr std::size_t kGpioStateCount = 28;
}  // namespace

BatchSimUvmsSystemHardware::~BatchSimUvmsSystemHardware()
{
  stop_ros_interfaces();
}

std::string BatchSimUvmsSystemHardware::string_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

std::size_t BatchSimUvmsSystemHardware::size_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  std::size_t fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return static_cast<std::size_t>(std::stoull(it->second));
  } catch (...) {
    return fallback;
  }
}

double BatchSimUvmsSystemHardware::double_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (...) {
    return fallback;
  }
}

hardware_interface::CallbackReturn BatchSimUvmsSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = get_hardware_info();
  robot_count_ = std::max<std::size_t>(1, size_param(info, "robot_count", robot_count_));
  arm_state_dim_ = 2 * arm_joint_count_;
  observation_dim_ = vehicle_state_dim_ + arm_state_dim_;
  action_dim_ = vehicle_action_dim_ + arm_action_dim_;
  selected_robot_index_ = std::min(
    size_param(info, "selected_robot_index", selected_robot_index_), robot_count_ - 1);
  state_update_frequency_ = std::max(
    1.0, double_param(info, "state_update_frequency", state_update_frequency_));
  publish_state_rate_ = std::max(
    0.0, double_param(info, "publish_state_rate", publish_state_rate_));
  world_frame_id_ = string_param(info, "world_frame_id", world_frame_id_);
  batch_observation_topic_ = string_param(
    info, "batch_observation_topic", batch_observation_topic_);
  batch_command_topic_ = string_param(info, "batch_command_topic", batch_command_topic_);
  batch_reset_service_ = string_param(info, "batch_reset_service", batch_reset_service_);

  if (info.gpios.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatchSimUvmsSystemHardware"),
      "batch UVMS hardware expects exactly one gpio block, got %zu", info.gpios.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info.gpios[0].command_interfaces.size() != 9 ||
      info.gpios[0].state_interfaces.size() != kGpioStateCount) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatchSimUvmsSystemHardware"),
      "batch UVMS gpio expects 9 command and %zu state interfaces, got %zu command and %zu state",
      kGpioStateCount,
      info.gpios[0].command_interfaces.size(),
      info.gpios[0].state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info.joints.size() != arm_joint_count_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatchSimUvmsSystemHardware"),
      "batch UVMS selected arm proxy expects %zu joints, got %zu", arm_joint_count_, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info.joints) {
    if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(
        rclcpp::get_logger("BatchSimUvmsSystemHardware"),
        "selected arm proxy joints expect 1 command and 3 state interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  core_.configure(robot_count_);
  gpio_state_interfaces_.assign(kGpioStateCount, 0.0);
  gpio_command_interfaces_.assign(info.gpios[0].command_interfaces.size(), 0.0);
  arm_position_state_interfaces_.assign(arm_joint_count_, 0.0);
  arm_velocity_state_interfaces_.assign(arm_joint_count_, 0.0);
  arm_effort_state_interfaces_.assign(arm_joint_count_, 0.0);
  arm_effort_command_interfaces_.assign(arm_joint_count_, 0.0);
  gpio_command_interfaces_[kGpioCommandSelectedIndex] = static_cast<double>(selected_robot_index_);
  gpio_command_interfaces_[kGpioCommandHold] = 1.0;

  RCLCPP_INFO(
    rclcpp::get_logger("BatchSimUvmsSystemHardware"),
    "configured mock whole-body batch UVMS hardware: robots=%zu obs_dim=%zu action_dim=%zu arm_joint_count=%zu",
    robot_count_, observation_dim_, action_dim_, arm_joint_count_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatchSimUvmsSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    node_ = std::make_shared<rclcpp::Node>(get_hardware_info().name + "_topics_interface");
    observation_pub_ = node_->create_publisher<msg::BatchObservation>(
      batch_observation_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort());
    command_sub_ = node_->create_subscription<msg::BatchCommand>(
      batch_command_topic_,
      rclcpp::QoS(rclcpp::KeepLast(2)).reliable(),
      [this](const msg::BatchCommand::SharedPtr msg) { command_callback(msg); });
    reset_service_ = node_->create_service<srv::ResetBatchSim>(
      batch_reset_service_,
      [this](
        const std::shared_ptr<srv::ResetBatchSim::Request> request,
        std::shared_ptr<srv::ResetBatchSim::Response> response) {
        {
          std::lock_guard<std::mutex> lock(buffer_mutex_);
          if (request->set_selected_robot) {
            selected_robot_index_ = std::min<std::size_t>(
              request->selected_robot_index, robot_count_ - 1);
            gpio_command_interfaces_[kGpioCommandSelectedIndex] =
              static_cast<double>(selected_robot_index_);
          }
          if (!core_.reset(request->hold_commands, request->states)) {
            response->success = false;
            response->message = "states must be empty or match robot_count * observation_dim";
            response->tick_id = core_.tick_id();
            return;
          }
          core_.set_selected_robot_index(selected_robot_index_);
          gpio_command_interfaces_[kGpioCommandHold] = request->hold_commands ? 1.0 : 0.0;
          update_selected_interfaces();
          response->tick_id = core_.tick_id();
        }
        response->success = true;
        response->message = "reset mock batch UVMS simulator";
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BatchSimUvmsSystemHardware"),
      "failed to configure ROS interfaces: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_buffers(true);
  RCLCPP_INFO(
    rclcpp::get_logger("BatchSimUvmsSystemHardware"),
    "mock batch UVMS ROS API ready: actions=%s observations=%s reset=%s",
    batch_command_topic_.c_str(), batch_observation_topic_.c_str(), batch_reset_service_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatchSimUvmsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_ros_interfaces();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatchSimUvmsSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  reset_buffers(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BatchSimUvmsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  core_.set_hold_commands(true);
  gpio_command_interfaces_[kGpioCommandHold] = 1.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BatchSimUvmsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const auto & gpio = get_hardware_info().gpios[0];
  state_interfaces.reserve(gpio.state_interfaces.size() + get_hardware_info().joints.size() * 3);
  for (std::size_t i = 0; i < gpio.state_interfaces.size(); ++i) {
    state_interfaces.emplace_back(gpio.name, gpio.state_interfaces[i].name, &gpio_state_interfaces_[i]);
  }
  for (std::size_t i = 0; i < get_hardware_info().joints.size(); ++i) {
    const auto & joint = get_hardware_info().joints[i];
    state_interfaces.emplace_back(joint.name, joint.state_interfaces[0].name, &arm_position_state_interfaces_[i]);
    state_interfaces.emplace_back(joint.name, joint.state_interfaces[1].name, &arm_velocity_state_interfaces_[i]);
    state_interfaces.emplace_back(joint.name, joint.state_interfaces[2].name, &arm_effort_state_interfaces_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BatchSimUvmsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  const auto & gpio = get_hardware_info().gpios[0];
  command_interfaces.reserve(gpio.command_interfaces.size() + get_hardware_info().joints.size());
  for (std::size_t i = 0; i < gpio.command_interfaces.size(); ++i) {
    command_interfaces.emplace_back(gpio.name, gpio.command_interfaces[i].name, &gpio_command_interfaces_[i]);
  }
  for (std::size_t i = 0; i < get_hardware_info().joints.size(); ++i) {
    const auto & joint = get_hardware_info().joints[i];
    command_interfaces.emplace_back(joint.name, joint.command_interfaces[0].name, &arm_effort_command_interfaces_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type BatchSimUvmsSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  (void)period;
  update_selected_interfaces();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BatchSimUvmsSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto t0 = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    selected_robot_index_ = std::min<std::size_t>(
      static_cast<std::size_t>(std::max(0.0, std::round(gpio_command_interfaces_[kGpioCommandSelectedIndex]))),
      robot_count_ - 1);
    core_.set_selected_robot_index(selected_robot_index_);
    core_.set_hold_commands(gpio_command_interfaces_[kGpioCommandHold] >= 0.5);

    if (gpio_command_interfaces_[kGpioCommandResetEpoch] != last_reset_epoch_command_) {
      last_reset_epoch_command_ = gpio_command_interfaces_[kGpioCommandResetEpoch];
      core_.reset(gpio_command_interfaces_[kGpioCommandHold] >= 0.5);
    }

    bool proxy_command_nonzero = false;
    for (std::size_t i = 0; i < 6; ++i) {
      proxy_command_nonzero = proxy_command_nonzero ||
        std::abs(gpio_command_interfaces_[kGpioCommandVehicleStart + i]) > 1e-12;
    }
    for (const double command : arm_effort_command_interfaces_) {
      proxy_command_nonzero = proxy_command_nonzero || std::abs(command) > 1e-12;
    }

    if (proxy_command_nonzero) {
      core_.set_selected_action(
        selected_robot_index_,
        &gpio_command_interfaces_[kGpioCommandVehicleStart],
        arm_effort_command_interfaces_);
      gpio_command_interfaces_[kGpioCommandHold] = 0.0;
    }

    core_.step(std::max(0.0, period.seconds()));
    update_selected_interfaces();
  }
  const auto t1 = std::chrono::steady_clock::now();
  gpu_step_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
  publish_observation_if_due(time);
  return hardware_interface::return_type::OK;
}

void BatchSimUvmsSystemHardware::stop_ros_interfaces() noexcept
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  reset_service_.reset();
  command_sub_.reset();
  observation_pub_.reset();
  executor_.reset();
  node_.reset();
}

void BatchSimUvmsSystemHardware::reset_buffers(bool hold_commands)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  core_.reset(hold_commands);
  core_.set_selected_robot_index(selected_robot_index_);
  gpio_command_interfaces_[kGpioCommandHold] = hold_commands ? 1.0 : 0.0;
  gpu_step_ms_ = 0.0;
  host_to_device_ms_ = 0.0;
  device_to_host_ms_ = 0.0;
  publish_ms_ = 0.0;
  dropped_command_count_ = 0.0;
  last_publish_sim_time_ = -1.0;
  update_selected_interfaces();
}

void BatchSimUvmsSystemHardware::update_selected_interfaces()
{
  const auto & observations = core_.observations();
  const auto & actions = core_.actions();
  selected_robot_index_ = core_.selected_robot_index();

  gpio_state_interfaces_[0] = static_cast<double>(core_.robot_count());
  gpio_state_interfaces_[1] = core_.sim_time();
  gpio_state_interfaces_[2] = core_.sim_period();
  gpio_state_interfaces_[3] = core_.step_count();
  gpio_state_interfaces_[4] = gpu_step_ms_;
  gpio_state_interfaces_[5] = host_to_device_ms_;
  gpio_state_interfaces_[6] = device_to_host_ms_;
  gpio_state_interfaces_[7] = publish_ms_;
  gpio_state_interfaces_[8] = dropped_command_count_;
  gpio_state_interfaces_[9] = static_cast<double>(selected_robot_index_);

  const std::size_t obs_offset = core_.selected_observation_offset();
  for (std::size_t i = 0; i < BatchUvmsCore::kVehicleStateDim; ++i) {
    gpio_state_interfaces_[kSelectedVehicleStateStart + i] = observations[obs_offset + i];
  }
  const std::size_t action_offset = core_.selected_action_offset();
  for (std::size_t i = 0; i < 6; ++i) {
    gpio_state_interfaces_[kSelectedVehicleStateStart + BatchUvmsCore::kVehicleStateDim + i] =
      i < vehicle_action_dim_ ? actions[action_offset + i] : 0.0F;
  }

  const std::size_t q_offset = core_.arm_position_offset(selected_robot_index_);
  const std::size_t qd_offset = core_.arm_velocity_offset(selected_robot_index_);
  for (std::size_t i = 0; i < arm_joint_count_; ++i) {
    arm_position_state_interfaces_[i] = observations[q_offset + i];
    arm_velocity_state_interfaces_[i] = observations[qd_offset + i];
    arm_effort_state_interfaces_[i] = actions[action_offset + vehicle_action_dim_ + i];
  }
}

void BatchSimUvmsSystemHardware::command_callback(const msg::BatchCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!core_.action_shape_matches(
      msg->robot_count, msg->action_dim, msg->vehicle_action_dim, msg->arm_action_dim,
      msg->arm_joint_count, msg->actions.size())) {
    ++dropped_command_count_;
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("BatchSimUvmsSystemHardware"),
      *node_->get_clock(),
      2000,
      "dropping batch command with incompatible shape");
    return;
  }

  core_.set_actions(msg->actions, msg->tick_id);
  gpio_command_interfaces_[kGpioCommandHold] = 0.0;
}

void BatchSimUvmsSystemHardware::publish_observation_if_due(const rclcpp::Time & time)
{
  if (!observation_pub_ || publish_state_rate_ <= 0.0) {
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  msg::BatchObservation msg;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    const double min_period = 1.0 / publish_state_rate_;
    if (last_publish_sim_time_ >= 0.0 &&
        (core_.sim_time() - last_publish_sim_time_) < min_period) {
      return;
    }

    msg.header.stamp = time;
    msg.header.frame_id = world_frame_id_;
    msg.tick_id = core_.tick_id();
    msg.robot_count = static_cast<std::uint32_t>(core_.robot_count());
    msg.observation_dim = static_cast<std::uint32_t>(observation_dim_);
    msg.vehicle_state_dim = static_cast<std::uint32_t>(vehicle_state_dim_);
    msg.arm_state_dim = static_cast<std::uint32_t>(arm_state_dim_);
    msg.arm_joint_count = static_cast<std::uint32_t>(arm_joint_count_);
    msg.action_dim = static_cast<std::uint32_t>(action_dim_);
    msg.vehicle_action_dim = static_cast<std::uint32_t>(vehicle_action_dim_);
    msg.arm_action_dim = static_cast<std::uint32_t>(arm_action_dim_);
    msg.observations = core_.observations();
    msg.rewards = core_.rewards();
    msg.dones = core_.dones();
    msg.selected_robot_index = static_cast<std::uint32_t>(selected_robot_index_);
    msg.selected_state = core_.selected_state();
    msg.gpu_step_ms = static_cast<float>(gpu_step_ms_);
    msg.host_to_device_ms = static_cast<float>(host_to_device_ms_);
    msg.device_to_host_ms = static_cast<float>(device_to_host_ms_);
    msg.publish_ms = static_cast<float>(publish_ms_);
    last_publish_sim_time_ = core_.sim_time();
  }
  observation_pub_->publish(msg);
  const auto t1 = std::chrono::steady_clock::now();
  publish_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

}  // namespace ros2_control_blue_reach_5

PLUGINLIB_EXPORT_CLASS(
  ros2_control_blue_reach_5::BatchSimUvmsSystemHardware,
  hardware_interface::SystemInterface)
