// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"
#include "ros2_control_blue_reach_5/gpu_dynamics/uvms_gpu_dynamics.h"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ros2_control_blue_reach_5
{
namespace
{
constexpr std::size_t kVehicleParamDim = 33;
constexpr std::size_t kArmParamDim = 81;
constexpr std::size_t kExternalWrenchDim = 6;
constexpr std::size_t kLockMaskDim = 4;

void check_cuda(cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

std::vector<float> flatten_float_array(const py::array_t<float, py::array::c_style | py::array::forcecast> & array)
{
  const py::buffer_info info = array.request();
  const auto * data = static_cast<const float *>(info.ptr);
  return std::vector<float>(data, data + info.size);
}

py::array_t<float> make_float_array(const std::vector<float> & values, const std::vector<py::ssize_t> & shape)
{
  py::array_t<float> array(shape);
  py::buffer_info info = array.request();
  auto * data = static_cast<float *>(info.ptr);
  std::copy(values.begin(), values.end(), data);
  return array;
}

py::array_t<std::uint8_t> make_uint8_array(
  const std::vector<std::uint8_t> & values,
  const std::vector<py::ssize_t> & shape)
{
  py::array_t<std::uint8_t> array(shape);
  py::buffer_info info = array.request();
  auto * data = static_cast<std::uint8_t *>(info.ptr);
  std::copy(values.begin(), values.end(), data);
  return array;
}

std::vector<float> default_vehicle_params()
{
  return {
    3.72028553e+01F, 2.21828075e+01F, 6.61734807e+01F, 3.38909801e+00F,
    6.41362046e-01F, 6.41362034e-01F, 3.38909800e+00F, 1.39646394e+00F,
    4.98032205e-01F, 2.53118738e+00F, 1.05000000e+02F, 9.78296453e+01F,
    8.27479545e-01F, 1.36822559e-01F, 4.25841171e+00F, -7.36416666e+01F,
    -3.36082112e+01F, -8.94055107e+01F, -2.98736214e+00F, -1.57921531e+00F,
    -3.39766499e+00F, -1.47912104e-04F, -5.16373030e-04F, -9.85522538e+01F,
    -3.05907788e-02F, -1.27877517e-01F, -1.63514832e+00F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
}

std::vector<float> default_arm_params()
{
  return {
    1.94000000e-01F, 4.29000000e-01F, 1.14999999e-01F, 3.32999998e-01F,
    -0.00000000e+00F, -0.00000000e+00F, -0.00000000e+00F, -4.29000003e-02F,
    1.96649101e-02F, 4.29000003e-02F, 2.88077923e-03F, 7.23516749e-03F,
    9.16434754e-03F, 2.16416476e-03F, -1.19076924e-03F, 8.07346553e-03F,
    7.10109586e-01F, 7.10109586e-01F, 1.99576149e-06F, -0.00000000e+00F,
    -0.00000000e+00F, -0.00000000e+00F, 1.10178508e-01F, 1.83331277e-01F,
    1.04292121e-01F, -3.32240937e-02F, -8.30350362e-02F, -3.83631263e-02F,
    1.18956416e-01F, 1.22363853e-01F, 4.34411664e-03F, -3.96112974e-04F,
    -2.13904668e-02F, -1.77228242e-03F, 1.92510932e-02F, 2.56548460e-02F,
    7.17220917e-03F, 1.48789886e-03F, 4.53687373e-04F, -1.09861913e-03F,
    2.39569756e+00F, 2.23596482e+00F, 8.19671021e-01F, 3.57249665e-01F,
    0.0F, 0.0F, 0.0F, 0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F,
    0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.0F,
    0.19F, 0.0F, -0.12F, 3.14159F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.04F, 0.0F, 0.0F, 0.0F};
}

void tile_into(const std::vector<float> & row, std::size_t count, std::vector<float> & out)
{
  out.clear();
  out.reserve(row.size() * count);
  for (std::size_t i = 0; i < count; ++i) {
    out.insert(out.end(), row.begin(), row.end());
  }
}

void split_observations(
  const std::vector<float> & observations,
  std::size_t robot_count,
  std::vector<float> & vehicle,
  std::vector<float> & arm)
{
  vehicle.resize(robot_count * BatchUvmsCore::kVehicleStateDim);
  arm.resize(robot_count * BatchUvmsCore::kArmStateDim);
  for (std::size_t robot = 0; robot < robot_count; ++robot) {
    const auto obs_offset = robot * BatchUvmsCore::kObservationDim;
    std::copy_n(
      observations.data() + static_cast<std::ptrdiff_t>(obs_offset),
      BatchUvmsCore::kVehicleStateDim,
      vehicle.data() + static_cast<std::ptrdiff_t>(robot * BatchUvmsCore::kVehicleStateDim));
    std::copy_n(
      observations.data() + static_cast<std::ptrdiff_t>(obs_offset + BatchUvmsCore::kVehicleStateDim),
      BatchUvmsCore::kArmStateDim,
      arm.data() + static_cast<std::ptrdiff_t>(robot * BatchUvmsCore::kArmStateDim));
  }
}

void join_observations(
  const std::vector<float> & vehicle,
  const std::vector<float> & arm,
  std::size_t robot_count,
  std::vector<float> & observations)
{
  observations.assign(robot_count * BatchUvmsCore::kObservationDim, 0.0F);
  for (std::size_t robot = 0; robot < robot_count; ++robot) {
    const auto obs_offset = robot * BatchUvmsCore::kObservationDim;
    std::copy_n(
      vehicle.data() + static_cast<std::ptrdiff_t>(robot * BatchUvmsCore::kVehicleStateDim),
      BatchUvmsCore::kVehicleStateDim,
      observations.data() + static_cast<std::ptrdiff_t>(obs_offset));
    std::copy_n(
      arm.data() + static_cast<std::ptrdiff_t>(robot * BatchUvmsCore::kArmStateDim),
      BatchUvmsCore::kArmStateDim,
      observations.data() + static_cast<std::ptrdiff_t>(obs_offset + BatchUvmsCore::kVehicleStateDim));
  }
}

class DeviceFloatBuffer
{
public:
  DeviceFloatBuffer() = default;
  DeviceFloatBuffer(const DeviceFloatBuffer &) = delete;
  DeviceFloatBuffer & operator=(const DeviceFloatBuffer &) = delete;

  DeviceFloatBuffer(DeviceFloatBuffer && other) noexcept
  : data_(std::exchange(other.data_, nullptr)), count_(std::exchange(other.count_, 0))
  {
  }

  DeviceFloatBuffer & operator=(DeviceFloatBuffer && other) noexcept
  {
    if (this != &other) {
      release();
      data_ = std::exchange(other.data_, nullptr);
      count_ = std::exchange(other.count_, 0);
    }
    return *this;
  }

  ~DeviceFloatBuffer()
  {
    release();
  }

  void resize(std::size_t count)
  {
    if (count == count_) {
      return;
    }
    release();
    count_ = count;
    if (count_ > 0) {
      check_cuda(cudaMalloc(reinterpret_cast<void **>(&data_), count_ * sizeof(float)), "cudaMalloc");
    }
  }

  void fill(float value)
  {
    std::vector<float> host(count_, value);
    copy_from_host(host);
  }

  void copy_from_host(const std::vector<float> & host)
  {
    if (host.size() != count_) {
      throw std::invalid_argument("host/device buffer size mismatch");
    }
    if (count_ > 0) {
      check_cuda(
        cudaMemcpy(data_, host.data(), count_ * sizeof(float), cudaMemcpyHostToDevice),
        "cudaMemcpy host to device");
    }
  }

  void copy_to_host(std::vector<float> & host) const
  {
    host.resize(count_);
    if (count_ > 0) {
      check_cuda(
        cudaMemcpy(host.data(), data_, count_ * sizeof(float), cudaMemcpyDeviceToHost),
        "cudaMemcpy device to host");
    }
  }

  float * data() { return data_; }
  const float * data() const { return data_; }

private:
  void release() noexcept
  {
    if (data_ != nullptr) {
      cudaFree(data_);
      data_ = nullptr;
    }
    count_ = 0;
  }

  float * data_{nullptr};
  std::size_t count_{0};
};

class PyBatchUvmsGpuCore
{
public:
  explicit PyBatchUvmsGpuCore(std::size_t robot_count)
  {
    configure(robot_count);
  }

  void reset(bool hold_commands, py::object observations)
  {
    const bool has_observations = !observations.is_none();
    observations_host_.assign(robot_count_ * BatchUvmsCore::kObservationDim, 0.0F);
    actions_host_.assign(robot_count_ * BatchUvmsCore::kActionDim, 0.0F);
    vehicle_wrench_host_.assign(robot_count_ * BatchUvmsCore::kVehicleActionDim, 0.0F);
    rewards_.assign(robot_count_, 0.0F);
    dones_.assign(robot_count_, 0U);

    if (has_observations) {
      const auto array = py::array_t<float, py::array::c_style | py::array::forcecast>::ensure(observations);
      if (!array) {
        throw std::invalid_argument("observations must be convertible to a float32 array");
      }
      observations_host_ = flatten_float_array(array);
      if (observations_host_.size() != robot_count_ * BatchUvmsCore::kObservationDim) {
        throw std::invalid_argument("observations must have shape [robot_count, observation_dim] or flat matching size");
      }
    }

    std::vector<float> vehicle_host;
    std::vector<float> arm_host;
    split_observations(observations_host_, robot_count_, vehicle_host, arm_host);
    vehicle_state_.copy_from_host(vehicle_host);
    arm_state_.copy_from_host(arm_host);
    vehicle_next_.fill(0.0F);
    arm_next_.fill(0.0F);
    actions_device_.copy_from_host(actions_host_);
    vehicle_wrench_.copy_from_host(vehicle_wrench_host_);
    arm_torque_.fill(0.0F);
    lock_mask_.fill(0.0F);

    commands_held_ = hold_commands;
    tick_id_ = 0;
    applied_action_tick_id_ = 0;
    sim_time_ = 0.0;
    step_count_ = 0.0;
    observations_dirty_ = false;
    if (has_observations) {
      ++tick_id_;
    }
  }

  void set_actions(py::array_t<float, py::array::c_style | py::array::forcecast> actions, std::uint64_t tick_id)
  {
    actions_host_ = flatten_float_array(actions);
    if (actions_host_.size() != robot_count_ * BatchUvmsCore::kActionDim) {
      throw std::invalid_argument("actions must have shape [robot_count, action_dim] or flat matching size");
    }

    actions_device_.copy_from_host(actions_host_);
    split_device_actions();
    actions_dirty_ = false;
    applied_action_tick_id_ = tick_id;
    commands_held_ = false;
  }

  void set_actions_from_device(std::uintptr_t actions_ptr, std::uint64_t tick_id)
  {
    if (actions_ptr == 0U) {
      throw std::invalid_argument("actions_ptr must be a nonzero CUDA device pointer");
    }
    const auto * source = reinterpret_cast<const float *>(actions_ptr);
    check_cuda(
      cudaMemcpy(
        actions_device_.data(), source, robot_count_ * BatchUvmsCore::kActionDim * sizeof(float),
        cudaMemcpyDeviceToDevice),
      "cudaMemcpy device actions");
    split_device_actions();
    actions_dirty_ = true;
    applied_action_tick_id_ = tick_id;
    commands_held_ = false;
  }

  void set_vehicle_params(py::array_t<float, py::array::c_style | py::array::forcecast> params)
  {
    const std::vector<float> flat = flatten_float_array(params);
    if (flat.size() != kVehicleParamDim) {
      throw std::invalid_argument("vehicle params must have shape [33]");
    }
    std::vector<float> tiled;
    tile_into(flat, robot_count_, tiled);
    vehicle_params_.copy_from_host(tiled);
  }

  void set_arm_params(py::array_t<float, py::array::c_style | py::array::forcecast> params)
  {
    const std::vector<float> flat = flatten_float_array(params);
    if (flat.size() != kArmParamDim) {
      throw std::invalid_argument("arm params must have shape [81]");
    }
    std::vector<float> tiled;
    tile_into(flat, robot_count_, tiled);
    arm_params_.copy_from_host(tiled);
  }

  void set_arm_environment(
    float endeffector_mass,
    float endeffector_damping,
    float endeffector_stiffness,
    float baumgarte_alpha)
  {
    ee_mass_.fill(endeffector_mass);
    ee_damping_.fill(endeffector_damping);
    ee_stiffness_.fill(endeffector_stiffness);
    baumgarte_alpha_.fill(baumgarte_alpha);
  }

  void step(double dt)
  {
    if (dt <= 0.0) {
      return;
    }

    if (!commands_held_) {
      dt_host_.assign(robot_count_, static_cast<float>(dt));
      dt_.copy_from_host(dt_host_);

      uvms_gpu::VehicleStepDevicePointers vehicle_ptrs{
        vehicle_state_.data(), vehicle_wrench_.data(), vehicle_params_.data(), dt_.data(),
        external_wrench_.data(), vehicle_next_.data()};
      uvms_gpu::ArmStepDevicePointers arm_ptrs{
        arm_state_.data(), arm_torque_.data(), dt_.data(), arm_params_.data(), ee_mass_.data(),
        ee_damping_.data(), ee_stiffness_.data(), lock_mask_.data(), baumgarte_alpha_.data(),
        arm_next_.data()};

      {
        py::gil_scoped_release release;
        uvms_gpu::launch_vehicle_step(vehicle_ptrs, static_cast<int>(robot_count_), 256, 0, false);
        uvms_gpu::launch_prepare_arm_step(
          arm_state_.data(), arm_torque_.data(), lock_mask_.data(), static_cast<int>(robot_count_),
          256, 0, false);
        uvms_gpu::launch_arm_step(arm_ptrs, static_cast<int>(robot_count_), 256, 0, false);
        uvms_gpu::launch_clamp_arm_state(arm_next_.data(), static_cast<int>(robot_count_), 256, 0, false);
        uvms_gpu::device_synchronize();
      }

      std::swap(vehicle_state_, vehicle_next_);
      std::swap(arm_state_, arm_next_);
      observations_dirty_ = true;
    }

    sim_time_ += dt;
    step_count_ += 1.0;
    tick_id_ = std::max(tick_id_ + 1, applied_action_tick_id_);
    std::fill(rewards_.begin(), rewards_.end(), 0.0F);
    std::fill(dones_.begin(), dones_.end(), 0U);
  }

  py::array_t<float> observations()
  {
    sync_observations_to_host();
    return make_float_array(
      observations_host_,
      {static_cast<py::ssize_t>(robot_count_), static_cast<py::ssize_t>(BatchUvmsCore::kObservationDim)});
  }

  py::array_t<float> actions() const
  {
    if (actions_dirty_) {
      actions_device_.copy_to_host(actions_host_);
      actions_dirty_ = false;
    }
    return make_float_array(
      actions_host_,
      {static_cast<py::ssize_t>(robot_count_), static_cast<py::ssize_t>(BatchUvmsCore::kActionDim)});
  }

  py::array_t<float> rewards() const
  {
    return make_float_array(rewards_, {static_cast<py::ssize_t>(robot_count_)});
  }

  py::array_t<std::uint8_t> dones() const
  {
    return make_uint8_array(dones_, {static_cast<py::ssize_t>(robot_count_)});
  }

  py::array_t<float> selected_state()
  {
    sync_observations_to_host();
    const auto begin = observations_host_.begin() +
      static_cast<std::ptrdiff_t>(selected_robot_index_ * BatchUvmsCore::kObservationDim);
    std::vector<float> state(begin, begin + static_cast<std::ptrdiff_t>(BatchUvmsCore::kObservationDim));
    return make_float_array(state, {static_cast<py::ssize_t>(BatchUvmsCore::kObservationDim)});
  }

  void set_selected_robot_index(std::size_t index)
  {
    selected_robot_index_ = std::min(index, robot_count_ - 1);
  }

  std::size_t robot_count() const { return robot_count_; }
  std::size_t selected_robot_index() const { return selected_robot_index_; }
  bool commands_held() const { return commands_held_; }
  std::uint64_t tick_id() const { return tick_id_; }
  double sim_time() const { return sim_time_; }
  double step_count() const { return step_count_; }
  std::uintptr_t vehicle_state_ptr() const { return reinterpret_cast<std::uintptr_t>(vehicle_state_.data()); }
  std::uintptr_t arm_state_ptr() const { return reinterpret_cast<std::uintptr_t>(arm_state_.data()); }
  std::uintptr_t actions_ptr() const { return reinterpret_cast<std::uintptr_t>(actions_device_.data()); }

private:
  void configure(std::size_t robot_count)
  {
    robot_count_ = std::max<std::size_t>(1, robot_count);
    selected_robot_index_ = std::min(selected_robot_index_, robot_count_ - 1);

    vehicle_state_.resize(robot_count_ * BatchUvmsCore::kVehicleStateDim);
    vehicle_next_.resize(robot_count_ * BatchUvmsCore::kVehicleStateDim);
    arm_state_.resize(robot_count_ * BatchUvmsCore::kArmStateDim);
    arm_next_.resize(robot_count_ * BatchUvmsCore::kArmStateDim);
    actions_device_.resize(robot_count_ * BatchUvmsCore::kActionDim);
    vehicle_wrench_.resize(robot_count_ * BatchUvmsCore::kVehicleActionDim);
    arm_torque_.resize(robot_count_ * BatchUvmsCore::kArmActionDim);
    dt_.resize(robot_count_);
    external_wrench_.resize(robot_count_ * kExternalWrenchDim);
    ee_mass_.resize(robot_count_);
    ee_damping_.resize(robot_count_);
    ee_stiffness_.resize(robot_count_);
    lock_mask_.resize(robot_count_ * kLockMaskDim);
    baumgarte_alpha_.resize(robot_count_);

    std::vector<float> vehicle_params_host;
    std::vector<float> arm_params_host;
    tile_into(default_vehicle_params(), robot_count_, vehicle_params_host);
    tile_into(default_arm_params(), robot_count_, arm_params_host);
    vehicle_params_.resize(vehicle_params_host.size());
    arm_params_.resize(arm_params_host.size());
    vehicle_params_.copy_from_host(vehicle_params_host);
    arm_params_.copy_from_host(arm_params_host);

    external_wrench_.fill(0.0F);
    ee_mass_.fill(300.0F);
    ee_damping_.fill(400.0F);
    ee_stiffness_.fill(0.0F);
    lock_mask_.fill(0.0F);
    baumgarte_alpha_.fill(200.0F);
    reset(true, py::none());
  }

  void sync_observations_to_host()
  {
    if (!observations_dirty_) {
      return;
    }
    std::vector<float> vehicle_host;
    std::vector<float> arm_host;
    vehicle_state_.copy_to_host(vehicle_host);
    arm_state_.copy_to_host(arm_host);
    join_observations(vehicle_host, arm_host, robot_count_, observations_host_);
    observations_dirty_ = false;
  }

  void split_device_actions()
  {
    check_cuda(
      cudaMemcpy2D(
        vehicle_wrench_.data(), BatchUvmsCore::kVehicleActionDim * sizeof(float),
        actions_device_.data(), BatchUvmsCore::kActionDim * sizeof(float),
        BatchUvmsCore::kVehicleActionDim * sizeof(float), robot_count_, cudaMemcpyDeviceToDevice),
      "cudaMemcpy2D vehicle actions");
    check_cuda(
      cudaMemcpy2D(
        arm_torque_.data(), BatchUvmsCore::kArmActionDim * sizeof(float),
        actions_device_.data() + BatchUvmsCore::kVehicleActionDim,
        BatchUvmsCore::kActionDim * sizeof(float),
        BatchUvmsCore::kArmActionDim * sizeof(float), robot_count_, cudaMemcpyDeviceToDevice),
      "cudaMemcpy2D arm actions");
  }

  std::size_t robot_count_{1};
  std::size_t selected_robot_index_{0};
  std::vector<float> observations_host_;
  mutable std::vector<float> actions_host_;
  std::vector<float> vehicle_wrench_host_;
  std::vector<float> dt_host_;
  std::vector<float> rewards_;
  std::vector<std::uint8_t> dones_;
  DeviceFloatBuffer vehicle_state_;
  DeviceFloatBuffer vehicle_next_;
  DeviceFloatBuffer arm_state_;
  DeviceFloatBuffer arm_next_;
  DeviceFloatBuffer actions_device_;
  DeviceFloatBuffer vehicle_wrench_;
  DeviceFloatBuffer arm_torque_;
  DeviceFloatBuffer vehicle_params_;
  DeviceFloatBuffer arm_params_;
  DeviceFloatBuffer dt_;
  DeviceFloatBuffer external_wrench_;
  DeviceFloatBuffer ee_mass_;
  DeviceFloatBuffer ee_damping_;
  DeviceFloatBuffer ee_stiffness_;
  DeviceFloatBuffer lock_mask_;
  DeviceFloatBuffer baumgarte_alpha_;
  bool commands_held_{true};
  bool observations_dirty_{false};
  mutable bool actions_dirty_{false};
  std::uint64_t tick_id_{0};
  std::uint64_t applied_action_tick_id_{0};
  double sim_time_{0.0};
  double step_count_{0.0};
};
}  // namespace
}  // namespace ros2_control_blue_reach_5

PYBIND11_MODULE(_batch_uvms_gpu_core, module)
{
  using ros2_control_blue_reach_5::BatchUvmsCore;
  using ros2_control_blue_reach_5::PyBatchUvmsGpuCore;

  module.doc() = "Python bindings for the CUDA batched UVMS simulator core.";
  module.attr("VEHICLE_STATE_DIM") = py::int_(BatchUvmsCore::kVehicleStateDim);
  module.attr("ARM_JOINT_COUNT") = py::int_(BatchUvmsCore::kArmJointCount);
  module.attr("ARM_STATE_DIM") = py::int_(BatchUvmsCore::kArmStateDim);
  module.attr("OBSERVATION_DIM") = py::int_(BatchUvmsCore::kObservationDim);
  module.attr("VEHICLE_ACTION_DIM") = py::int_(BatchUvmsCore::kVehicleActionDim);
  module.attr("ARM_ACTION_DIM") = py::int_(BatchUvmsCore::kArmActionDim);
  module.attr("ACTION_DIM") = py::int_(BatchUvmsCore::kActionDim);

  py::class_<PyBatchUvmsGpuCore>(module, "BatchUvmsCore")
    .def(py::init<std::size_t>(), py::arg("robot_count"))
    .def("reset", &PyBatchUvmsGpuCore::reset, py::arg("hold_commands") = false, py::arg("observations") = py::none())
    .def("set_vehicle_params", &PyBatchUvmsGpuCore::set_vehicle_params, py::arg("params"))
    .def("set_arm_params", &PyBatchUvmsGpuCore::set_arm_params, py::arg("params"))
    .def(
      "set_arm_environment",
      &PyBatchUvmsGpuCore::set_arm_environment,
      py::arg("endeffector_mass"),
      py::arg("endeffector_damping"),
      py::arg("endeffector_stiffness"),
      py::arg("baumgarte_alpha"))
    .def("set_actions", &PyBatchUvmsGpuCore::set_actions, py::arg("actions"), py::arg("tick_id"))
    .def("set_actions_from_device", &PyBatchUvmsGpuCore::set_actions_from_device, py::arg("actions_ptr"), py::arg("tick_id"))
    .def("step", &PyBatchUvmsGpuCore::step, py::arg("dt"))
    .def("observations", &PyBatchUvmsGpuCore::observations)
    .def("actions", &PyBatchUvmsGpuCore::actions)
    .def("rewards", &PyBatchUvmsGpuCore::rewards)
    .def("dones", &PyBatchUvmsGpuCore::dones)
    .def("selected_state", &PyBatchUvmsGpuCore::selected_state)
    .def("set_selected_robot_index", &PyBatchUvmsGpuCore::set_selected_robot_index, py::arg("index"))
    .def_property_readonly("robot_count", &PyBatchUvmsGpuCore::robot_count)
    .def_property_readonly("selected_robot_index", &PyBatchUvmsGpuCore::selected_robot_index)
    .def_property_readonly("commands_held", &PyBatchUvmsGpuCore::commands_held)
    .def_property_readonly("tick_id", &PyBatchUvmsGpuCore::tick_id)
    .def_property_readonly("sim_time", &PyBatchUvmsGpuCore::sim_time)
    .def_property_readonly("step_count", &PyBatchUvmsGpuCore::step_count)
    .def_property_readonly("vehicle_state_ptr", &PyBatchUvmsGpuCore::vehicle_state_ptr)
    .def_property_readonly("arm_state_ptr", &PyBatchUvmsGpuCore::arm_state_ptr)
    .def_property_readonly("actions_ptr", &PyBatchUvmsGpuCore::actions_ptr);
}
