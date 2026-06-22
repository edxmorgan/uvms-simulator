// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ros2_control_blue_reach_5
{
namespace
{
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

class PyBatchUvmsCore
{
public:
  explicit PyBatchUvmsCore(std::size_t robot_count)
  {
    core_.configure(robot_count);
  }

  void reset(bool hold_commands, py::object observations)
  {
    if (observations.is_none()) {
      core_.reset(hold_commands);
      return;
    }

    const auto array = py::array_t<float, py::array::c_style | py::array::forcecast>::ensure(observations);
    if (!array) {
      throw std::invalid_argument("observations must be convertible to a float32 array");
    }
    const std::vector<float> flat = flatten_float_array(array);
    if (!core_.reset(hold_commands, flat)) {
      throw std::invalid_argument("observations must have shape [robot_count, observation_dim] or flat matching size");
    }
  }

  void set_actions(py::array_t<float, py::array::c_style | py::array::forcecast> actions, std::uint64_t tick_id)
  {
    const std::vector<float> flat = flatten_float_array(actions);
    if (!core_.set_actions(flat, tick_id)) {
      throw std::invalid_argument("actions must have shape [robot_count, action_dim] or flat matching size");
    }
  }

  void set_observations(py::array_t<float, py::array::c_style | py::array::forcecast> observations)
  {
    const std::vector<float> flat = flatten_float_array(observations);
    if (!core_.set_observations(flat)) {
      throw std::invalid_argument("observations must have shape [robot_count, observation_dim] or flat matching size");
    }
  }

  void set_vehicle_params(py::array_t<float, py::array::c_style | py::array::forcecast> params)
  {
    const std::vector<float> flat = flatten_float_array(params);
    if (!core_.set_vehicle_params(flat)) {
      throw std::invalid_argument("vehicle params must have shape [33]");
    }
  }

  void set_arm_params(py::array_t<float, py::array::c_style | py::array::forcecast> params)
  {
    const std::vector<float> flat = flatten_float_array(params);
    if (!core_.set_arm_params(flat)) {
      throw std::invalid_argument("arm params must have shape [81]");
    }
  }

  void set_arm_environment(
    float endeffector_mass,
    float endeffector_damping,
    float endeffector_stiffness,
    float baumgarte_alpha)
  {
    core_.set_arm_environment(
      endeffector_mass, endeffector_damping, endeffector_stiffness, baumgarte_alpha);
  }

  void step(double dt)
  {
    core_.step(dt);
  }

  py::array_t<float> observations() const
  {
    return make_float_array(
      core_.observations(),
      {static_cast<py::ssize_t>(core_.robot_count()), static_cast<py::ssize_t>(BatchUvmsCore::kObservationDim)});
  }

  py::array_t<float> actions() const
  {
    return make_float_array(
      core_.actions(),
      {static_cast<py::ssize_t>(core_.robot_count()), static_cast<py::ssize_t>(BatchUvmsCore::kActionDim)});
  }

  py::array_t<float> rewards() const
  {
    return make_float_array(core_.rewards(), {static_cast<py::ssize_t>(core_.robot_count())});
  }

  py::array_t<std::uint8_t> dones() const
  {
    return make_uint8_array(core_.dones(), {static_cast<py::ssize_t>(core_.robot_count())});
  }

  py::array_t<float> selected_state() const
  {
    return make_float_array(core_.selected_state(), {static_cast<py::ssize_t>(BatchUvmsCore::kObservationDim)});
  }

  std::size_t robot_count() const { return core_.robot_count(); }
  std::size_t selected_robot_index() const { return core_.selected_robot_index(); }
  void set_selected_robot_index(std::size_t index) { core_.set_selected_robot_index(index); }
  bool commands_held() const { return core_.commands_held(); }
  std::uint64_t tick_id() const { return core_.tick_id(); }
  double sim_time() const { return core_.sim_time(); }
  double step_count() const { return core_.step_count(); }

private:
  BatchUvmsCore core_;
};
}  // namespace
}  // namespace ros2_control_blue_reach_5

PYBIND11_MODULE(_batch_uvms_core, module)
{
  using ros2_control_blue_reach_5::BatchUvmsCore;
  using ros2_control_blue_reach_5::PyBatchUvmsCore;

  module.doc() = "Python bindings for the batched UVMS simulator core.";
  module.attr("VEHICLE_STATE_DIM") = py::int_(BatchUvmsCore::kVehicleStateDim);
  module.attr("ARM_JOINT_COUNT") = py::int_(BatchUvmsCore::kArmJointCount);
  module.attr("ARM_STATE_DIM") = py::int_(BatchUvmsCore::kArmStateDim);
  module.attr("OBSERVATION_DIM") = py::int_(BatchUvmsCore::kObservationDim);
  module.attr("VEHICLE_ACTION_DIM") = py::int_(BatchUvmsCore::kVehicleActionDim);
  module.attr("ARM_ACTION_DIM") = py::int_(BatchUvmsCore::kArmActionDim);
  module.attr("ACTION_DIM") = py::int_(BatchUvmsCore::kActionDim);

  py::class_<PyBatchUvmsCore>(module, "BatchUvmsCore")
    .def(py::init<std::size_t>(), py::arg("robot_count"))
    .def("reset", &PyBatchUvmsCore::reset, py::arg("hold_commands") = false, py::arg("observations") = py::none())
    .def("set_vehicle_params", &PyBatchUvmsCore::set_vehicle_params, py::arg("params"))
    .def("set_arm_params", &PyBatchUvmsCore::set_arm_params, py::arg("params"))
    .def(
      "set_arm_environment",
      &PyBatchUvmsCore::set_arm_environment,
      py::arg("endeffector_mass"),
      py::arg("endeffector_damping"),
      py::arg("endeffector_stiffness"),
      py::arg("baumgarte_alpha"))
    .def("set_actions", &PyBatchUvmsCore::set_actions, py::arg("actions"), py::arg("tick_id"))
    .def("set_observations", &PyBatchUvmsCore::set_observations, py::arg("observations"))
    .def("step", &PyBatchUvmsCore::step, py::arg("dt"))
    .def("observations", &PyBatchUvmsCore::observations)
    .def("actions", &PyBatchUvmsCore::actions)
    .def("rewards", &PyBatchUvmsCore::rewards)
    .def("dones", &PyBatchUvmsCore::dones)
    .def("selected_state", &PyBatchUvmsCore::selected_state)
    .def("set_selected_robot_index", &PyBatchUvmsCore::set_selected_robot_index, py::arg("index"))
    .def_property_readonly("robot_count", &PyBatchUvmsCore::robot_count)
    .def_property_readonly("selected_robot_index", &PyBatchUvmsCore::selected_robot_index)
    .def_property_readonly("commands_held", &PyBatchUvmsCore::commands_held)
    .def_property_readonly("tick_id", &PyBatchUvmsCore::tick_id)
    .def_property_readonly("sim_time", &PyBatchUvmsCore::sim_time)
    .def_property_readonly("step_count", &PyBatchUvmsCore::step_count);
}
