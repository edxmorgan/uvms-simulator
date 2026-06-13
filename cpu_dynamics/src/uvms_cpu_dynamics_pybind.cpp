// Author: edward morgan

#include "ros2_control_blue_reach_5/cpu_dynamics/uvms_cpu_dynamics.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace {
constexpr py::ssize_t kVehicleStateDim = 12;
constexpr py::ssize_t kVehicleWrenchDim = 6;
constexpr py::ssize_t kVehicleParamDim = 33;
constexpr py::ssize_t kArmStateDim = 10;
constexpr py::ssize_t kArmTorqueDim = 5;
constexpr py::ssize_t kArmParamDim = 81;
constexpr py::ssize_t kLockMaskDim = 4;
constexpr py::ssize_t kUvmsStateDim = kVehicleStateDim + kArmStateDim;

using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;

FloatArray as_array(py::handle object, const char* name) {
    FloatArray array = FloatArray::ensure(object);
    if (!array) {
        throw std::invalid_argument(std::string(name) + " must be convertible to float32 ndarray");
    }
    return array;
}

void require_2d(const FloatArray& array, const char* name, py::ssize_t cols) {
    const py::buffer_info info = array.request();
    if (info.ndim != 2 || info.shape[1] != cols) {
        throw std::invalid_argument(
            std::string(name) + " must have shape [batch_size, " + std::to_string(cols) + "]");
    }
}

void require_1d(const FloatArray& array, const char* name) {
    const py::buffer_info info = array.request();
    if (info.ndim != 1) {
        throw std::invalid_argument(std::string(name) + " must have shape [batch_size]");
    }
}

py::ssize_t batch_size(const FloatArray& array) {
    return array.request().shape[0];
}

void require_batch(const FloatArray& array, const char* name, py::ssize_t expected) {
    if (batch_size(array) != expected) {
        throw std::invalid_argument(std::string(name) + " batch dimension mismatch");
    }
}

const float* data(const FloatArray& array) {
    return static_cast<const float*>(array.request().ptr);
}

float* mutable_data(FloatArray& array) {
    return static_cast<float*>(array.request().ptr);
}

FloatArray vehicle_step(
    py::handle x_vehicle_obj,
    py::handle vehicle_wrench_obj,
    py::handle vehicle_params_obj,
    py::handle dt_obj,
    py::handle external_wrench_obj) {
    FloatArray x_vehicle = as_array(x_vehicle_obj, "x_vehicle");
    FloatArray vehicle_wrench = as_array(vehicle_wrench_obj, "vehicle_wrench");
    FloatArray vehicle_params = as_array(vehicle_params_obj, "vehicle_params");
    FloatArray dt = as_array(dt_obj, "dt");
    FloatArray external_wrench = as_array(external_wrench_obj, "external_wrench");

    require_2d(x_vehicle, "x_vehicle", kVehicleStateDim);
    const py::ssize_t n = batch_size(x_vehicle);
    require_2d(vehicle_wrench, "vehicle_wrench", kVehicleWrenchDim);
    require_2d(vehicle_params, "vehicle_params", kVehicleParamDim);
    require_1d(dt, "dt");
    require_2d(external_wrench, "external_wrench", kVehicleWrenchDim);
    require_batch(vehicle_wrench, "vehicle_wrench", n);
    require_batch(vehicle_params, "vehicle_params", n);
    require_batch(dt, "dt", n);
    require_batch(external_wrench, "external_wrench", n);

    FloatArray out({n, kVehicleStateDim});
    uvms_cpu::VehicleStepHostPointers ptrs{
        data(x_vehicle), data(vehicle_wrench), data(vehicle_params), data(dt), data(external_wrench), mutable_data(out)};
    uvms_cpu::step_vehicle(ptrs, static_cast<int>(n));
    return out;
}

FloatArray arm_step(
    py::handle x_arm_obj,
    py::handle arm_torque_obj,
    py::handle dt_obj,
    py::handle arm_params_obj,
    py::handle ee_mass_obj,
    py::handle ee_damping_obj,
    py::handle ee_stiffness_obj,
    py::handle lock_mask_obj,
    py::handle baumgarte_alpha_obj) {
    FloatArray x_arm = as_array(x_arm_obj, "x_arm");
    FloatArray arm_torque = as_array(arm_torque_obj, "arm_torque");
    FloatArray dt = as_array(dt_obj, "dt");
    FloatArray arm_params = as_array(arm_params_obj, "arm_params");
    FloatArray ee_mass = as_array(ee_mass_obj, "ee_mass");
    FloatArray ee_damping = as_array(ee_damping_obj, "ee_damping");
    FloatArray ee_stiffness = as_array(ee_stiffness_obj, "ee_stiffness");
    FloatArray lock_mask = as_array(lock_mask_obj, "lock_mask");
    FloatArray baumgarte_alpha = as_array(baumgarte_alpha_obj, "baumgarte_alpha");

    require_2d(x_arm, "x_arm", kArmStateDim);
    const py::ssize_t n = batch_size(x_arm);
    require_2d(arm_torque, "arm_torque", kArmTorqueDim);
    require_1d(dt, "dt");
    require_2d(arm_params, "arm_params", kArmParamDim);
    require_1d(ee_mass, "ee_mass");
    require_1d(ee_damping, "ee_damping");
    require_1d(ee_stiffness, "ee_stiffness");
    require_2d(lock_mask, "lock_mask", kLockMaskDim);
    require_1d(baumgarte_alpha, "baumgarte_alpha");
    require_batch(arm_torque, "arm_torque", n);
    require_batch(dt, "dt", n);
    require_batch(arm_params, "arm_params", n);
    require_batch(ee_mass, "ee_mass", n);
    require_batch(ee_damping, "ee_damping", n);
    require_batch(ee_stiffness, "ee_stiffness", n);
    require_batch(lock_mask, "lock_mask", n);
    require_batch(baumgarte_alpha, "baumgarte_alpha", n);

    FloatArray out({n, kArmStateDim});
    uvms_cpu::ArmStepHostPointers ptrs{
        data(x_arm), data(arm_torque), data(dt), data(arm_params), data(ee_mass), data(ee_damping),
        data(ee_stiffness), data(lock_mask), data(baumgarte_alpha), mutable_data(out)};
    uvms_cpu::step_arm(ptrs, static_cast<int>(n));
    return out;
}

FloatArray uvms_step(
    py::handle x_uvms_obj,
    py::handle vehicle_wrench_obj,
    py::handle arm_torque_obj,
    py::handle vehicle_params_obj,
    py::handle arm_params_obj,
    py::handle dt_obj,
    py::handle external_wrench_obj,
    py::handle ee_mass_obj,
    py::handle ee_damping_obj,
    py::handle ee_stiffness_obj,
    py::handle lock_mask_obj,
    py::handle baumgarte_alpha_obj) {
    FloatArray x_uvms = as_array(x_uvms_obj, "x_uvms");
    FloatArray vehicle_wrench = as_array(vehicle_wrench_obj, "vehicle_wrench");
    FloatArray arm_torque = as_array(arm_torque_obj, "arm_torque");
    FloatArray vehicle_params = as_array(vehicle_params_obj, "vehicle_params");
    FloatArray arm_params = as_array(arm_params_obj, "arm_params");
    FloatArray dt = as_array(dt_obj, "dt");
    FloatArray external_wrench = as_array(external_wrench_obj, "external_wrench");
    FloatArray ee_mass = as_array(ee_mass_obj, "ee_mass");
    FloatArray ee_damping = as_array(ee_damping_obj, "ee_damping");
    FloatArray ee_stiffness = as_array(ee_stiffness_obj, "ee_stiffness");
    FloatArray lock_mask = as_array(lock_mask_obj, "lock_mask");
    FloatArray baumgarte_alpha = as_array(baumgarte_alpha_obj, "baumgarte_alpha");

    require_2d(x_uvms, "x_uvms", kUvmsStateDim);
    const py::ssize_t n = batch_size(x_uvms);
    require_2d(vehicle_wrench, "vehicle_wrench", kVehicleWrenchDim);
    require_2d(arm_torque, "arm_torque", kArmTorqueDim);
    require_2d(vehicle_params, "vehicle_params", kVehicleParamDim);
    require_2d(arm_params, "arm_params", kArmParamDim);
    require_1d(dt, "dt");
    require_2d(external_wrench, "external_wrench", kVehicleWrenchDim);
    require_1d(ee_mass, "ee_mass");
    require_1d(ee_damping, "ee_damping");
    require_1d(ee_stiffness, "ee_stiffness");
    require_2d(lock_mask, "lock_mask", kLockMaskDim);
    require_1d(baumgarte_alpha, "baumgarte_alpha");
    for (const auto& item : {
        std::pair<const FloatArray*, const char*>{&vehicle_wrench, "vehicle_wrench"},
        {&arm_torque, "arm_torque"}, {&vehicle_params, "vehicle_params"}, {&arm_params, "arm_params"},
        {&dt, "dt"}, {&external_wrench, "external_wrench"}, {&ee_mass, "ee_mass"},
        {&ee_damping, "ee_damping"}, {&ee_stiffness, "ee_stiffness"}, {&lock_mask, "lock_mask"},
        {&baumgarte_alpha, "baumgarte_alpha"}}) {
        require_batch(*item.first, item.second, n);
    }

    FloatArray out({n, kUvmsStateDim});
    uvms_cpu::UvmsStepHostPointers ptrs{
        data(x_uvms), data(vehicle_wrench), data(arm_torque), data(vehicle_params), data(arm_params),
        data(dt), data(external_wrench), data(ee_mass), data(ee_damping), data(ee_stiffness),
        data(lock_mask), data(baumgarte_alpha), mutable_data(out)};
    uvms_cpu::step_uvms(ptrs, static_cast<int>(n));
    return out;
}

}  // namespace

PYBIND11_MODULE(_uvms_cpu_dynamics, module) {
    module.doc() = "CPU CasADi batch UVMS dynamics.";
    module.attr("VEHICLE_STATE_DIM") = py::int_(kVehicleStateDim);
    module.attr("ARM_STATE_DIM") = py::int_(kArmStateDim);
    module.attr("UVMS_STATE_DIM") = py::int_(kUvmsStateDim);
    module.attr("VEHICLE_WRENCH_DIM") = py::int_(kVehicleWrenchDim);
    module.attr("ARM_TORQUE_DIM") = py::int_(kArmTorqueDim);
    module.attr("VEHICLE_PARAM_DIM") = py::int_(kVehicleParamDim);
    module.attr("ARM_PARAM_DIM") = py::int_(kArmParamDim);
    module.def("vehicle_step", &vehicle_step);
    module.def("arm_step", &arm_step);
    module.def("uvms_step", &uvms_step);
}
