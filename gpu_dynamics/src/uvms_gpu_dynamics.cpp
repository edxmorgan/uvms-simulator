// Author: edward morgan

#include "ros2_control_blue_reach_5/gpu_dynamics/uvms_gpu_dynamics.h"

#include <vector>

#include "casadi_on_gpu_api.h"

namespace uvms_gpu {
namespace {

std::uintptr_t as_uintptr(const float* ptr) {
    return reinterpret_cast<std::uintptr_t>(ptr);
}

std::uintptr_t as_uintptr(float* ptr) {
    return reinterpret_cast<std::uintptr_t>(ptr);
}

}  // namespace

void launch_vehicle_step(const VehicleStepDevicePointers& ptrs,
                         int batch_size,
                         int threads_per_block,
                         std::uintptr_t stream_ptr,
                         bool sync) {
    const std::vector<std::uintptr_t> inputs = {
        as_uintptr(ptrs.x_vehicle),
        as_uintptr(ptrs.vehicle_wrench),
        as_uintptr(ptrs.vehicle_params),
        as_uintptr(ptrs.dt),
        as_uintptr(ptrs.external_wrench),
    };
    const std::vector<std::uintptr_t> outputs = {as_uintptr(ptrs.x_vehicle_next)};
    casadi_on_gpu::launch(
        "Vnext_reg", inputs, outputs, batch_size, threads_per_block, stream_ptr, sync);
}

void launch_arm_step(const ArmStepDevicePointers& ptrs,
                     int batch_size,
                     int threads_per_block,
                     std::uintptr_t stream_ptr,
                     bool sync) {
    const std::vector<std::uintptr_t> inputs = {
        as_uintptr(ptrs.x_arm),
        as_uintptr(ptrs.arm_torque),
        as_uintptr(ptrs.dt),
        as_uintptr(ptrs.arm_params),
        as_uintptr(ptrs.ee_mass),
        as_uintptr(ptrs.ee_damping),
        as_uintptr(ptrs.ee_stiffness),
        as_uintptr(ptrs.lock_mask),
        as_uintptr(ptrs.baumgarte_alpha),
    };
    const std::vector<std::uintptr_t> outputs = {as_uintptr(ptrs.x_arm_next)};
    casadi_on_gpu::launch(
        "Mnext_reg", inputs, outputs, batch_size, threads_per_block, stream_ptr, sync);
}

void device_synchronize() {
    casadi_on_gpu::device_synchronize();
}

}  // namespace uvms_gpu
