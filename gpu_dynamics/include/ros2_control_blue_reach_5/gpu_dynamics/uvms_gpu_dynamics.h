#pragma once

// Author: edward morgan

#include <cstdint>

namespace uvms_gpu {

struct VehicleStepDevicePointers {
    const float* x_vehicle;
    const float* vehicle_wrench;
    const float* vehicle_params;
    const float* dt;
    const float* external_wrench;
    float* x_vehicle_next;
};

struct ArmStepDevicePointers {
    const float* x_arm;
    const float* arm_torque;
    const float* dt;
    const float* arm_params;
    const float* ee_mass;
    const float* ee_damping;
    const float* ee_stiffness;
    const float* lock_mask;
    const float* baumgarte_alpha;
    float* x_arm_next;
};

// Full UVMS dynamics is exposed as separate vehicle and arm batch kernels.
void launch_vehicle_step(const VehicleStepDevicePointers& ptrs,
                         int batch_size,
                         int threads_per_block = 256,
                         std::uintptr_t stream_ptr = 0,
                         bool sync = false);

void launch_arm_step(const ArmStepDevicePointers& ptrs,
                     int batch_size,
                     int threads_per_block = 256,
                     std::uintptr_t stream_ptr = 0,
                     bool sync = false);

void launch_prepare_arm_step(const float* x_arm,
                             float* arm_torque,
                             float* lock_mask,
                             int batch_size,
                             int threads_per_block = 256,
                             std::uintptr_t stream_ptr = 0,
                             bool sync = false);

void launch_clamp_arm_state(float* x_arm,
                            int batch_size,
                            int threads_per_block = 256,
                            std::uintptr_t stream_ptr = 0,
                            bool sync = false);

void device_synchronize();

}  // namespace uvms_gpu
